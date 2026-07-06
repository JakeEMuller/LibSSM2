// Ssm2Client.cpp -- SSM2 command set on top of IsoTpTransport.
//
// Each command serializes its request into a small stack-resident scratch
// buffer, calls m_cfg.transport->Exchange(), and validates the response code +
// length. No heap allocation; the only "internal state" we accumulate is the
// 0xAA init response so IsSupported() / CapFlags() can be queried later.
//
// Wire formats (matches FreeSSM's SSMP2communication_core.cpp):
//   Init          [AA]                                            -> [EA SSM(3) ROM(5) CAP(96)]
//   ReadBlock     [A0 pad addrH addrM addrL (N-1)]                -> [E0 data(N)]
//   ReadAddresses [A8 pad (addrH addrM addrL)*K]                  -> [E8 data(K)]
//   WriteAddress  [B8 addrH addrM addrL value]                    -> [F8 value]
//   WriteBlock    [B0 addrH addrM addrL data(N)]                  -> [F0 data(N)]
//
// `pad` is 0x00 for single-shot. The streaming variant (bit 0 set) is
// reserved for the deferred continuous-mode implementation.

#include "subiediag/Ssm2.h"
#include "subiediag/SsmBaseTable.h"

#include <string.h>

namespace subiediag::ssm2
{

    namespace
    {

        constexpr uint8_t c_padSingleShot = 0x00;

        // Decoded properties of a parameter's wire encoding, derived from
        // its eStorageType. Used by ReadParameters to drive byte-count
        // gather sizing and post-read reassembly (signed vs unsigned vs
        // IEEE-754).
        struct tParameterType
        {
            uint8_t byteCount;  // 0 == Unknown / invalid
            bool    isSigned;
            bool    isFloat;
        };

        // Map an eStorageType to its descriptor. Unknown maps to {0,...}
        // which ReadParameters rejects as InvalidFrame.
        constexpr tParameterType GetParameterType(eStorageType s) noexcept
        {
            switch (s)
            {
            case eStorageType::Uint8:
                return {1, false, false};
            case eStorageType::Uint16:
                return {2, false, false};
            case eStorageType::Uint32:
                return {4, false, false};
            case eStorageType::Int8:
                return {1, true, false};
            case eStorageType::Int16:
                return {2, true, false};
            case eStorageType::Int32:
                return {4, true, false};
            case eStorageType::Float:
                return {4, false, true};
            case eStorageType::Unknown:
                return {0, false, false};
            }
            return {0, false, false};
        }

        // Reassemble `byteCount` big-endian bytes into a double, honoring
        // signed sign-extension and IEEE-754 reinterpretation for Float.
        double ReassembleRaw(const uint8_t *bytes, tParameterType info) noexcept
        {
            // Pack into a uint32 big-endian (max width we support is 4).
            uint32_t u = 0;
            for (uint8_t i = 0; i < info.byteCount; ++i)
            {
                u = (u << 8) | bytes[i];
            }

            if (info.isFloat)
            {
                // bit_cast u -> float -> double, preserving the wire encoding.
                float f = 0.0f;
                memcpy(&f, &u, sizeof(f));
                return static_cast<double>(f);
            }

            if (info.isSigned)
            {
                // Sign-extend from byteCount*8 bits to 32, then to int64.
                const uint32_t signBit = 1U << (info.byteCount * 8 - 1);
                if (u & signBit)
                {
                    const uint32_t mask = (info.byteCount == 4) ? 0xFFFFFFFFU : ((1U << (info.byteCount * 8)) - 1U);
                    // Sign-extend by setting all bits above the value bits.
                    u |= ~mask;
                }
                return static_cast<double>(static_cast<int32_t>(u));
            }

            return static_cast<double>(u);
        }

        // Acknowledgment string required to enable Write* commands. Kept here
        // (not in the public header) so callers must type the phrase
        // literally in their code, making the intent visible at code review.
        constexpr const char *c_writeUnlockPhrase = "I UNDERSTAND THIS CAN DAMAGE MY VEHICLE";

        // Pack a 24-bit address into 3 big-endian bytes at `out[0..2]`.
        // Address bits above bit 23 are ignored by the caller's responsibility
        // (the public API validates).
        void PackAddr24(uint8_t *out, uint32_t addr) noexcept
        {
            out[0] = static_cast<uint8_t>((addr >> 16) & 0xFF);
            out[1] = static_cast<uint8_t>((addr >> 8) & 0xFF);
            out[2] = static_cast<uint8_t>(addr & 0xFF);
        }

    }  // namespace

    // ---------------------------------------------------------------------------

    Ssm2Client::Ssm2Client(const tConfig &cfg) noexcept
        : m_cfg(cfg)
    {
    }

    uint32_t Ssm2Client::EffectiveTimeoutMs(uint32_t timeoutMs) const noexcept
    {
        return timeoutMs == 0 ? m_cfg.defaultTimeoutMs : timeoutMs;
    }

    // -------- Init -------------------------------------------------------------

    eStatus Ssm2Client::Init(tInitResponse *out, uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        if (out == nullptr)
        {
            return eStatus::InvalidFrame;
        }

        uint8_t req[1];
        req[0]                         = static_cast<uint8_t>(eSsm2Cmd::Init);

        constexpr size_t c_initRespLen = 1 + c_ssmIdLen + c_romIdLen + c_capFlagsLen;
        uint8_t          resp[c_initRespLen];
        size_t           respLen = 0;

        const eStatus s          = m_cfg.transport->Exchange(req, sizeof(req), resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }

        if (respLen != c_initRespLen)
        {
            return eStatus::ProtocolError;
        }
        if (static_cast<eSsm2Rsp>(resp[0]) != eSsm2Rsp::Init)
        {
            return eStatus::ProtocolError;
        }

        memcpy(out->ssmId, &resp[1], c_ssmIdLen);
        memcpy(out->romId, &resp[1 + c_ssmIdLen], c_romIdLen);
        memcpy(out->capFlags, &resp[1 + c_ssmIdLen + c_romIdLen], c_capFlagsLen);

        m_init        = *out;
        m_initialized = true;
        return eStatus::Ok;
    }

    // -------- Connect (Open + Init) --------------------------------------------

    eStatus Ssm2Client::Connect(tInitResponse *out, uint32_t timeoutMs)
    {
        if (m_cfg.transport == nullptr || m_cfg.transport->Bus() == nullptr)
        {
            return eStatus::BackendUnavailable;
        }
        const eStatus s = m_cfg.transport->Bus()->Open();
        if (!IsOk(s))
        {
            return s;
        }
        return Init(out, timeoutMs);
    }

    // -------- ReadAddresses (0xA8 single-shot) ---------------------------------

    eStatus Ssm2Client::ReadAddresses(const uint32_t *addrs, size_t addrCount, uint8_t *out, uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        if (addrs == nullptr || out == nullptr)
        {
            return eStatus::InvalidFrame;
        }
        if (addrCount == 0 || addrCount > c_maxAddrsPerRead)
        {
            return eStatus::Overrun;
        }

        // Request layout: [A8] [pad] [addrH addrM addrL]*K
        uint8_t req[2 + 3 * c_maxAddrsPerRead];
        req[0]              = static_cast<uint8_t>(eSsm2Cmd::ReadAddresses);
        req[1]              = c_padSingleShot;
        const size_t reqLen = 2 + 3 * addrCount;
        for (size_t i = 0; i < addrCount; ++i)
        {
            if (addrs[i] > 0xFFFFFF)
            {
                return eStatus::InvalidFrame;
            }
            PackAddr24(&req[2 + 3 * i], addrs[i]);
        }

        // Response: [E8] data(K)
        uint8_t resp[1 + c_maxAddrsPerRead];
        size_t  respLen = 0;

        const eStatus s = m_cfg.transport->Exchange(req, reqLen, resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }

        if (respLen != 1 + addrCount)
        {
            return eStatus::ProtocolError;
        }
        if (static_cast<eSsm2Rsp>(resp[0]) != eSsm2Rsp::ReadAddresses)
        {
            return eStatus::ProtocolError;
        }

        memcpy(out, &resp[1], addrCount);
        return eStatus::Ok;
    }

    // -------- ReadBlock (0xA0) -------------------------------------------------

    eStatus Ssm2Client::ReadBlock(uint32_t startAddr, uint8_t *out, size_t outLen, uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        if (out == nullptr)
        {
            return eStatus::InvalidFrame;
        }
        if (outLen == 0 || outLen > c_maxReadBlockSize)
        {
            return eStatus::Overrun;
        }
        if (startAddr > 0xFFFFFF)
        {
            return eStatus::InvalidFrame;
        }

        // Request: [A0] [pad] [addrH addrM addrL] [outLen-1]
        uint8_t req[6];
        req[0] = static_cast<uint8_t>(eSsm2Cmd::ReadBlock);
        req[1] = c_padSingleShot;
        PackAddr24(&req[2], startAddr);
        req[5] = static_cast<uint8_t>(outLen - 1);

        // Response: [E0] data(outLen)
        uint8_t resp[1 + c_maxReadBlockSize];
        size_t  respLen = 0;

        const eStatus s = m_cfg.transport->Exchange(req, sizeof(req), resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }

        if (respLen != 1 + outLen)
        {
            return eStatus::ProtocolError;
        }
        if (static_cast<eSsm2Rsp>(resp[0]) != eSsm2Rsp::ReadBlock)
        {
            return eStatus::ProtocolError;
        }

        memcpy(out, &resp[1], outLen);
        return eStatus::Ok;
    }

    // -------- WriteAddress (0xB8) ----------------------------------------------

    eStatus Ssm2Client::WriteAddress(uint32_t addr, uint8_t value, uint32_t timeoutMs)
    {
        if (!m_writesUnlocked)
        {
            return eStatus::NotSupported;
        }
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        if (addr > 0xFFFFFF)
        {
            return eStatus::InvalidFrame;
        }

        // Request: [B8] [addrH addrM addrL] [value]
        uint8_t req[5];
        req[0] = static_cast<uint8_t>(eSsm2Cmd::WriteAddress);
        PackAddr24(&req[1], addr);
        req[4] = value;

        // Response: [F8] [value-echo]
        uint8_t resp[2];
        size_t  respLen = 0;

        const eStatus s = m_cfg.transport->Exchange(req, sizeof(req), resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }

        if (respLen != sizeof(resp))
        {
            return eStatus::ProtocolError;
        }
        if (static_cast<eSsm2Rsp>(resp[0]) != eSsm2Rsp::WriteAddress)
        {
            return eStatus::ProtocolError;
        }
        if (resp[1] != value)
        {
            return eStatus::ProtocolError;
        }

        return eStatus::Ok;
    }

    // -------- WriteBlock (0xB0) ------------------------------------------------

    eStatus Ssm2Client::WriteBlock(uint32_t startAddr, const uint8_t *data, size_t dataLen, uint32_t timeoutMs)
    {
        if (!m_writesUnlocked)
        {
            return eStatus::NotSupported;
        }
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        if (data == nullptr)
        {
            return eStatus::InvalidFrame;
        }
        if (dataLen == 0 || dataLen > c_maxWriteBlockSize)
        {
            return eStatus::Overrun;
        }
        if (startAddr > 0xFFFFFF)
        {
            return eStatus::InvalidFrame;
        }

        // Request: [B0] [addrH addrM addrL] data(N)
        uint8_t req[4 + c_maxWriteBlockSize];
        req[0] = static_cast<uint8_t>(eSsm2Cmd::WriteBlock);
        PackAddr24(&req[1], startAddr);
        memcpy(&req[4], data, dataLen);
        const size_t reqLen = 4 + dataLen;

        // Response: [F0] data-echo(N)
        uint8_t resp[1 + c_maxWriteBlockSize];
        size_t  respLen = 0;

        const eStatus s = m_cfg.transport->Exchange(req, reqLen, resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }

        if (respLen != 1 + dataLen)
        {
            return eStatus::ProtocolError;
        }
        if (static_cast<eSsm2Rsp>(resp[0]) != eSsm2Rsp::WriteBlock)
        {
            return eStatus::ProtocolError;
        }
        if (memcmp(&resp[1], data, dataLen) != 0)
        {
            return eStatus::ProtocolError;
        }

        return eStatus::Ok;
    }

    // -------- Continuous mode (deferred) ---------------------------------------

    eStatus Ssm2Client::StartContinuous(const uint32_t * /*addrs*/, size_t /*addrCount*/)
    {
        return eStatus::NotSupported;
    }

    eStatus Ssm2Client::PollContinuous(uint8_t * /*out*/, size_t /*outCapacity*/, size_t * /*outRecords*/, uint32_t /*timeoutMs*/)
    {
        return eStatus::NotSupported;
    }

    eStatus Ssm2Client::StopContinuous()
    {
        return eStatus::NotSupported;
    }

    // -------- write protection -------------------------------------------------

    bool Ssm2Client::UnlockWrites(const char *acknowledgment) noexcept
    {
        if (acknowledgment == nullptr)
        {
            return false;
        }
        if (strcmp(acknowledgment, c_writeUnlockPhrase) != 0)
        {
            return false;
        }
        m_writesUnlocked = true;
        return true;
    }

    void Ssm2Client::LockWrites() noexcept
    {
        m_writesUnlocked = false;
    }

    // -------- post-init accessor -----------------------------------------------

    bool Ssm2Client::IsSupported(uint8_t capByte, uint8_t capBit) const noexcept
    {
        if (!m_initialized)
        {
            return false;
        }
        if (capByte == 0 || capByte > c_capFlagsLen)
        {
            return false;
        }
        if (capBit == 0 || capBit > 8)
        {
            return false;
        }
        return (m_init.capFlags[capByte - 1] & (1 << (capBit - 1))) != 0;
    }

    bool Ssm2Client::IsSupported(const tParameter &param) const noexcept
    {
        // Non-gated parameters have no cap-flag bit; the table doesn't
        // know their per-ROM availability, so we report "not supported"
        // conservatively. The integer overload below handles the
        // not-initialized and out-of-range cases.
        if (!param.cap.Gated())
        {
            return false;
        }
        return IsSupported(param.cap.byte, param.cap.bit);
    }

    // -------- ReadParameters (batched A8 + decode) -----------------------------

    eStatus Ssm2Client::ReadParameters(const tParameter *const *params, size_t paramCount, double *outValues, uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        if (params == nullptr || outValues == nullptr || paramCount == 0)
        {
            return eStatus::InvalidFrame;
        }

        // Flatten parameters into a byte-address gather list. Validate each
        // parameter as we go: null pointer, Unknown storage, and offset==0
        // (alts-only) are all rejected.
        uint32_t addrs[c_maxAddrsPerRead];
        size_t   totalBytes = 0;
        size_t   paramOffset[c_maxAddrsPerRead];  // outValues[i] starts at addrs[paramOffset[i]]
        // ^ at most one paramOffset entry per parameter; in the worst case
        // every parameter is 1 byte so paramCount <= totalBytes <= cap.

        if (paramCount > c_maxAddrsPerRead)
        {
            return eStatus::Overrun;  // even 1-byte parameters would overflow
        }

        for (size_t i = 0; i < paramCount; ++i)
        {
            const tParameter *p = params[i];
            if (p == nullptr || p->offset == 0)
            {
                return eStatus::InvalidFrame;
            }
            const tParameterType info = GetParameterType(p->storage);
            if (info.byteCount == 0)
            {
                return eStatus::InvalidFrame;
            }
            if (totalBytes + info.byteCount > c_maxAddrsPerRead)
            {
                return eStatus::Overrun;
            }
            paramOffset[i] = totalBytes;
            for (uint8_t b = 0; b < info.byteCount; ++b)
            {
                addrs[totalBytes++] = p->offset + b;
            }
        }

        // One round trip.
        uint8_t       bytes[c_maxAddrsPerRead];
        const eStatus s = ReadAddresses(addrs, totalBytes, bytes, timeoutMs);
        if (!IsOk(s))
        {
            return s;
        }

        // Reassemble and decode per parameter.
        for (size_t i = 0; i < paramCount; ++i)
        {
            const tParameter    *p    = params[i];
            const tParameterType info = GetParameterType(p->storage);
            double               raw  = ReassembleRaw(&bytes[paramOffset[i]], info);

            // Switch/bool parameter: the read byte packs up to 8 independent
            // signals, one per bit. Mask down to just this parameter's bit
            // (1-based) so the value is a clean 0/1 instead of the whole byte.
            if (p->dataBit != 0)
            {
                const uint32_t whole = static_cast<uint32_t>(raw);
                raw = (whole & (1U << (p->dataBit - 1))) ? 1.0 : 0.0;
            }

            outValues[i] = p->convert.linear ? (raw * p->convert.scale + p->convert.offset) : raw;
        }

        return eStatus::Ok;
    }

    // -------- FindParameterByName / ByAddress ----------------------------------

    const tParameter *FindParameterByName(std::string_view name) noexcept
    {
        for (const auto &p : c_baseTable)
        {
            if (p.name == name)
            {
                return &p;
            }
        }
        return nullptr;
    }

    const tParameter *FindParameterByAddress(uint32_t addr) noexcept
    {
        for (const auto &p : c_baseTable)
        {
            if (p.offset == addr)
            {
                return &p;
            }
        }
        return nullptr;
    }

}  // namespace subiediag::ssm2
