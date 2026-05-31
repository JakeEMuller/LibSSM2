// Obd2Client.cpp -- OBD-II command set on top of IsoTpTransport.
//
// Wire formats (SAE J1979 / ISO 15031-5:2015 Section 8 for ISO 15765-4):
//   Mode 01    [01 PID]                           -> [41 PID data...]
//   Mode 03    [03]                               -> [43 #DTC (DTC1) ... ]
//                                                    (legacy ECUs may omit #DTC; see eObd2DtcFormat)
//   Mode 04    [04]                               -> [44]
//   Mode 09/02 [09 02]                            -> [49 02 NODI data(17)]
//
// Each DTC is 2 bytes: top 2 bits = category (P/C/B/U), low 14 bits = code.

#include "subiediag/Obd2.h"
#include "subiediag/Obd2DtcDb.h"
#include "subiediag/Obd2PidTable.h"

#include <string.h>

namespace subiediag
{

    // ---------------------------------------------------------------------------
    // DtcToString
    // ---------------------------------------------------------------------------

    void DtcToString(tDtc dtc, char *out, size_t outCapacity) noexcept
    {
        if (out == nullptr || outCapacity < 6)
        {
            return;
        }
        constexpr char categoryChars[] = "PCBU";
        constexpr char hexChars[]      = "0123456789ABCDEF";
        out[0]                         = categoryChars[static_cast<size_t>(dtc.category) & 0x03];
        out[1]                         = hexChars[(dtc.code >> 12) & 0x0F];
        out[2]                         = hexChars[(dtc.code >> 8) & 0x0F];
        out[3]                         = hexChars[(dtc.code >> 4) & 0x0F];
        out[4]                         = hexChars[dtc.code & 0x0F];
        out[5]                         = '\0';
    }

    // ---------------------------------------------------------------------------
    // DescribeDtc
    // ---------------------------------------------------------------------------

    std::string_view DescribeDtc(tDtc dtc) noexcept
    {
        // Linear scan over the sorted table. ~200 entries; fine for this use case.
        // Could binary-search if/when the table grows much larger.
        for (const auto &row : c_obd2DtcDb)
        {
            if (row.category == dtc.category && row.code == dtc.code)
            {
                return row.description;
            }
        }
        return {};
    }

    size_t DescribeDtc(tDtc dtc, char *out, size_t outCapacity) noexcept
    {
        if (out == nullptr || outCapacity == 0)
        {
            return 0;
        }
        const std::string_view desc = DescribeDtc(dtc);
        if (desc.empty())
        {
            out[0] = '\0';
            return 0;
        }
        const size_t copy = desc.size() < outCapacity - 1 ? desc.size() : outCapacity - 1;
        memcpy(out, desc.data(), copy);
        out[copy] = '\0';
        return copy;
    }

    // ---------------------------------------------------------------------------
    // PID table helpers
    // ---------------------------------------------------------------------------

    const tObd2PidInfo *FindObd2Pid(uint8_t pid) noexcept
    {
        // Linear scan -- table is small (~25 entries). Could binary-search since
        // entries are sorted, but the constant factor doesn't matter here.
        for (const auto &row : c_obd2PidTable)
        {
            if (row.pid == pid)
            {
                return &row;
            }
        }
        return nullptr;
    }

    bool Obd2DecodePid(uint8_t pid, const uint8_t *raw, size_t rawLen, double *outValue) noexcept
    {
        if (raw == nullptr || outValue == nullptr)
        {
            return false;
        }
        const tObd2PidInfo *info = FindObd2Pid(pid);
        if (info == nullptr || rawLen < info->bytes)
        {
            return false;
        }
        // Parse the first `bytes` bytes as big-endian unsigned.
        uint32_t rawInt = 0;
        for (uint8_t i = 0; i < info->bytes; ++i)
        {
            rawInt = (rawInt << 8) | raw[i];
        }
        *outValue = static_cast<double>(rawInt) * static_cast<double>(info->scale) + static_cast<double>(info->offset);
        return true;
    }

    // ---------------------------------------------------------------------------
    // Obd2Client
    // ---------------------------------------------------------------------------

    Obd2Client::Obd2Client(const tConfig &cfg) noexcept
        : m_cfg(cfg)
    {
    }

    uint32_t Obd2Client::EffectiveTimeoutMs(uint32_t timeoutMs) const noexcept
    {
        return timeoutMs == 0 ? m_cfg.defaultTimeoutMs : timeoutMs;
    }

    eStatus Obd2Client::ExchangeWithPendingRetry(const uint8_t *req,
                                                 size_t         reqLen,
                                                 uint8_t       *resp,
                                                 size_t         respCap,
                                                 size_t        *respLen,
                                                 uint32_t       timeoutMs)
    {
        IsoTpTransport *t = m_cfg.transport;

        eStatus s         = t->SendRequest(req, reqLen, timeoutMs);
        if (!IsOk(s))
        {
            return s;
        }

        // Loop receives until we get a non-pending response. The ECU may
        // send 0x7F <SID> 0x78 one or more times while it works on the
        // request; each one re-arms our wait to P2*CAN_max.
        uint32_t waitMs = timeoutMs;
        while (true)
        {
            s = t->ReceiveResponse(resp, respCap, respLen, waitMs);
            if (!IsOk(s))
            {
                return s;
            }
            if (*respLen >= 3 && resp[0] == c_obd2NegRespSid && resp[2] == c_obd2NrcRcrRp)
            {
                waitMs = c_obd2RcrRpTimeoutMs;
                continue;
            }
            return eStatus::Ok;
        }
    }

    // -------- Connect ----------------------------------------------------------

    eStatus Obd2Client::Connect(uint32_t timeoutMs)
    {
        if (m_cfg.transport == nullptr || m_cfg.transport->Bus() == nullptr)
        {
            return eStatus::BackendUnavailable;
        }
        eStatus s = m_cfg.transport->Bus()->Open();
        if (!IsOk(s))
        {
            return s;
        }

        // Discover supported PIDs 0x01..0x20 via Mode 01 PID 0x00.
        uint8_t bitmap[4] = {};
        size_t  len       = 0;
        s = ReadPid(static_cast<uint8_t>(ePid::SupportedPids01_20), bitmap, sizeof(bitmap), &len, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }
        if (len != 4)
        {
            return eStatus::ProtocolError;
        }
        m_supportedPids00 = (static_cast<uint32_t>(bitmap[0]) << 24) | (static_cast<uint32_t>(bitmap[1]) << 16)
                            | (static_cast<uint32_t>(bitmap[2]) << 8) | static_cast<uint32_t>(bitmap[3]);
        m_connected = true;
        return eStatus::Ok;
    }

    // -------- ReadPid ----------------------------------------------------------

    eStatus Obd2Client::ReadPid(ePid pid, uint8_t *out, size_t outCapacity, size_t *outLen, uint32_t timeoutMs)
    {
        return ReadPid(static_cast<uint8_t>(pid), out, outCapacity, outLen, timeoutMs);
    }

    eStatus Obd2Client::ReadPid(uint8_t pid, uint8_t *out, size_t outCapacity, size_t *outLen, uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        if (out == nullptr || outLen == nullptr)
        {
            return eStatus::InvalidFrame;
        }
        *outLen = 0;

        // Request: [01] [PID]
        uint8_t req[2];
        req[0] = static_cast<uint8_t>(eObd2Mode::CurrentData);
        req[1] = pid;

        // Response: [0x41] [PID] [data...]
        // Mode 01 responses are at most 4 data bytes for the standard PIDs,
        // but a few non-standard ones can be larger. 16 bytes is comfortable.
        uint8_t       resp[16];
        size_t        respLen = 0;
        const eStatus s       = ExchangeWithPendingRetry(req, sizeof(req), resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }
        if (respLen < 2)
        {
            return eStatus::ProtocolError;
        }
        if (static_cast<eObd2Rsp>(resp[0]) != eObd2Rsp::CurrentData)
        {
            return eStatus::ProtocolError;
        }
        if (resp[1] != pid)
        {
            return eStatus::ProtocolError;  // PID echo must match request
        }

        const size_t dataLen = respLen - 2;
        if (dataLen > outCapacity)
        {
            return eStatus::Overrun;
        }
        memcpy(out, &resp[2], dataLen);
        *outLen = dataLen;
        return eStatus::Ok;
    }

    // -------- ReadDtcs ---------------------------------------------------------

    namespace
    {

        // Decode a Mode 03 response payload into DTCs. `withCountByte`
        // selects framing:
        //   true  -- [0x43] [#DTC] [pairs...]   (ISO 15031-5 §8.3.2.2 Table 174)
        //   false -- [0x43] [pairs...]          (legacy / pre-CAN form)
        //
        // Returns:
        //   Ok            -- payload parsed cleanly into *outCount DTCs.
        //   Overrun       -- parse OK but more DTCs than outCapacity.
        //   ProtocolError -- framing did not match this variant (caller may
        //                    retry with the other variant).
        eStatus ParseMode03Response(const uint8_t *resp,
                                    size_t         respLen,
                                    bool           withCountByte,
                                    tDtc          *out,
                                    size_t         outCapacity,
                                    size_t        *outCount) noexcept
        {
            if (respLen < 1 || static_cast<eObd2Rsp>(resp[0]) != eObd2Rsp::ShowStoredDtcs)
            {
                return eStatus::ProtocolError;
            }

            size_t pairsStart;
            size_t pairsBytes;
            if (withCountByte)
            {
                if (respLen < 2)
                {
                    return eStatus::ProtocolError;
                }
                pairsStart = 2;
                pairsBytes = respLen - 2;
                if (pairsBytes % 2 != 0)
                {
                    return eStatus::ProtocolError;
                }
                if (resp[1] != pairsBytes / 2)
                {
                    return eStatus::ProtocolError;
                }
            }
            else
            {
                pairsStart = 1;
                pairsBytes = respLen - 1;
                if (pairsBytes % 2 != 0)
                {
                    return eStatus::ProtocolError;
                }
            }

            const size_t numDtcs = pairsBytes / 2;
            if (numDtcs > outCapacity)
            {
                return eStatus::Overrun;
            }
            for (size_t i = 0; i < numDtcs; ++i)
            {
                const uint8_t hi = resp[pairsStart + i * 2];
                const uint8_t lo = resp[pairsStart + i * 2 + 1];
                out[i].category  = static_cast<eDtcCategory>((hi >> 6) & 0x03);
                out[i].code      = static_cast<uint16_t>((static_cast<uint16_t>(hi & 0x3F) << 8) | lo);
            }
            *outCount = numDtcs;
            return eStatus::Ok;
        }

    }  // namespace

    eStatus Obd2Client::ReadDtcs(tDtc *out, size_t outCapacity, size_t *outCount, uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        if (out == nullptr || outCount == nullptr)
        {
            return eStatus::InvalidFrame;
        }
        *outCount = 0;

        // Request: [03]
        uint8_t req[1];
        req[0] = static_cast<uint8_t>(eObd2Mode::ShowStoredDtcs);

        // Response (ISO 15031-5:2015 §8.3.2.2 Table 174):
        //   [0x43] [#DTC] [DTC1_hi DTC1_lo] ...
        // Legacy / pre-CAN ECUs omit the count byte and stream pairs after
        // the SID. We try the spec form first and fall back on a framing
        // mismatch (ProtocolError). Overrun shortcuts -- it's a real
        // buffer-size problem, not a format ambiguity.
        uint8_t       resp[64];
        size_t        respLen = 0;
        const eStatus s       = ExchangeWithPendingRetry(req, sizeof(req), resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }

        const eStatus first = ParseMode03Response(resp, respLen, /*withCountByte=*/true, out, outCapacity, outCount);
        if (first != eStatus::ProtocolError)
        {
            return first;
        }
        return ParseMode03Response(resp, respLen, /*withCountByte=*/false, out, outCapacity, outCount);
    }

    // -------- ClearDtcs --------------------------------------------------------

    eStatus Obd2Client::ClearDtcs(uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        // Request: [04] (no args)
        uint8_t req[1];
        req[0] = static_cast<uint8_t>(eObd2Mode::ClearDtcs);

        // Response: [0x44] (no data)
        uint8_t       resp[8];
        size_t        respLen = 0;
        const eStatus s       = ExchangeWithPendingRetry(req, sizeof(req), resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }
        if (respLen < 1)
        {
            return eStatus::ProtocolError;
        }
        if (static_cast<eObd2Rsp>(resp[0]) != eObd2Rsp::ClearDtcs)
        {
            return eStatus::ProtocolError;
        }
        return eStatus::Ok;
    }

    // -------- GetVin -----------------------------------------------------------

    eStatus Obd2Client::GetVin(char *out, size_t outCapacity, size_t *outLen, uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        if (out == nullptr || outLen == nullptr)
        {
            return eStatus::InvalidFrame;
        }
        *outLen = 0;

        // Request: [09] [02]
        uint8_t req[2];
        req[0] = static_cast<uint8_t>(eObd2Mode::VehicleInfo);
        req[1] = c_obd2InfoVin;

        // Response (ISO 15031-5:2015 Section 8.9.2.4, Table 212):
        //   [0x49] [0x02] [NODI] [VIN data...]
        // NODI (Number Of Data Items) is 0x01 on CAN for VIN. Some ECUs
        // emit additional 0x00 padding before the VIN bytes; skip those.
        uint8_t       resp[32];
        size_t        respLen = 0;
        const eStatus s       = ExchangeWithPendingRetry(req, sizeof(req), resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }
        if (respLen < 3)
        {
            return eStatus::ProtocolError;
        }
        if (static_cast<eObd2Rsp>(resp[0]) != eObd2Rsp::VehicleInfo)
        {
            return eStatus::ProtocolError;
        }
        if (resp[1] != c_obd2InfoVin)
        {
            return eStatus::ProtocolError;
        }

        // Skip [49 02 NRR] header and any leading 0x00 padding bytes.
        size_t vinStart = 3;
        while (vinStart < respLen && resp[vinStart] == 0x00)
        {
            ++vinStart;
        }
        const size_t vinLen = respLen - vinStart;
        if (vinLen != c_obd2VinLength)
        {
            return eStatus::ProtocolError;
        }
        if (vinLen > outCapacity)
        {
            return eStatus::Overrun;
        }
        memcpy(out, &resp[vinStart], vinLen);
        *outLen = vinLen;
        return eStatus::Ok;
    }

    // -------- supported-PIDs accessors -----------------------------------------

    bool Obd2Client::IsPidSupported(ePid pid) const noexcept
    {
        return IsPidSupported(static_cast<uint8_t>(pid));
    }

    bool Obd2Client::IsPidSupported(uint8_t pid) const noexcept
    {
        if (!m_connected)
        {
            return false;
        }
        if (pid == 0x00)
        {
            return true;  // 0x00 itself is always queryable
        }
        if (pid > 0x20)
        {
            return false;  // cascade discovery (PID 0x20, 0x40, ...) not implemented
        }
        // Bitmap layout: byte 0 bit 7 = PID 0x01, byte 0 bit 0 = PID 0x08, ...,
        // byte 3 bit 0 = PID 0x20. Stored MSB-first as uint32 -> bit (32 - N).
        return ((m_supportedPids00 >> (32 - pid)) & 0x1U) != 0;
    }

}  // namespace subiediag
