// subiediag/Ssm2.h
//
// High-level SSM2 client. Exposes the diagnostic command set (init, read
// addresses, read block, write block, continuous read) on top of an
// IsoTpTransport wrapped around a non-owning ICanBus.
//
// Lifetime:
//   - tConfig::transport must point at an IsoTpTransport that outlives the
//     Ssm2Client. The transport itself holds the ICanBus pointer and the
//     ECU (reqId, respId, padByte) it speaks to.
//   - Construct one Ssm2Client per ECU. Two clients on the same ECU (e.g.
//     Ssm2Client + Obd2Client on 0x7E0/0x7E8) can share one transport.
//
// Threading: not thread-safe. Each Ssm2Client is single-owner; the app
// handles any threading model it wants on top of these synchronous calls.
//
// Timeouts: every command accepts a timeoutMs argument. Pass 0 to use
// tConfig::defaultTimeoutMs.

#pragma once

#include "subiediag/Can.h"
#include "subiediag/Common.h"
#include "subiediag/IsoTp.h"

#include <stddef.h>
#include <stdint.h>

namespace subiediag::ssm2
{

    // ---------------------------------------------------------------------------
    // SSM2 protocol constants.
    // ---------------------------------------------------------------------------

    // 0xAA init response field sizes.
    constexpr size_t c_ssmIdLen    = 3;
    constexpr size_t c_romIdLen    = 5;
    constexpr size_t c_capFlagsLen = 96;

    // CAN IDs (subiediag::can::c_engineReqId etc.) live with the CAN
    // abstraction; c_defaultTimeoutMs lives at root subiediag::.

    // Per-command upper bounds. Used to size internal scratch buffers.
    constexpr size_t c_maxAddrsPerRead   = 84;   // ReadAddresses: 2 + 3*N <= ~256
    constexpr size_t c_maxReadBlockSize  = 256;  // ReadBlock count-1 fits in one byte
    constexpr size_t c_maxWriteBlockSize = 256;  // matches ReadBlock cap symmetrically

    // SSM2 command bytes carried as the first byte of the request payload.
    enum class eSsm2Cmd : uint8_t
    {
        ReadBlock     = 0xA0,
        ReadAddresses = 0xA8,
        Init          = 0xAA,
        WriteBlock    = 0xB0,
        WriteAddress  = 0xB8,
    };

    // Positive-response codes the ECU sends back, one per command.
    // Protocol convention: rsp == cmd | 0x40 (see static_asserts below).
    enum class eSsm2Rsp : uint8_t
    {
        ReadBlock     = 0xE0,
        ReadAddresses = 0xE8,
        Init          = 0xEA,
        WriteBlock    = 0xF0,
        WriteAddress  = 0xF8,
    };

    // Documented for anyone interpreting raw SSM2 traffic. Not used internally.
    constexpr uint8_t c_responseMask = 0x40;

    static_assert((static_cast<uint8_t>(eSsm2Cmd::ReadBlock) | c_responseMask) == static_cast<uint8_t>(eSsm2Rsp::ReadBlock));
    static_assert((static_cast<uint8_t>(eSsm2Cmd::ReadAddresses) | c_responseMask) == static_cast<uint8_t>(eSsm2Rsp::ReadAddresses));
    static_assert((static_cast<uint8_t>(eSsm2Cmd::Init) | c_responseMask) == static_cast<uint8_t>(eSsm2Rsp::Init));
    static_assert((static_cast<uint8_t>(eSsm2Cmd::WriteBlock) | c_responseMask) == static_cast<uint8_t>(eSsm2Rsp::WriteBlock));
    static_assert((static_cast<uint8_t>(eSsm2Cmd::WriteAddress) | c_responseMask) == static_cast<uint8_t>(eSsm2Rsp::WriteAddress));

    // ---------------------------------------------------------------------------
    // Types.
    // ---------------------------------------------------------------------------

    // Decoded payload of an SSM2 0xAA init response.
    //   ssmId:    diagnostic interface identifier, e.g. A2 10 02
    //   romId:    ECU ROM ID, unique per firmware revision
    //   capFlags: capability table. A parameter at (capByte, capBit) is
    //             supported iff capFlags[capByte - 1] & (1 << (capBit - 1)) != 0.
    struct tInitResponse
    {
        uint8_t ssmId[c_ssmIdLen];
        uint8_t romId[c_romIdLen];
        uint8_t capFlags[c_capFlagsLen];
    };

    class Ssm2Client
    {
    public:

        struct tConfig
        {
            // Non-owning pointer to the ISO-TP transport bound to this ECU's
            // (reqId, respId, padByte). The app constructs the transport and
            // must keep it alive for the lifetime of the client. Sharing one
            // transport between multiple clients addressing the same ECU
            // (e.g. Ssm2Client + Obd2Client on 0x7E0/0x7E8) is the intended
            // pattern.
            isotp::IsoTpTransport *transport        = nullptr;
            uint32_t               defaultTimeoutMs = c_defaultTimeoutMs;
        };

        explicit Ssm2Client(const tConfig &cfg) noexcept;

        Ssm2Client(const Ssm2Client &)            = delete;
        Ssm2Client(Ssm2Client &&)                 = delete;
        Ssm2Client &operator=(const Ssm2Client &) = delete;
        Ssm2Client &operator=(Ssm2Client &&)      = delete;

        // -------- one-shot commands --------------------------------------------
        //
        // SSM2 addresses are 24-bit values. The low 24 bits of each uint32_t
        // are the address; the high byte is ignored.

        // eSsm2Cmd::Init - ECU init. Populates *out with ssmId, romId, capFlags.
        [[nodiscard]] eStatus Init(tInitResponse *out, uint32_t timeoutMs = 0);

        // Convenience: opens the underlying bus (via the transport) then
        // performs Init() in one call. Equivalent to:
        //   transport->Bus()->Open();      // skipped if already open
        //   client.Init(out, timeoutMs);
        // For complex lifetimes (e.g. several clients sharing one transport
        // or one bus) keep the Open() and Init() calls separate.
        [[nodiscard]] eStatus Connect(tInitResponse *out, uint32_t timeoutMs = 0);

        // eSsm2Cmd::ReadAddresses - single-shot read of `addrCount` addresses.
        // Writes `addrCount` bytes (one per address, in order) into `out`.
        [[nodiscard]] eStatus ReadAddresses(const uint32_t *addrs, size_t addrCount, uint8_t *out, uint32_t timeoutMs = 0);

        // eSsm2Cmd::ReadBlock - sequential block read of `outLen` bytes starting
        // at startAddr.
        [[nodiscard]] eStatus ReadBlock(uint32_t startAddr, uint8_t *out, size_t outLen, uint32_t timeoutMs = 0);

        // eSsm2Cmd::WriteAddress - write a single byte to one address.
        [[nodiscard]] eStatus WriteAddress(uint32_t addr, uint8_t value, uint32_t timeoutMs = 0);

        // eSsm2Cmd::WriteBlock - sequential block write of `dataLen` bytes
        // starting at startAddr.
        [[nodiscard]] eStatus WriteBlock(uint32_t startAddr, const uint8_t *data, size_t dataLen, uint32_t timeoutMs = 0);

        // -------- continuous mode (eSsm2Cmd::ReadAddresses streaming) ----------
        //
        // After StartContinuous(), the ECU streams `addrCount` bytes per record
        // at its own pace. The library buffers complete records in a fixed-size
        // ring (see c_ringCapacity below). PollContinuous() drains as many whole
        // records as fit in `out`.
        //
        // If the app doesn't drain quickly enough, the oldest records are
        // dropped and the next poll surfaces eStatus::Overrun (without losing
        // the newer data). The app keeps polling.

        [[nodiscard]] eStatus StartContinuous(const uint32_t *addrs, size_t addrCount);

        [[nodiscard]] eStatus PollContinuous(uint8_t *out, size_t outCapacity, size_t *outRecords, uint32_t timeoutMs);

        [[nodiscard]] eStatus StopContinuous();

        bool InContinuousMode() const noexcept { return m_continuous; }

        // -------- write protection ---------------------------------------------
        //
        // SSM2 writes (0xB0 / 0xB8) modify ECU RAM in real time. They cannot
        // brick the ECU permanently (RAM, not flash), but they CAN damage the
        // engine while running by altering ignition timing, injector pulse
        // width, idle target, etc.
        //
        // WriteAddress() and WriteBlock() return eStatus::NotSupported until
        // UnlockWrites() succeeds. The acknowledgment string is intentionally
        // not exposed as a constant -- you must type it literally in your
        // code so reviewers can see the intent. Wrong / null string is a
        // no-op. LockWrites() re-locks. Returns true on successful unlock.

        bool UnlockWrites(const char *acknowledgment) noexcept;
        void LockWrites() noexcept;
        bool WritesUnlocked() const noexcept { return m_writesUnlocked; }

        // -------- post-init accessors ------------------------------------------

        bool IsInitialized() const noexcept { return m_initialized; }

        const tInitResponse &InitResponse() const noexcept { return m_init; }

        // Pointer to the 96-byte cap-flag array. Valid only after Init().
        const uint8_t *CapFlags() const noexcept { return m_init.capFlags; }

        // 1-based byte and bit indices, matching SsmBaseTable.h cap.byte / cap.bit.
        // Returns false before Init() has been called.
        bool IsSupported(uint8_t capByte, uint8_t capBit) const noexcept;

        const tConfig &Config() const noexcept { return m_cfg; }

    private:

        // Returns `timeoutMs` if non-zero, else m_cfg.defaultTimeoutMs.
        uint32_t EffectiveTimeoutMs(uint32_t timeoutMs) const noexcept;

        // True iff cfg.transport is a usable non-null pointer.
        bool HasTransport() const noexcept { return m_cfg.transport != nullptr; }

        tConfig           m_cfg;
        tInitResponse m_init{};
        bool              m_initialized = false;

        bool   m_continuous             = false;
        size_t m_continuousRecordSize   = 0;
        bool   m_writesUnlocked         = false;

        // Fixed ring buffer for continuous-mode records.
        static constexpr size_t c_ringCapacity         = 1024;
        uint8_t                 m_ring[c_ringCapacity] = {};
        size_t                  m_ringHead             = 0;
        size_t                  m_ringTail             = 0;
    };

}  // namespace subiediag::ssm2
