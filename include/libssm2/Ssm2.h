// libssm2/Ssm2.h
//
// High-level SSM2 client. Exposes the diagnostic command set (init, read
// addresses, read block, write block, continuous read) on top of an
// IsoTpTransport wrapped around a non-owning ICanBus.
//
// Lifetime:
//   - The ICanBus pointed to by tConfig::bus must outlive the Ssm2Client.
//   - Construct one Ssm2Client per ECU. Multiple clients can share one bus;
//     each filters incoming frames by its own respId at the IsoTp layer.
//
// Threading: not thread-safe. Each Ssm2Client is single-owner; the app
// handles any threading model it wants on top of these synchronous calls.
//
// Timeouts: every command accepts a timeoutMs argument. Pass 0 to use
// tConfig::defaultTimeoutMs.

#pragma once

#include "libssm2/Can.h"
#include "libssm2/Common.h"
#include "libssm2/IsoTp.h"

#include <stddef.h>
#include <stdint.h>

namespace libssm2
{

    // ---------------------------------------------------------------------------
    // SSM2 protocol constants.
    // ---------------------------------------------------------------------------

    // 0xAA init response field sizes.
    constexpr size_t c_ssmIdLen    = 3;
    constexpr size_t c_romIdLen    = 5;
    constexpr size_t c_capFlagsLen = 96;

    // Standard SSM2-over-CAN (ISO 15765-4) ECU CAN IDs.
    constexpr uint32_t c_engineReqId        = 0x7E0;
    constexpr uint32_t c_engineRespId       = 0x7E8;
    constexpr uint32_t c_transmissionReqId  = 0x7E1;
    constexpr uint32_t c_transmissionRespId = 0x7E9;

    // Default round-trip deadline used when a per-call timeoutMs is 0.
    constexpr uint32_t c_defaultTimeoutMs = 500;

    // SSM2 command bytes carried as the first byte of the request payload.
    // A positive response from the ECU starts with (cmd | c_responseMask).
    enum class eSsm2Cmd : uint8_t
    {
        ReadBlock     = 0xA0,
        ReadAddresses = 0xA8,
        Init          = 0xAA,
        WriteBlock    = 0xB0,
        WriteAddress  = 0xB8,
    };

    constexpr uint8_t c_responseMask = 0x40;

    // Helper: compute the positive-response code the ECU sends back for a given
    // request command. Used to verify a response against the request that
    // triggered it.
    [[nodiscard]] constexpr uint8_t Ssm2ResponseCode(eSsm2Cmd cmd) noexcept
    {
        return static_cast<uint8_t>(cmd) | c_responseMask;
    }

    // ---------------------------------------------------------------------------
    // Types.
    // ---------------------------------------------------------------------------

    // Decoded payload of an SSM2 0xAA init response.
    //   ssmId:    diagnostic interface identifier, e.g. A2 10 02
    //   romId:    ECU ROM ID, unique per firmware revision
    //   capFlags: capability table. A parameter at (capByte, capBit) is
    //             supported iff capFlags[capByte - 1] & (1 << (capBit - 1)) != 0.
    struct tSsm2InitResponse
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
            ICanBus *bus              = nullptr;             // required, non-owning
            uint32_t reqId            = c_engineReqId;       // engine ECU request
            uint32_t respId           = c_engineRespId;      // engine ECU response
            uint32_t defaultTimeoutMs = c_defaultTimeoutMs;  // used when a call passes timeoutMs == 0
            uint8_t  padByte          = 0x00;                // fill byte for unused CAN frame positions
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
        [[nodiscard]] eStatus Init(tSsm2InitResponse *out, uint32_t timeoutMs = 0);

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

        // -------- post-init accessors ------------------------------------------

        bool IsInitialized() const noexcept { return m_initialized; }

        const tSsm2InitResponse &InitResponse() const noexcept { return m_init; }

        // Pointer to the 96-byte cap-flag array. Valid only after Init().
        const uint8_t *CapFlags() const noexcept { return m_init.capFlags; }

        // 1-based byte and bit indices, matching SsmBaseTable.h cap.byte / cap.bit.
        // Returns false before Init() has been called.
        bool IsSupported(uint8_t capByte, uint8_t capBit) const noexcept;

        const tConfig &Config() const noexcept { return m_cfg; }

    private:

        tConfig           m_cfg;
        IsoTpTransport    m_transport;
        tSsm2InitResponse m_init{};
        bool              m_initialized = false;

        bool   m_continuous             = false;
        size_t m_continuousRecordSize   = 0;

        // Fixed ring buffer for continuous-mode records.
        static constexpr size_t c_ringCapacity         = 1024;
        uint8_t                 m_ring[c_ringCapacity] = {};
        size_t                  m_ringHead             = 0;
        size_t                  m_ringTail             = 0;
    };

}  // namespace libssm2
