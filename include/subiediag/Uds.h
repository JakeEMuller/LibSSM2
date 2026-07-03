// subiediag/Uds.h
//
// UDS (ISO 14229-1:2020) client on top of the existing ISO-TP transport.
//
// v1 scope is the generic, cross-manufacturer READ path:
//   0x22 ReadDataByIdentifier   -- single- and multi-DID reads
//   0x10 DiagnosticSessionControl
//   0x3E TesterPresent
//
// The headline v1 use case is the OBD-mirror DID range 0xF400..0xF4FF
// (ISO 14229-1 Table C.1 "OBDDataIdentifier"): DID 0xF4nn carries the same
// payload as OBD-II Mode 01 PID 0xnn per SAE J1979-DA, so the Obd2PidTable
// linear decoders apply unchanged (see DecodeObdDid). Vehicles built to
// SAE J1979-2 (2023+) expose their emissions data ONLY through this form.
// Manufacturer-specific DIDs (0x0100..0xEFFF etc.) can be read raw with
// the same calls; decoding those is the application's problem until a
// per-vehicle DID table exists.
//
// Read-only by design: no WriteDataByIdentifier, no SecurityAccess, no
// routine control. DTCs stay on Obd2Client (Mode 03/04) for now; UDS
// 0x19/0x14 come later if a target vehicle needs them.
//
// Unlike OBD-II there is no global "supported DIDs" bitmap -- support is
// discovered per-DID by reading it: NRC 0x31 (requestOutOfRange) means the
// server doesn't have it. Those NRCs surface as eStatus::NotSupported (see
// LastNrc()), so a probe loop can tell "car doesn't have this" apart from
// a broken bus.
//
// Threading: not thread-safe. Single-owner, same as the other clients.
// Sharing one IsoTpTransport with an Obd2Client / Ssm2Client addressing
// the same ECU is the intended pattern.
//
// Timeouts: every command accepts a timeoutMs argument. Pass 0 to use
// tConfig::defaultTimeoutMs. After an NRC 0x78 (response pending) the wait
// is re-armed to P2*server_max -- 5 000 ms by default, refined by the
// value the server reports in a DiagnosticSessionControl response.

#pragma once

#include "subiediag/Can.h"
#include "subiediag/Common.h"
#include "subiediag/IsoTp.h"

#include <stddef.h>
#include <stdint.h>

namespace subiediag::uds
{

    // ---------------------------------------------------------------------------
    // Protocol constants
    // ---------------------------------------------------------------------------

    // Service identifiers carried as the first byte of the request.
    enum class eUdsService : uint8_t
    {
        DiagnosticSessionControl = 0x10,
        ReadDataByIdentifier     = 0x22,
        TesterPresent            = 0x3E,
    };

    // Positive-response SID = request SID | 0x40 (ISO 14229-1 8.4).
    constexpr uint8_t c_positiveRspMask = 0x40;

    // diagnosticSessionType sub-function values (ISO 14229-1 Table 25).
    enum class eDiagnosticSession : uint8_t
    {
        Default          = 0x01,
        Programming      = 0x02,
        Extended         = 0x03,
        SafetySystem     = 0x04,
    };

    // SubFunction bit 7: suppressPosRspMsgIndicationBit -- the server sends
    // no positive response (negative responses still come through).
    constexpr uint8_t c_suppressPosRspBit = 0x80;

    // Negative-response markers (ISO 14229-2; OBD-II inherited the same
    // format, see subiediag::obd2).
    constexpr uint8_t c_negRespSid = 0x7F;

    // NRCs the v1 services can produce (ISO 14229-1 Table A.1, subset).
    // Exposed via UdsClient::LastNrc() after a call fails.
    constexpr uint8_t c_nrcServiceNotSupported                = 0x11;
    constexpr uint8_t c_nrcSubFunctionNotSupported            = 0x12;
    constexpr uint8_t c_nrcIncorrectMessageLength             = 0x13;
    constexpr uint8_t c_nrcResponseTooLong                    = 0x14;
    constexpr uint8_t c_nrcConditionsNotCorrect               = 0x22;
    constexpr uint8_t c_nrcRequestOutOfRange                  = 0x31;
    constexpr uint8_t c_nrcSecurityAccessDenied               = 0x33;
    constexpr uint8_t c_nrcResponsePending                    = 0x78;  // handled internally
    constexpr uint8_t c_nrcSubFunctionNotSupportedInSession   = 0x7E;
    constexpr uint8_t c_nrcServiceNotSupportedInSession       = 0x7F;

    // Default P2*server_max (the post-0x78 wait) until a session-control
    // response provides the server's real value. 5 000 ms matches the
    // ISO 15765-4 OBD figure and is a safe upper bound.
    constexpr uint32_t c_defaultP2StarMs = 5000;

    // Request sizing: one 0x22 request carries at most this many DIDs.
    // Bounds the internal scratch buffers; generous for gauge polling
    // (a dashboard's whole poll set is typically < 20).
    constexpr size_t c_maxDidsPerRead = 64;

    // Internal response scratch for 0x22. Bounds a multi-DID read's total
    // response: sum over DIDs of (2 + expectedLen) + 1 must fit.
    constexpr size_t c_readScratchLen = 512;

    // ---------------------------------------------------------------------------
    // OBD-mirror DID helpers (0xF400..0xF4FF, SAE J1979-DA)
    // ---------------------------------------------------------------------------

    constexpr uint16_t c_obdDidBase = 0xF400;

    // The DID that mirrors OBD-II Mode 01 PID `pid`.
    [[nodiscard]] constexpr uint16_t DidForObd2Pid(uint8_t pid) noexcept
    {
        return static_cast<uint16_t>(c_obdDidBase | pid);
    }

    // True iff `did` is in the 8-bit-PID OBD-mirror range 0xF400..0xF4FF.
    [[nodiscard]] constexpr bool IsObdDid(uint16_t did) noexcept
    {
        return (did & 0xFF00u) == c_obdDidBase;
    }

    // Decode an OBD-mirror DID's dataRecord using the Obd2PidTable linear
    // formula for the mirrored PID. Returns false if `did` isn't in the
    // 0xF4xx range, the PID has no table entry, or rawLen is too short.
    [[nodiscard]] bool DecodeObdDid(uint16_t did, const uint8_t *raw, size_t rawLen, double *outValue) noexcept;

    // ---------------------------------------------------------------------------
    // Types
    // ---------------------------------------------------------------------------

    // Server timing from a DiagnosticSessionControl positive response
    // (sessionParameterRecord, ISO 14229-1 Table 26).
    struct tSessionTiming
    {
        uint16_t p2ServerMaxMs;      // max time to first response, in ms
        uint16_t p2StarServerMax10;  // enhanced timeout after NRC 0x78, in 10 ms units
    };

    // One entry of a multi-DID ReadDids() call. The 0x62 response carries
    // records back-to-back with NO length delimiters, so the caller must
    // supply each DID's exact dataRecord length up front (for OBD-mirror
    // DIDs that's the mirrored PID's byte count from Obd2PidTable).
    struct tDidRead
    {
        uint16_t did;          // in: dataIdentifier to read
        uint8_t *out;          // in: caller buffer for the dataRecord
        size_t   outCapacity;  // in: bytes available at `out`
        size_t   expectedLen;  // in: exact dataRecord length (> 0)
        size_t   len;          // out: bytes written; 0 = server omitted this DID
    };

    // ---------------------------------------------------------------------------
    // Client
    // ---------------------------------------------------------------------------

    class UdsClient
    {
    public:

        struct tConfig
        {
            // Non-owning pointer to the ISO-TP transport bound to the target
            // ECU's (reqId, respId, padByte). App constructs the transport
            // and keeps it alive for the client's lifetime. The same
            // transport can be shared with an Obd2Client / Ssm2Client
            // addressing the same ECU.
            isotp::IsoTpTransport *transport        = nullptr;
            uint32_t               defaultTimeoutMs = c_defaultTimeoutMs;
        };

        explicit UdsClient(const tConfig &cfg) noexcept;

        UdsClient(const UdsClient &)            = delete;
        UdsClient(UdsClient &&)                 = delete;
        UdsClient &operator=(const UdsClient &) = delete;
        UdsClient &operator=(UdsClient &&)      = delete;

        // -------- one-shot commands --------------------------------------------

        // Open the underlying bus (via the transport) and verify a UDS
        // server answers by exchanging one TesterPresent. UDS has no
        // supported-DID bitmap to prefetch, so this is the whole handshake;
        // DID support is discovered per-DID via ReadDid (NRC 0x31 ->
        // NotSupported).
        [[nodiscard]] eStatus Connect(uint32_t timeoutMs = 0);

        // 0x10 -- switch the server's diagnostic session. On success the
        // server's timing parameters are captured (see LastSessionTiming)
        // and the post-0x78 wait is re-armed from P2*server_max; `outTiming`
        // also receives them when non-null. Emissions-related OBD-mirror
        // DIDs are readable in the Default session -- Extended is only
        // needed for manufacturer DIDs that demand it.
        [[nodiscard]] eStatus DiagnosticSessionControl(eDiagnosticSession session,
                                                       tSessionTiming    *outTiming = nullptr,
                                                       uint32_t           timeoutMs = 0);

        // 0x3E -- keep-alive. With suppressResponse the request sets the
        // suppressPosRspMsgIndicationBit and the call returns right after
        // the send (the fire-and-forget form used to hold a non-default
        // session without stealing bus time).
        [[nodiscard]] eStatus TesterPresent(bool suppressResponse = false, uint32_t timeoutMs = 0);

        // 0x22, single DID. `out` receives the dataRecord (without the
        // echoed DID); *outLen is its length -- unambiguous here because
        // the record is the whole remaining payload.
        [[nodiscard]] eStatus ReadDid(uint16_t did, uint8_t *out, size_t outCapacity, size_t *outLen, uint32_t timeoutMs = 0);

        // 0x22, multiple DIDs in one round trip (the gauge-poll fast path,
        // the UDS analogue of SSM2's block read). Every entry needs a
        // correct expectedLen -- a wrong length desynchronises the parse of
        // everything after it (ProtocolError). The server includes only the
        // DIDs it supports; omitted ones come back with len == 0 and the
        // call still returns Ok (all-unsupported yields NRC 0x31 ->
        // NotSupported instead). Entries are matched in request order.
        [[nodiscard]] eStatus ReadDids(tDidRead *reads, size_t count, uint32_t timeoutMs = 0);

        // -------- accessors ----------------------------------------------------

        bool IsConnected() const noexcept { return m_connected; }

        // NRC from the most recent negative response, 0 if the last call
        // didn't end in one. Lets a probe loop distinguish "this car
        // doesn't have that DID" (0x31) from other failures without
        // growing eStatus per-NRC.
        uint8_t LastNrc() const noexcept { return m_lastNrc; }

        // Timing captured by the last successful DiagnosticSessionControl.
        // Zeros until one succeeds.
        tSessionTiming LastSessionTiming() const noexcept { return m_sessionTiming; }

        const tConfig &Config() const noexcept { return m_cfg; }

    private:

        uint32_t EffectiveTimeoutMs(uint32_t timeoutMs) const noexcept;

        bool HasTransport() const noexcept { return m_cfg.transport != nullptr; }

        // Send + receive with 0x78 pending handling, then classify the
        // reply: positive -> Ok (m_lastNrc = 0); negative -> capture the
        // NRC and map it to a status (NotSupported for the "server doesn't
        // offer that" family, ProtocolError otherwise).
        eStatus Exchange(const uint8_t *req,
                         size_t         reqLen,
                         uint8_t       *resp,
                         size_t         respCap,
                         size_t        *respLen,
                         uint32_t       timeoutMs);

        tConfig        m_cfg;
        bool           m_connected     = false;
        uint8_t        m_lastNrc       = 0;
        uint32_t       m_p2StarWaitMs  = c_defaultP2StarMs;
        tSessionTiming m_sessionTiming = {};
    };

}  // namespace subiediag::uds
