// UdsClient.cpp -- ISO 14229-1 v1 command set on top of IsoTpTransport.
//
// Wire formats (ISO 14229-1:2020):
//   0x10 [10 session]              -> [50 session P2hi P2lo P2*hi P2*lo]
//   0x22 [22 DIDhi DIDlo ...]      -> [62 (DIDhi DIDlo data...)+]
//   0x3E [3E 00|80]                -> [7E 00] (suppressed: no response)
//   negative                        -> [7F SID NRC]
//
// The 0x62 response carries dataRecords back-to-back with no delimiters --
// single-DID reads take the whole remainder, multi-DID reads split on the
// caller-supplied expected lengths.

#include "subiediag/Uds.h"
#include "subiediag/Obd2PidTable.h"

#include "PendingExchange.h"

#include <string.h>

namespace subiediag::uds
{

    // ---------------------------------------------------------------------------
    // DecodeObdDid
    // ---------------------------------------------------------------------------

    bool DecodeObdDid(uint16_t did, const uint8_t *raw, size_t rawLen, double *outValue) noexcept
    {
        if (!IsObdDid(did))
        {
            return false;
        }
        return obd2::DecodePid(static_cast<uint8_t>(did & 0xFFu), raw, rawLen, outValue);
    }

    // ---------------------------------------------------------------------------
    // UdsClient
    // ---------------------------------------------------------------------------

    UdsClient::UdsClient(const tConfig &cfg) noexcept
        : m_cfg(cfg)
    {
    }

    uint32_t UdsClient::EffectiveTimeoutMs(uint32_t timeoutMs) const noexcept
    {
        return timeoutMs == 0 ? m_cfg.defaultTimeoutMs : timeoutMs;
    }

    eStatus UdsClient::Exchange(const uint8_t *req, size_t reqLen, uint8_t *resp, size_t respCap, size_t *respLen, uint32_t timeoutMs)
    {
        m_lastNrc = 0;

        const eStatus s =
            detail::ExchangeWithPendingRetry(*m_cfg.transport, req, reqLen, resp, respCap, respLen, timeoutMs, m_p2StarWaitMs);
        if (!IsOk(s))
        {
            return s;
        }

        if (*respLen >= 3 && resp[0] == c_negRespSid)
        {
            m_lastNrc = resp[2];
            switch (m_lastNrc)
            {
            // The "server doesn't offer that" family -- expected during
            // per-DID support probing and session negotiation, so callers
            // get the same status the other clients use for a missing
            // capability rather than a scary protocol error.
            case c_nrcServiceNotSupported:
            case c_nrcSubFunctionNotSupported:
            case c_nrcRequestOutOfRange:
            case c_nrcSubFunctionNotSupportedInSession:
            case c_nrcServiceNotSupportedInSession:
                return eStatus::NotSupported;
            default:
                return eStatus::ProtocolError;
            }
        }
        return eStatus::Ok;
    }

    // -------- Connect ----------------------------------------------------------

    eStatus UdsClient::Connect(uint32_t timeoutMs)
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

        // Liveness handshake: any UDS server answers TesterPresent in the
        // default session, and it has no side effects. A plain-OBD-only ECU
        // times out or NRCs here, telling the caller "no UDS on this car".
        s = TesterPresent(/*suppressResponse=*/false, timeoutMs);
        if (!IsOk(s))
        {
            return s;
        }
        m_connected = true;
        return eStatus::Ok;
    }

    // -------- DiagnosticSessionControl ------------------------------------------

    eStatus UdsClient::DiagnosticSessionControl(eDiagnosticSession session, tSessionTiming *outTiming, uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }

        // Request: [10] [session]
        uint8_t req[2];
        req[0] = static_cast<uint8_t>(eUdsService::DiagnosticSessionControl);
        req[1] = static_cast<uint8_t>(session);

        // Response: [50] [session] [P2 hi lo] [P2* hi lo]
        uint8_t       resp[8];
        size_t        respLen = 0;
        const eStatus s       = Exchange(req, sizeof(req), resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }
        if (respLen < 6)
        {
            return eStatus::ProtocolError;
        }
        if (resp[0] != (static_cast<uint8_t>(eUdsService::DiagnosticSessionControl) | c_positiveRspMask))
        {
            return eStatus::ProtocolError;
        }
        if (resp[1] != static_cast<uint8_t>(session))
        {
            return eStatus::ProtocolError;  // session echo must match request
        }

        m_sessionTiming.p2ServerMaxMs     = static_cast<uint16_t>((static_cast<uint16_t>(resp[2]) << 8) | resp[3]);
        m_sessionTiming.p2StarServerMax10 = static_cast<uint16_t>((static_cast<uint16_t>(resp[4]) << 8) | resp[5]);

        // Re-arm the post-0x78 wait from the server's own P2*server_max
        // (10 ms units). A server reporting 0 keeps the safe default.
        if (m_sessionTiming.p2StarServerMax10 != 0)
        {
            m_p2StarWaitMs = static_cast<uint32_t>(m_sessionTiming.p2StarServerMax10) * 10u;
        }
        if (outTiming != nullptr)
        {
            *outTiming = m_sessionTiming;
        }
        return eStatus::Ok;
    }

    // -------- TesterPresent ------------------------------------------------------

    eStatus UdsClient::TesterPresent(bool suppressResponse, uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }

        // Request: [3E] [00] -- sub-function zeroSubFunction, with the
        // suppress bit folded in for the fire-and-forget keep-alive form.
        uint8_t req[2];
        req[0] = static_cast<uint8_t>(eUdsService::TesterPresent);
        req[1] = suppressResponse ? c_suppressPosRspBit : 0x00;

        if (suppressResponse)
        {
            // No response will come; a receive would just burn the timeout.
            return m_cfg.transport->SendRequest(req, sizeof(req), EffectiveTimeoutMs(timeoutMs));
        }

        // Response: [7E] [00]
        uint8_t       resp[8];
        size_t        respLen = 0;
        const eStatus s       = Exchange(req, sizeof(req), resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }
        if (respLen < 2)
        {
            return eStatus::ProtocolError;
        }
        if (resp[0] != (static_cast<uint8_t>(eUdsService::TesterPresent) | c_positiveRspMask))
        {
            return eStatus::ProtocolError;
        }
        return eStatus::Ok;
    }

    // -------- ReadDid (single) ----------------------------------------------------

    eStatus UdsClient::ReadDid(uint16_t did, uint8_t *out, size_t outCapacity, size_t *outLen, uint32_t timeoutMs)
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

        // Request: [22] [DID hi] [DID lo]
        uint8_t req[3];
        req[0] = static_cast<uint8_t>(eUdsService::ReadDataByIdentifier);
        req[1] = static_cast<uint8_t>(did >> 8);
        req[2] = static_cast<uint8_t>(did & 0xFFu);

        // Response: [62] [DID hi] [DID lo] [dataRecord...] -- the record is
        // everything after the echoed DID, so no length knowledge needed.
        uint8_t       resp[c_readScratchLen];
        size_t        respLen = 0;
        const eStatus s       = Exchange(req, sizeof(req), resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }
        if (respLen < 4)
        {
            return eStatus::ProtocolError;  // 62 + DID + at least one data byte
        }
        if (resp[0] != (static_cast<uint8_t>(eUdsService::ReadDataByIdentifier) | c_positiveRspMask))
        {
            return eStatus::ProtocolError;
        }
        const uint16_t echoed = static_cast<uint16_t>((static_cast<uint16_t>(resp[1]) << 8) | resp[2]);
        if (echoed != did)
        {
            return eStatus::ProtocolError;  // DID echo must match request
        }

        const size_t dataLen = respLen - 3;
        if (dataLen > outCapacity)
        {
            return eStatus::Overrun;
        }
        memcpy(out, &resp[3], dataLen);
        *outLen = dataLen;
        return eStatus::Ok;
    }

    // -------- ReadDids (multi) ----------------------------------------------------

    eStatus UdsClient::ReadDids(tDidRead *reads, size_t count, uint32_t timeoutMs)
    {
        if (!HasTransport())
        {
            return eStatus::BackendUnavailable;
        }
        if (reads == nullptr || count == 0 || count > c_maxDidsPerRead)
        {
            return eStatus::InvalidFrame;
        }

        // Validate entries and pre-check that the full response (every DID
        // answered) fits the scratch buffer -- better to reject up front
        // than to Overrun mid-parse.
        size_t worstCaseResp = 1;  // [62]
        for (size_t i = 0; i < count; ++i)
        {
            if (reads[i].out == nullptr || reads[i].expectedLen == 0 || reads[i].outCapacity < reads[i].expectedLen)
            {
                return eStatus::InvalidFrame;
            }
            reads[i].len = 0;
            worstCaseResp += 2 + reads[i].expectedLen;
        }
        if (worstCaseResp > c_readScratchLen)
        {
            return eStatus::Overrun;
        }

        // Request: [22] ([DID hi] [DID lo])+
        uint8_t req[1 + 2 * c_maxDidsPerRead];
        req[0] = static_cast<uint8_t>(eUdsService::ReadDataByIdentifier);
        for (size_t i = 0; i < count; ++i)
        {
            req[1 + 2 * i] = static_cast<uint8_t>(reads[i].did >> 8);
            req[2 + 2 * i] = static_cast<uint8_t>(reads[i].did & 0xFFu);
        }

        uint8_t       resp[c_readScratchLen];
        size_t        respLen = 0;
        const eStatus s       = Exchange(req, 1 + 2 * count, resp, sizeof(resp), &respLen, EffectiveTimeoutMs(timeoutMs));
        if (!IsOk(s))
        {
            return s;
        }
        if (respLen < 1 || resp[0] != (static_cast<uint8_t>(eUdsService::ReadDataByIdentifier) | c_positiveRspMask))
        {
            return eStatus::ProtocolError;
        }

        // Split the undelimited record stream. The server includes only the
        // DIDs it supports, in request order (ISO 14229-1 11.2), so each
        // echoed DID is matched against the not-yet-filled tail of `reads`
        // and its expectedLen tells us where the next record starts. A DID
        // we never asked for -- or one whose record length lied -- breaks
        // the scan and fails the whole parse.
        size_t pos     = 1;
        size_t nextIdx = 0;  // first reads[] entry not yet matched
        while (pos < respLen)
        {
            if (respLen - pos < 3)
            {
                return eStatus::ProtocolError;  // dangling partial record
            }
            const uint16_t echoed = static_cast<uint16_t>((static_cast<uint16_t>(resp[pos]) << 8) | resp[pos + 1]);

            size_t match          = nextIdx;
            while (match < count && reads[match].did != echoed)
            {
                ++match;  // skipped entries were omitted by the server
            }
            if (match == count)
            {
                return eStatus::ProtocolError;  // unrequested DID (or desynced parse)
            }

            tDidRead &r = reads[match];
            if (respLen - (pos + 2) < r.expectedLen)
            {
                return eStatus::ProtocolError;  // record truncated
            }
            memcpy(r.out, &resp[pos + 2], r.expectedLen);
            r.len = r.expectedLen;
            pos += 2 + r.expectedLen;
            nextIdx = match + 1;
        }
        return eStatus::Ok;
    }

}  // namespace subiediag::uds
