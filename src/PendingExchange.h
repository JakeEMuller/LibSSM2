// PendingExchange.h -- internal (not installed with the public headers).
//
// Shared request/response exchange with NRC 0x78 (requestCorrectlyReceived-
// ResponsePending) handling, used by Obd2Client and UdsClient. Both
// protocols share the ISO 14229-2 negative-response format: on a negative
// response whose NRC is 0x78 the server is asking for more time, so the
// receive deadline is re-armed to `pendingWaitMs` (P2*server_max) and the
// wait continues until a real reply arrives.
//
// Any other response -- positive or negative -- is returned to the caller
// as-is in resp/respLen with eStatus::Ok; interpreting it is the caller's
// job (Obd2Client treats an unexpected SID as ProtocolError, UdsClient
// additionally captures the NRC).

#pragma once

#include "subiediag/Common.h"
#include "subiediag/IsoTp.h"

#include <stddef.h>
#include <stdint.h>

namespace subiediag::detail
{

    // ISO 15031-5 / ISO 14229-2 negative-response markers.
    constexpr uint8_t c_negRespSid = 0x7F;
    constexpr uint8_t c_nrcRcrRp   = 0x78;  // requestCorrectlyReceived-ResponsePending

    inline eStatus ExchangeWithPendingRetry(isotp::IsoTpTransport &transport,
                                            const uint8_t         *req,
                                            size_t                 reqLen,
                                            uint8_t               *resp,
                                            size_t                 respCap,
                                            size_t                *respLen,
                                            uint32_t               timeoutMs,
                                            uint32_t               pendingWaitMs)
    {
        eStatus s = transport.SendRequest(req, reqLen, timeoutMs);
        if (!IsOk(s))
        {
            return s;
        }

        // Loop receives until we get a non-pending response. The ECU may
        // send 0x7F <SID> 0x78 one or more times while it works on the
        // request; each one re-arms our wait.
        uint32_t waitMs = timeoutMs;
        while (true)
        {
            s = transport.ReceiveResponse(resp, respCap, respLen, waitMs);
            if (!IsOk(s))
            {
                return s;
            }
            if (*respLen >= 3 && resp[0] == c_negRespSid && resp[2] == c_nrcRcrRp)
            {
                waitMs = pendingWaitMs;
                continue;
            }
            return eStatus::Ok;
        }
    }

}  // namespace subiediag::detail
