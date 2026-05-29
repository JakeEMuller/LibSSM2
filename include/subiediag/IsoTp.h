// subiediag/IsoTp.h
//
// ISO 15765-2 transport over CAN.
//
// Wraps a non-owning ICanBus pointer and handles single-frame (SF),
// first-frame / consecutive-frame (FF / CF), and flow-control (FC) framing
// to exchange byte-array messages with a peer ECU on a fixed request /
// response CAN-ID pair.
//
// Buffers are caller-provided -- the transport does not allocate. The
// receive side rejects frames whose CAN ID does not match m_respId, so it
// is safe to use on a bus shared by multiple ECUs (e.g. engine + TCM).
//
// Threading: not thread-safe. Each IsoTpTransport instance is single-owner.

#pragma once

#include "subiediag/Can.h"
#include "subiediag/Common.h"

#include <stddef.h>
#include <stdint.h>

namespace subiediag
{

    class IsoTpTransport
    {
    public:

        IsoTpTransport(ICanBus *bus, uint32_t reqId, uint32_t respId, uint8_t padByte = 0x00) noexcept;

        // Send `payloadLen` bytes from `payload`. Chooses SF vs. FF/CF framing
        // automatically. Honors the peer's flow-control reply, including STmin
        // and block-size parameters.
        [[nodiscard]] eStatus SendRequest(const uint8_t *payload, size_t payloadLen, uint32_t timeoutMs);

        // Receive a full message into `out` (capacity `outCapacity`). Writes the
        // actual length to *outLen on success. Returns eStatus::Overrun if the
        // reassembled message is bigger than outCapacity.
        [[nodiscard]] eStatus ReceiveResponse(uint8_t *out, size_t outCapacity, size_t *outLen, uint32_t timeoutMs);

        // Convenience: send + receive. The deadline applies to the whole round-
        // trip, not each phase individually.
        [[nodiscard]] eStatus Exchange(const uint8_t *req,
                                       size_t         reqLen,
                                       uint8_t       *out,
                                       size_t         outCapacity,
                                       size_t        *outLen,
                                       uint32_t       timeoutMs);

        uint32_t RequestId() const noexcept { return m_reqId; }
        uint32_t ResponseId() const noexcept { return m_respId; }
        ICanBus *Bus() noexcept { return m_pBus; }

    private:

        ICanBus *m_pBus;
        uint32_t m_reqId;
        uint32_t m_respId;
        uint8_t  m_padByte;
    };

}  // namespace subiediag
