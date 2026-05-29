// IsoTp.cpp -- ISO 15765-2 transport over CAN.
//
// Implements single-frame (SF) and segmented (FF / CF) message exchange,
// including the flow-control handshake. Buffers are caller-provided.
//
// Padding: each transmitted CAN frame is full-length (8 bytes). Bytes
// beyond the ISO-TP payload are filled with m_padByte.

#include "subiediag/IsoTp.h"

#include <chrono>
#include <string.h>

namespace subiediag
{

    namespace
    {

        // ISO-TP byte-0 high nibble (frame type). Private to this TU.
        enum class eFrameType : uint8_t
        {
            SingleFrame      = 0x0,
            FirstFrame       = 0x1,
            ConsecutiveFrame = 0x2,
            FlowControl      = 0x3,
        };

        // Payload sizing.
        constexpr size_t   c_sfPayloadMax   = 7;     // single frame: 1 byte header
        constexpr size_t   c_ffPayloadFirst = 6;     // first frame:  2 byte header
        constexpr size_t   c_cfPayloadMax   = 7;     // consecutive:  1 byte header
        constexpr uint16_t c_ffTotalLenMax  = 4095;  // 12-bit length field

        // Flow-control parameters we send when WE receive a multi-frame message.
        // BS=0  -> tell the sender to stream all CFs without further FC.
        // STmin -> no separation requirement (0 ms).
        constexpr uint8_t c_fcBlockSize = 0;
        constexpr uint8_t c_fcStMin     = 0;

        // FC flow-status nibble (byte 0 low nibble).
        constexpr uint8_t c_fcContinue = 0x0;
        constexpr uint8_t c_fcWait     = 0x1;
        constexpr uint8_t c_fcOverflow = 0x2;

        uint32_t MonotonicMs() noexcept
        {
            using namespace std::chrono;
            return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
        }

        // Tracks remaining budget against a starting time. Used so that a single
        // caller-provided timeoutMs covers a multi-step exchange.
        class Deadline
        {
        public:

            explicit Deadline(uint32_t timeoutMs) noexcept
                : m_start(MonotonicMs())
                , m_budget(timeoutMs)
            {
            }

            uint32_t Remaining() const noexcept
            {
                const uint32_t elapsed = MonotonicMs() - m_start;
                return elapsed >= m_budget ? 0 : m_budget - elapsed;
            }

        private:

            uint32_t m_start;
            uint32_t m_budget;
        };

        // Helpers to construct each frame type into a caller-provided tCanFrame.

        void BuildSingleFrame(tCanFrame &f, uint32_t id, const uint8_t *payload, size_t len, uint8_t padByte) noexcept
        {
            f.timestampUs = 0;
            f.id          = id;
            f.dlc         = c_canMaxDataLen;
            f.extended    = false;
            f.data[0]     = static_cast<uint8_t>(len);  // high nibble = SF type (0)
            memcpy(&f.data[1], payload, len);
            const size_t used = 1 + len;
            if (used < c_canMaxDataLen)
            {
                memset(&f.data[used], padByte, c_canMaxDataLen - used);
            }
        }

        void BuildFirstFrame(tCanFrame &f, uint32_t id, const uint8_t *payload, uint16_t totalLen) noexcept
        {
            f.timestampUs = 0;
            f.id          = id;
            f.dlc         = c_canMaxDataLen;
            f.extended    = false;
            f.data[0]     = static_cast<uint8_t>(0x10 | ((totalLen >> 8) & 0x0F));
            f.data[1]     = static_cast<uint8_t>(totalLen & 0xFF);
            memcpy(&f.data[2], payload, c_ffPayloadFirst);
        }

        void BuildConsecutiveFrame(tCanFrame &f, uint32_t id, uint8_t seq, const uint8_t *payload, size_t len, uint8_t padByte) noexcept
        {
            f.timestampUs = 0;
            f.id          = id;
            f.dlc         = c_canMaxDataLen;
            f.extended    = false;
            f.data[0]     = static_cast<uint8_t>(0x20 | (seq & 0x0F));
            memcpy(&f.data[1], payload, len);
            const size_t used = 1 + len;
            if (used < c_canMaxDataLen)
            {
                memset(&f.data[used], padByte, c_canMaxDataLen - used);
            }
        }

        void BuildFlowControl(tCanFrame &f, uint32_t id, uint8_t bs, uint8_t stMin, uint8_t padByte) noexcept
        {
            f.timestampUs = 0;
            f.id          = id;
            f.dlc         = c_canMaxDataLen;
            f.extended    = false;
            f.data[0]     = 0x30 | c_fcContinue;
            f.data[1]     = bs;
            f.data[2]     = stMin;
            memset(&f.data[3], padByte, c_canMaxDataLen - 3);
        }

    }  // namespace

    // ---------------------------------------------------------------------------

    IsoTpTransport::IsoTpTransport(ICanBus *bus, uint32_t reqId, uint32_t respId, uint8_t padByte) noexcept
        : m_pBus(bus)
        , m_reqId(reqId)
        , m_respId(respId)
        , m_padByte(padByte)
    {
    }

    eStatus IsoTpTransport::SendRequest(const uint8_t *payload, size_t payloadLen, uint32_t timeoutMs)
    {
        if (m_pBus == nullptr)
        {
            return eStatus::BackendUnavailable;
        }
        if (payload == nullptr && payloadLen > 0)
        {
            return eStatus::InvalidFrame;
        }
        if (payloadLen == 0 || payloadLen > c_ffTotalLenMax)
        {
            return eStatus::InvalidFrame;
        }

        Deadline deadline(timeoutMs);

        // --- single frame fast path ---
        if (payloadLen <= c_sfPayloadMax)
        {
            tCanFrame frame;
            BuildSingleFrame(frame, m_reqId, payload, payloadLen, m_padByte);
            const uint32_t r = deadline.Remaining();
            if (r == 0)
            {
                return eStatus::Timeout;
            }
            return m_pBus->Send(frame, r);
        }

        // --- first frame ---
        tCanFrame ff;
        BuildFirstFrame(ff, m_reqId, payload, static_cast<uint16_t>(payloadLen));
        {
            const uint32_t r = deadline.Remaining();
            if (r == 0)
            {
                return eStatus::Timeout;
            }
            const eStatus s = m_pBus->Send(ff, r);
            if (!IsOk(s))
            {
                return s;
            }
        }

        // --- wait for flow control from peer (may receive Wait responses) ---
        while (true)
        {
            tCanFrame      fc;
            const uint32_t r = deadline.Remaining();
            if (r == 0)
            {
                return eStatus::Timeout;
            }
            const eStatus s = m_pBus->Receive(&fc, r);
            if (!IsOk(s))
            {
                return s;
            }

            if (fc.id != m_respId)
            {
                continue;  // someone else's frame
            }
            if (fc.dlc < 3)
            {
                return eStatus::InvalidFrame;
            }
            const uint8_t type = fc.data[0] >> 4;
            if (type != static_cast<uint8_t>(eFrameType::FlowControl))
            {
                return eStatus::InvalidFrame;
            }

            const uint8_t flag = fc.data[0] & 0x0F;
            if (flag == c_fcContinue)
            {
                break;
            }
            if (flag == c_fcWait)
            {
                continue;  // peer asks to wait
            }
            if (flag == c_fcOverflow)
            {
                return eStatus::FlowControlAbort;
            }
            return eStatus::InvalidFrame;
        }

        // We intentionally ignore the peer's BS / STmin and stream all CFs back to
        // back. For the SSM2 ECUs we care about, this matches the proven-working
        // flow captured in the user's CAN log.

        // --- consecutive frames ---
        size_t  sent = c_ffPayloadFirst;
        uint8_t seq  = 1;
        while (sent < payloadLen)
        {
            const size_t remaining = payloadLen - sent;
            const size_t chunk     = remaining < c_cfPayloadMax ? remaining : c_cfPayloadMax;

            tCanFrame cf;
            BuildConsecutiveFrame(cf, m_reqId, seq, payload + sent, chunk, m_padByte);
            const uint32_t r = deadline.Remaining();
            if (r == 0)
            {
                return eStatus::Timeout;
            }
            const eStatus s = m_pBus->Send(cf, r);
            if (!IsOk(s))
            {
                return s;
            }

            sent += chunk;
            seq = (seq + 1) & 0x0F;  // wraps 15 -> 0
        }

        return eStatus::Ok;
    }

    eStatus IsoTpTransport::ReceiveResponse(uint8_t *out, size_t outCapacity, size_t *outLen, uint32_t timeoutMs)
    {
        if (m_pBus == nullptr)
        {
            return eStatus::BackendUnavailable;
        }
        if (out == nullptr || outLen == nullptr)
        {
            return eStatus::InvalidFrame;
        }

        Deadline deadline(timeoutMs);
        *outLen = 0;

        // --- wait for the first frame for us (SF or FF) ---
        tCanFrame first;
        while (true)
        {
            const uint32_t r = deadline.Remaining();
            if (r == 0)
            {
                return eStatus::Timeout;
            }
            const eStatus s = m_pBus->Receive(&first, r);
            if (!IsOk(s))
            {
                return s;
            }
            if (first.id == m_respId)
            {
                break;
            }
        }

        if (first.dlc < 1)
        {
            return eStatus::InvalidFrame;
        }
        const uint8_t firstType = first.data[0] >> 4;

        // --- single frame: done in one ---
        if (firstType == static_cast<uint8_t>(eFrameType::SingleFrame))
        {
            const uint8_t len = first.data[0] & 0x0F;
            if (len == 0 || len > c_sfPayloadMax)
            {
                return eStatus::InvalidFrame;
            }
            if (len > first.dlc - 1)
            {
                return eStatus::InvalidFrame;
            }
            if (len > outCapacity)
            {
                return eStatus::Overrun;
            }
            memcpy(out, &first.data[1], len);
            *outLen = len;
            return eStatus::Ok;
        }

        // --- otherwise must be a first frame ---
        if (firstType != static_cast<uint8_t>(eFrameType::FirstFrame))
        {
            return eStatus::InvalidFrame;
        }
        if (first.dlc < c_canMaxDataLen)
        {
            return eStatus::InvalidFrame;
        }

        const uint16_t totalLen = static_cast<uint16_t>((first.data[0] & 0x0F) << 8) | first.data[1];

        // A FF must carry > 7 bytes (else SF would have been used). Reject < 8.
        if (totalLen <= c_sfPayloadMax)
        {
            return eStatus::InvalidFrame;
        }
        if (totalLen > outCapacity)
        {
            return eStatus::Overrun;
        }

        memcpy(out, &first.data[2], c_ffPayloadFirst);
        size_t received = c_ffPayloadFirst;

        // --- send flow control back ---
        {
            tCanFrame fc;
            BuildFlowControl(fc, m_reqId, c_fcBlockSize, c_fcStMin, m_padByte);
            const uint32_t r = deadline.Remaining();
            if (r == 0)
            {
                return eStatus::Timeout;
            }
            const eStatus s = m_pBus->Send(fc, r);
            if (!IsOk(s))
            {
                return s;
            }
        }

        // --- consecutive frames ---
        uint8_t expectedSeq = 1;
        while (received < totalLen)
        {
            tCanFrame      cf;
            const uint32_t r = deadline.Remaining();
            if (r == 0)
            {
                return eStatus::Timeout;
            }
            const eStatus s = m_pBus->Receive(&cf, r);
            if (!IsOk(s))
            {
                return s;
            }

            if (cf.id != m_respId)
            {
                continue;
            }
            if (cf.dlc < 1)
            {
                return eStatus::InvalidFrame;
            }
            if ((cf.data[0] >> 4) != static_cast<uint8_t>(eFrameType::ConsecutiveFrame))
            {
                return eStatus::InvalidFrame;
            }
            if ((cf.data[0] & 0x0F) != expectedSeq)
            {
                return eStatus::SequenceError;
            }

            const size_t remaining = totalLen - received;
            const size_t take      = remaining < c_cfPayloadMax ? remaining : c_cfPayloadMax;
            // Defensive: the frame must carry at least `take` bytes of payload.
            if (cf.dlc < 1 + take)
            {
                return eStatus::InvalidFrame;
            }
            memcpy(out + received, &cf.data[1], take);
            received += take;
            expectedSeq = (expectedSeq + 1) & 0x0F;
        }

        *outLen = received;
        return eStatus::Ok;
    }

    eStatus IsoTpTransport::Exchange(const uint8_t *req,
                                     size_t         reqLen,
                                     uint8_t       *out,
                                     size_t         outCapacity,
                                     size_t        *outLen,
                                     uint32_t       timeoutMs)
    {
        Deadline deadline(timeoutMs);

        const uint32_t r1 = deadline.Remaining();
        if (r1 == 0)
        {
            return eStatus::Timeout;
        }
        const eStatus s = SendRequest(req, reqLen, r1);
        if (!IsOk(s))
        {
            return s;
        }

        const uint32_t r2 = deadline.Remaining();
        if (r2 == 0)
        {
            return eStatus::Timeout;
        }
        return ReceiveResponse(out, outCapacity, outLen, r2);
    }

}  // namespace subiediag
