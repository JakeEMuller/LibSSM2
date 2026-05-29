// MockCanBus.h
//
// In-memory ICanBus for unit tests. Not shipped with the library.
//
// Usage:
//   MockCanBus bus;
//   bus.QueueRx(frame);                 // enqueue a frame the next Receive() returns
//   IsoTpTransport t(&bus, 0x7E0, 0x7E8);
//   t.SendRequest(...);                 // captured in bus.tx
//   EXPECT_EQ(bus.tx.size(), 1u);
//
// The mock ignores timeoutMs except that Receive() on an empty queue returns
// eStatus::Timeout immediately.

#pragma once

#include "subiediag/Can.h"
#include "subiediag/Common.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <vector>

namespace subiediag_test
{

    class MockCanBus : public subiediag::ICanBus
    {
    public:

        // Public test surface: inspect Tx, populate Rx.
        std::vector<subiediag::tCanFrame> tx;  // frames captured from Send()
        std::vector<subiediag::tCanFrame> rx;  // frames to feed back to Receive()

        // Force the next Receive() to return this status instead of pulling from rx.
        // After one use, behavior reverts to the queue. (Default Ok.)
        subiediag::eStatus forceReceiveStatus = subiediag::eStatus::Ok;
        subiediag::eStatus forceSendStatus    = subiediag::eStatus::Ok;

        void QueueRx(uint32_t id, uint8_t dlc, std::initializer_list<uint8_t> data)
        {
            subiediag::tCanFrame f{};
            f.id     = id;
            f.dlc    = dlc;
            size_t i = 0;
            for (uint8_t b : data)
            {
                if (i >= subiediag::c_canMaxDataLen)
                {
                    break;
                }
                f.data[i++] = b;
            }
            rx.push_back(f);
        }

        // --- ICanBus impl ------------------------------------------------------

        subiediag::eStatus Open() override
        {
            m_isOpen = true;
            return subiediag::eStatus::Ok;
        }
        subiediag::eStatus Close() override
        {
            m_isOpen = false;
            return subiediag::eStatus::Ok;
        }
        bool IsOpen() const noexcept override { return m_isOpen; }

        subiediag::eStatus SetRxFilter(const uint32_t * /*ids*/, size_t /*count*/) override { return subiediag::eStatus::Ok; }

        subiediag::eStatus Send(const subiediag::tCanFrame &frame, uint32_t /*timeoutMs*/) override
        {
            if (forceSendStatus != subiediag::eStatus::Ok)
            {
                return forceSendStatus;
            }
            tx.push_back(frame);
            return subiediag::eStatus::Ok;
        }

        subiediag::eStatus Receive(subiediag::tCanFrame *out, uint32_t /*timeoutMs*/) override
        {
            if (forceReceiveStatus != subiediag::eStatus::Ok)
            {
                const subiediag::eStatus s = forceReceiveStatus;
                forceReceiveStatus       = subiediag::eStatus::Ok;
                return s;
            }
            if (rx.empty())
            {
                return subiediag::eStatus::Timeout;
            }
            *out = rx.front();
            rx.erase(rx.begin());
            return subiediag::eStatus::Ok;
        }

    private:

        bool m_isOpen = true;
    };

}  // namespace subiediag_test
