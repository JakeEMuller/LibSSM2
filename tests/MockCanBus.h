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

#include "libssm2/Can.h"
#include "libssm2/Common.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <vector>

namespace libssm2_test
{

    class MockCanBus : public libssm2::ICanBus
    {
    public:

        // Public test surface: inspect Tx, populate Rx.
        std::vector<libssm2::tCanFrame> tx;  // frames captured from Send()
        std::vector<libssm2::tCanFrame> rx;  // frames to feed back to Receive()

        // Force the next Receive() to return this status instead of pulling from rx.
        // After one use, behavior reverts to the queue. (Default Ok.)
        libssm2::eStatus forceReceiveStatus = libssm2::eStatus::Ok;
        libssm2::eStatus forceSendStatus    = libssm2::eStatus::Ok;

        void QueueRx(uint32_t id, uint8_t dlc, std::initializer_list<uint8_t> data)
        {
            libssm2::tCanFrame f{};
            f.id     = id;
            f.dlc    = dlc;
            size_t i = 0;
            for (uint8_t b : data)
            {
                if (i >= libssm2::c_canMaxDataLen)
                {
                    break;
                }
                f.data[i++] = b;
            }
            rx.push_back(f);
        }

        // --- ICanBus impl ------------------------------------------------------

        libssm2::eStatus Open() override
        {
            m_isOpen = true;
            return libssm2::eStatus::Ok;
        }
        libssm2::eStatus Close() override
        {
            m_isOpen = false;
            return libssm2::eStatus::Ok;
        }
        bool IsOpen() const noexcept override { return m_isOpen; }

        libssm2::eStatus SetRxFilter(const uint32_t * /*ids*/, size_t /*count*/) override { return libssm2::eStatus::Ok; }

        libssm2::eStatus Send(const libssm2::tCanFrame &frame, uint32_t /*timeoutMs*/) override
        {
            if (forceSendStatus != libssm2::eStatus::Ok)
            {
                return forceSendStatus;
            }
            tx.push_back(frame);
            return libssm2::eStatus::Ok;
        }

        libssm2::eStatus Receive(libssm2::tCanFrame *out, uint32_t /*timeoutMs*/) override
        {
            if (forceReceiveStatus != libssm2::eStatus::Ok)
            {
                const libssm2::eStatus s = forceReceiveStatus;
                forceReceiveStatus       = libssm2::eStatus::Ok;
                return s;
            }
            if (rx.empty())
            {
                return libssm2::eStatus::Timeout;
            }
            *out = rx.front();
            rx.erase(rx.begin());
            return libssm2::eStatus::Ok;
        }

    private:

        bool m_isOpen = true;
    };

}  // namespace libssm2_test
