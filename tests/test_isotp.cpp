// test_isotp.cpp -- ISO 15765-2 round-trip tests against MockCanBus.

#include "subiediag/Can.h"
#include "subiediag/Common.h"
#include "subiediag/IsoTp.h"
#include "MockCanBus.h"

#include <cstdio>
#include <string.h>

namespace
{

    using subiediag::c_canMaxDataLen;
    using subiediag::DescribeStatus;
    using subiediag::eStatus;
    using subiediag::IsOk;
    using subiediag::IsoTpTransport;
    using subiediag::tCanFrame;
    using subiediag_test::MockCanBus;

    // --- tiny test harness ------------------------------------------------------

    int         g_checks      = 0;
    int         g_failed      = 0;
    const char *g_currentTest = "";

#define CHECK(cond)                                                                             \
    do                                                                                          \
    {                                                                                           \
        ++g_checks;                                                                             \
        if (!(cond))                                                                            \
        {                                                                                       \
            ++g_failed;                                                                         \
            std::printf("    FAIL [%s] %s:%d: %s\n", g_currentTest, __FILE__, __LINE__, #cond); \
        }                                                                                       \
    }                                                                                           \
    while (0)

#define CHECK_OK(expr)                                                  \
    do                                                                  \
    {                                                                   \
        ++g_checks;                                                     \
        eStatus _s = (expr);                                            \
        if (!IsOk(_s))                                                  \
        {                                                               \
            ++g_failed;                                                 \
            std::printf("    FAIL [%s] %s:%d: expected Ok, got %.*s\n", \
                        g_currentTest,                                  \
                        __FILE__,                                       \
                        __LINE__,                                       \
                        static_cast<int>(DescribeStatus(_s).size()),    \
                        DescribeStatus(_s).data());                     \
        }                                                               \
    }                                                                   \
    while (0)

#define CHECK_STATUS(expr, expected)                                       \
    do                                                                     \
    {                                                                      \
        ++g_checks;                                                        \
        eStatus _s = (expr);                                               \
        if (_s != (expected))                                              \
        {                                                                  \
            ++g_failed;                                                    \
            std::printf("    FAIL [%s] %s:%d: expected %.*s, got %.*s\n",  \
                        g_currentTest,                                     \
                        __FILE__,                                          \
                        __LINE__,                                          \
                        static_cast<int>(DescribeStatus(expected).size()), \
                        DescribeStatus(expected).data(),                   \
                        static_cast<int>(DescribeStatus(_s).size()),       \
                        DescribeStatus(_s).data());                        \
        }                                                                  \
    }                                                                      \
    while (0)

#define RUN(fn)              \
    do                       \
    {                        \
        g_currentTest = #fn; \
        fn();                \
    }                        \
    while (0)

    constexpr uint32_t kReq  = 0x7E0;
    constexpr uint32_t kResp = 0x7E8;

    // --- tests ------------------------------------------------------------------

    // Single-frame send: 1-byte payload (the SSM2 0xAA init request).
    void test_sf_send_aa()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp);

        uint8_t req[] = {0xAA};
        CHECK_OK(t.SendRequest(req, sizeof(req), 100));

        CHECK(bus.tx.size() == 1);
        CHECK(bus.tx[0].id == kReq);
        CHECK(bus.tx[0].dlc == c_canMaxDataLen);
        CHECK(bus.tx[0].data[0] == 0x01);  // SF, len=1
        CHECK(bus.tx[0].data[1] == 0xAA);
        // Remaining bytes are padding.
        for (size_t i = 2; i < c_canMaxDataLen; ++i)
        {
            CHECK(bus.tx[0].data[i] == 0x00);
        }
    }

    // Single-frame receive: 1-byte SF response.
    void test_sf_recv()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp);

        bus.QueueRx(kResp, c_canMaxDataLen, {0x02, 0xEA, 0xA2, 0, 0, 0, 0, 0});

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_OK(t.ReceiveResponse(out, sizeof(out), &outLen, 100));
        CHECK(outLen == 2);
        CHECK(out[0] == 0xEA);
        CHECK(out[1] == 0xA2);
    }

    // FF + CF send: simulates a 12-byte request. FF carries 6 bytes, then 1 CF
    // with 6 more.
    void test_ff_cf_send()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp);

        // Mock provides the FC the transport will wait for after the FF.
        bus.QueueRx(kResp, c_canMaxDataLen, {0x30, 0x00, 0x00, 0, 0, 0, 0, 0});

        uint8_t req[12];
        for (size_t i = 0; i < sizeof(req); ++i)
        {
            req[i] = static_cast<uint8_t>(0x10 + i);
        }

        CHECK_OK(t.SendRequest(req, sizeof(req), 200));

        // Expect 2 frames: FF then one CF.
        CHECK(bus.tx.size() == 2);
        // FF: 0x1L LL data*6 ; LL = 12 -> low byte 0x0C, high nibble 0
        CHECK(bus.tx[0].data[0] == 0x10);
        CHECK(bus.tx[0].data[1] == 0x0C);
        for (size_t i = 0; i < 6; ++i)
        {
            CHECK(bus.tx[0].data[2 + i] == req[i]);
        }
        // CF1: seq 1, remaining 6 bytes
        CHECK(bus.tx[1].data[0] == 0x21);
        for (size_t i = 0; i < 6; ++i)
        {
            CHECK(bus.tx[1].data[1 + i] == req[6 + i]);
        }
        // 7th data byte = pad
        CHECK(bus.tx[1].data[7] == 0x00);
    }

    // FF + CF receive: 12-byte response.
    void test_ff_cf_recv()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp);

        // FF: 12 total, first 6 bytes
        bus.QueueRx(kResp, c_canMaxDataLen, {0x10, 0x0C, 0xEA, 0xA2, 0x10, 0x02, 0x51, 0x12});
        // CF seq 1: last 6 bytes (the rest is padding)
        bus.QueueRx(kResp, c_canMaxDataLen, {0x21, 0x18, 0x80, 0x07, 0xF3, 0xFA, 0xC9, 0x00});

        uint8_t out[20] = {};
        size_t  outLen  = 0;
        CHECK_OK(t.ReceiveResponse(out, sizeof(out), &outLen, 200));
        CHECK(outLen == 12);

        static const uint8_t expected[] = {
            0xEA,
            0xA2,
            0x10,
            0x02,
            0x51,
            0x12,
            0x18,
            0x80,
            0x07,
            0xF3,
            0xFA,
            0xC9,
        };
        CHECK(memcmp(out, expected, sizeof(expected)) == 0);

        // The transport should have sent exactly one FC.
        CHECK(bus.tx.size() == 1);
        CHECK(bus.tx[0].data[0] == 0x30);
        CHECK(bus.tx[0].data[1] == 0x00);
        CHECK(bus.tx[0].data[2] == 0x00);
    }

    // Custom pad byte propagates into transmitted frames.
    void test_custom_pad()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp, /*padByte=*/0xCC);

        uint8_t req[] = {0xAA};
        CHECK_OK(t.SendRequest(req, sizeof(req), 100));
        CHECK(bus.tx.size() == 1);
        for (size_t i = 2; i < c_canMaxDataLen; ++i)
        {
            CHECK(bus.tx[0].data[i] == 0xCC);
        }
    }

    // Receive ignores frames from other CAN IDs.
    void test_filter_other_id()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp);

        bus.QueueRx(0x123, c_canMaxDataLen, {0xDE, 0xAD, 0, 0, 0, 0, 0, 0});
        bus.QueueRx(0x456, c_canMaxDataLen, {0xBE, 0xEF, 0, 0, 0, 0, 0, 0});
        bus.QueueRx(kResp, c_canMaxDataLen, {0x01, 0xEA, 0, 0, 0, 0, 0, 0});

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_OK(t.ReceiveResponse(out, sizeof(out), &outLen, 100));
        CHECK(outLen == 1);
        CHECK(out[0] == 0xEA);
    }

    // Wrong CF sequence number -> SequenceError.
    void test_sequence_error()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp);

        bus.QueueRx(kResp, c_canMaxDataLen, {0x10, 0x0A, 0xEA, 0xA2, 0x10, 0x02, 0x51, 0x12});
        // CF should be seq 1, but mock sends seq 5.
        bus.QueueRx(kResp, c_canMaxDataLen, {0x25, 0x18, 0x80, 0x07, 0, 0, 0, 0});

        uint8_t out[20] = {};
        size_t  outLen  = 0;
        CHECK_STATUS(t.ReceiveResponse(out, sizeof(out), &outLen, 100), eStatus::SequenceError);
    }

    // FC with overflow flag -> FlowControlAbort on send.
    void test_flow_control_abort()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp);

        // Peer responds to our FF with FC overflow (0x32).
        bus.QueueRx(kResp, c_canMaxDataLen, {0x32, 0x00, 0x00, 0, 0, 0, 0, 0});

        uint8_t req[12];
        memset(req, 0xAA, sizeof(req));
        CHECK_STATUS(t.SendRequest(req, sizeof(req), 100), eStatus::FlowControlAbort);
    }

    // Response larger than buffer -> Overrun.
    void test_overrun_on_receive()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp);

        // FF says total = 12 bytes, but buffer is only 8.
        bus.QueueRx(kResp, c_canMaxDataLen, {0x10, 0x0C, 0xEA, 0xA2, 0x10, 0x02, 0x51, 0x12});

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_STATUS(t.ReceiveResponse(out, sizeof(out), &outLen, 100), eStatus::Overrun);
    }

    // Empty rx queue and a real timeout -> Timeout.
    void test_timeout_on_receive()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp);

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_STATUS(t.ReceiveResponse(out, sizeof(out), &outLen, 10), eStatus::Timeout);
    }

    // Sequence-number wrap: 16 CFs after FF means seq goes 1..15 then 0 then 1.
    // Total payload: 6 (FF) + 17 * 7 = 125 bytes covers wrap through seq 0.
    void test_sequence_wrap()
    {
        MockCanBus     bus;
        IsoTpTransport t(&bus, kReq, kResp);

        const size_t total = 6 + 17 * 7;  // 125
        uint8_t      req[125];
        for (size_t i = 0; i < sizeof(req); ++i)
        {
            req[i] = static_cast<uint8_t>(i);
        }

        bus.QueueRx(kResp, c_canMaxDataLen, {0x30, 0x00, 0x00, 0, 0, 0, 0, 0});
        CHECK_OK(t.SendRequest(req, sizeof(req), 500));

        CHECK(bus.tx.size() == 1 + 17);  // FF + 17 CFs

        // Verify the sequence numbers in the CFs run 1, 2, ..., 15, 0, 1.
        static const uint8_t expectedSeq[17] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1};
        for (size_t i = 0; i < 17; ++i)
        {
            CHECK((bus.tx[1 + i].data[0] & 0xF0) == 0x20);
            CHECK((bus.tx[1 + i].data[0] & 0x0F) == expectedSeq[i]);
        }
    }

}  // namespace

int main()
{
    std::printf("isotp tests:\n");

    RUN(test_sf_send_aa);
    RUN(test_sf_recv);
    RUN(test_ff_cf_send);
    RUN(test_ff_cf_recv);
    RUN(test_custom_pad);
    RUN(test_filter_other_id);
    RUN(test_sequence_error);
    RUN(test_flow_control_abort);
    RUN(test_overrun_on_receive);
    RUN(test_timeout_on_receive);
    RUN(test_sequence_wrap);

    std::printf("  %d/%d checks passed\n", g_checks - g_failed, g_checks);
    return g_failed == 0 ? 0 : 1;
}
