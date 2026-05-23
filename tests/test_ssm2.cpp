// test_ssm2.cpp -- Ssm2Client end-to-end tests against MockCanBus.
//
// Each test stages an ISO-TP response in the mock, invokes the client, and
// validates both the wire bytes the client emitted and the decoded result.
// The init test uses the actual response captured from the user's 2008
// Impreza so we know the bit layout matches reality.

#include "libssm2/Can.h"
#include "libssm2/Common.h"
#include "libssm2/Ssm2.h"
#include "MockCanBus.h"

#include <cstdio>
#include <string.h>

#include <vector>

namespace
{

    using libssm2::c_canMaxDataLen;
    using libssm2::c_capFlagsLen;
    using libssm2::c_engineReqId;
    using libssm2::c_engineRespId;
    using libssm2::c_romIdLen;
    using libssm2::c_ssmIdLen;
    using libssm2::DescribeStatus;
    using libssm2::eSsm2Cmd;
    using libssm2::eSsm2Rsp;
    using libssm2::eStatus;
    using libssm2::IsOk;
    using libssm2::Ssm2Client;
    using libssm2::tSsm2InitResponse;
    using libssm2_test::MockCanBus;

    // --- tiny test harness ----------------------------------------------------

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

    // --- ISO-TP framing helpers for stubbing responses ------------------------

    // Queue an ISO-TP single-frame response carrying `payload`.
    void QueueIsoTpSingleFrame(MockCanBus &bus, uint32_t id, const std::vector<uint8_t> &payload)
    {
        libssm2::tCanFrame f{};
        f.id      = id;
        f.dlc     = c_canMaxDataLen;
        f.data[0] = static_cast<uint8_t>(payload.size());
        for (size_t i = 0; i < payload.size(); ++i)
        {
            f.data[1 + i] = payload[i];
        }
        bus.rx.push_back(f);
    }

    // Queue an ISO-TP multi-frame response: one FF + N CFs covering `payload`.
    // Does NOT queue a flow-control frame -- caller's transport will send the
    // FC into the mock's tx queue and we don't need to "respond" to it; the
    // mock just keeps feeding the queued CFs.
    void QueueIsoTpMultiFrame(MockCanBus &bus, uint32_t id, const std::vector<uint8_t> &payload)
    {
        const uint16_t total = static_cast<uint16_t>(payload.size());

        libssm2::tCanFrame ff{};
        ff.id      = id;
        ff.dlc     = c_canMaxDataLen;
        ff.data[0] = static_cast<uint8_t>(0x10 | ((total >> 8) & 0x0F));
        ff.data[1] = static_cast<uint8_t>(total & 0xFF);
        for (size_t i = 0; i < 6 && i < payload.size(); ++i)
        {
            ff.data[2 + i] = payload[i];
        }
        bus.rx.push_back(ff);

        size_t  sent = 6;
        uint8_t seq  = 1;
        while (sent < payload.size())
        {
            libssm2::tCanFrame cf{};
            cf.id             = id;
            cf.dlc            = c_canMaxDataLen;
            cf.data[0]        = 0x20 | (seq & 0x0F);
            const size_t take = (payload.size() - sent) < 7 ? (payload.size() - sent) : 7;
            for (size_t i = 0; i < take; ++i)
            {
                cf.data[1 + i] = payload[sent + i];
            }
            bus.rx.push_back(cf);
            sent += take;
            seq = (seq + 1) & 0x0F;
        }
    }

    Ssm2Client MakeClient(MockCanBus &bus)
    {
        Ssm2Client::tConfig cfg;
        cfg.bus = &bus;
        return Ssm2Client(cfg);
    }

    // --- tests ----------------------------------------------------------------

    // Init: feed back the response captured from the user's 2008 Impreza.
    // Verifies request bytes on the wire, response decode, and IsSupported().
    void test_init_decodes_real_init_response()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);

        // Real init response (105 bytes) from the user's car.
        // Layout: EA SSM(3)=A2 10 02  ROM(5)=51 12 18 80 07  CAP(96)
        std::vector<uint8_t> payload = {
            0xEA, 0xA2, 0x10, 0x02, 0x51, 0x12, 0x18, 0x80, 0x07, 0xF3, 0xFA, 0xC9, 0x8E,  // cap bytes 1-4
            0x02, 0x04, 0x02, 0xAC, 0x00, 0x00, 0x00,                                      // 5-11
            0x66, 0xCE, 0x54, 0xF9, 0xB9, 0x84, 0x00,                                      // 12-18
            0x6F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,                                      // 19-25
            0xDC, 0x00, 0x00, 0x5D, 0x1F, 0x30, 0x80,                                      // 26-32
            0xF0, 0x24, 0x1F, 0x02, 0x43, 0xFB, 0x00,                                      // 33-39
            0xF5, 0xC1, 0x84, 0x00, 0x00, 0x00, 0x01,                                      // 40-46
            0xE1, 0xF1, 0x80, 0x00, 0x81, 0x80, 0x00,                                      // 47-53
        };
        // Pad to 105 bytes total (1 + 3 + 5 + 96).
        while (payload.size() < 1 + c_ssmIdLen + c_romIdLen + c_capFlagsLen)
        {
            payload.push_back(0x00);
        }

        QueueIsoTpMultiFrame(bus, c_engineRespId, payload);

        tSsm2InitResponse init{};
        CHECK_OK(client.Init(&init));

        // Request bytes on the wire: a single-frame [0x01, 0xAA, pad..]
        // First TX is FF? No -- request is just 1 byte so it's a single frame.
        // But there's also the flow control frame we sent after the FF response.
        // So bus.tx = [request SF, flow control].
        CHECK(bus.tx.size() == 2);
        CHECK(bus.tx[0].id == c_engineReqId);
        CHECK(bus.tx[0].data[0] == 0x01);  // SF, len=1
        CHECK(bus.tx[0].data[1] == static_cast<uint8_t>(eSsm2Cmd::Init));
        // Second frame: flow control 30 00 00 ...
        CHECK(bus.tx[1].data[0] == 0x30);

        // Decoded init response.
        CHECK(init.ssmId[0] == 0xA2 && init.ssmId[1] == 0x10 && init.ssmId[2] == 0x02);
        CHECK(init.romId[0] == 0x51 && init.romId[1] == 0x12 && init.romId[2] == 0x18 && init.romId[3] == 0x80 && init.romId[4] == 0x07);
        CHECK(init.capFlags[0] == 0xF3);
        CHECK(init.capFlags[7] == 0xAC);

        // IsSupported sanity: byte 1 / bit 1 = Engine Speed = supported (0xF3 has bit 0 set).
        CHECK(client.IsInitialized());
        CHECK(client.IsSupported(1, 1));
        // byte 1 / bit 4: 0xF3 = 1111_0011 -> bit 4 (1<<3 = 0x08) is clear.
        CHECK(!client.IsSupported(1, 4));
        // Out-of-range -> false.
        CHECK(!client.IsSupported(0, 1));
        CHECK(!client.IsSupported(1, 0));
        CHECK(!client.IsSupported(c_capFlagsLen + 1, 1));
    }

    // Init: response with the wrong response code -> ProtocolError.
    void test_init_rejects_bad_response_code()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);

        std::vector<uint8_t> payload(105, 0x00);
        payload[0] = 0x00;  // wrong (should be 0xEA)
        QueueIsoTpMultiFrame(bus, c_engineRespId, payload);

        tSsm2InitResponse init{};
        CHECK_STATUS(client.Init(&init), eStatus::ProtocolError);
        CHECK(!client.IsInitialized());
    }

    // ReadAddresses for a single address, fully fitting in a single ISO-TP
    // frame on both directions.
    void test_read_addresses_single_addr_sf()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);

        QueueIsoTpSingleFrame(bus, c_engineRespId, {static_cast<uint8_t>(eSsm2Rsp::ReadAddresses), 0x99});

        const uint32_t addrs[] = {0x123456};
        uint8_t        out     = 0;
        CHECK_OK(client.ReadAddresses(addrs, 1, &out));
        CHECK(out == 0x99);

        // Request payload: [A8 00 12 34 56] -> 5 bytes -> SF prefix 0x05.
        CHECK(bus.tx.size() == 1);
        CHECK(bus.tx[0].data[0] == 0x05);
        CHECK(bus.tx[0].data[1] == static_cast<uint8_t>(eSsm2Cmd::ReadAddresses));
        CHECK(bus.tx[0].data[2] == 0x00);
        CHECK(bus.tx[0].data[3] == 0x12);
        CHECK(bus.tx[0].data[4] == 0x34);
        CHECK(bus.tx[0].data[5] == 0x56);
    }

    // ReadBlock: A0 + 3-byte addr + count-1, response is E0 + N bytes.
    void test_read_block()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);

        // Response: E0 followed by 4 data bytes -> 5-byte payload, single frame.
        QueueIsoTpSingleFrame(bus, c_engineRespId, {static_cast<uint8_t>(eSsm2Rsp::ReadBlock), 0xDE, 0xAD, 0xBE, 0xEF});

        uint8_t out[4] = {};
        CHECK_OK(client.ReadBlock(0x800000, out, sizeof(out)));

        CHECK(out[0] == 0xDE && out[1] == 0xAD && out[2] == 0xBE && out[3] == 0xEF);

        // Request: [A0 00 80 00 00 03] -> 6 bytes, SF prefix 0x06.
        CHECK(bus.tx.size() == 1);
        CHECK(bus.tx[0].data[0] == 0x06);
        CHECK(bus.tx[0].data[1] == static_cast<uint8_t>(eSsm2Cmd::ReadBlock));
        CHECK(bus.tx[0].data[2] == 0x00);  // pad
        CHECK(bus.tx[0].data[3] == 0x80);
        CHECK(bus.tx[0].data[4] == 0x00);
        CHECK(bus.tx[0].data[5] == 0x00);
        CHECK(bus.tx[0].data[6] == 0x03);  // count - 1
    }

    // ReadBlock rejects bad response sizes / codes.
    void test_read_block_protocol_error_on_short_response()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);

        // ECU returned only 3 bytes instead of 5.
        QueueIsoTpSingleFrame(bus, c_engineRespId, {static_cast<uint8_t>(eSsm2Rsp::ReadBlock), 0x11, 0x22});

        uint8_t out[4] = {};
        CHECK_STATUS(client.ReadBlock(0x100, out, sizeof(out)), eStatus::ProtocolError);
    }

    // WriteAddress: round-trip, ECU echoes the value.
    void test_write_address()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);
        CHECK(client.UnlockWrites("I UNDERSTAND THIS CAN DAMAGE MY VEHICLE"));

        QueueIsoTpSingleFrame(bus, c_engineRespId, {static_cast<uint8_t>(eSsm2Rsp::WriteAddress), 0x55});

        CHECK_OK(client.WriteAddress(0xABCDEF, 0x55));

        // Request: [B8 AB CD EF 55] -> 5 bytes, SF prefix 0x05.
        CHECK(bus.tx.size() == 1);
        CHECK(bus.tx[0].data[0] == 0x05);
        CHECK(bus.tx[0].data[1] == static_cast<uint8_t>(eSsm2Cmd::WriteAddress));
        CHECK(bus.tx[0].data[2] == 0xAB);
        CHECK(bus.tx[0].data[3] == 0xCD);
        CHECK(bus.tx[0].data[4] == 0xEF);
        CHECK(bus.tx[0].data[5] == 0x55);
    }

    // WriteAddress: ECU echoes the WRONG value -> ProtocolError.
    void test_write_address_echo_mismatch()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);
        CHECK(client.UnlockWrites("I UNDERSTAND THIS CAN DAMAGE MY VEHICLE"));

        QueueIsoTpSingleFrame(bus, c_engineRespId, {static_cast<uint8_t>(eSsm2Rsp::WriteAddress), 0x66});

        CHECK_STATUS(client.WriteAddress(0x000100, 0x55), eStatus::ProtocolError);
    }

    // WriteBlock: B0 + addr + data, ECU echoes the data.
    void test_write_block()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);
        CHECK(client.UnlockWrites("I UNDERSTAND THIS CAN DAMAGE MY VEHICLE"));

        const uint8_t data[] = {0x01, 0x02, 0x03};
        const uint8_t echo[] = {static_cast<uint8_t>(eSsm2Rsp::WriteBlock), 0x01, 0x02, 0x03};
        QueueIsoTpSingleFrame(bus, c_engineRespId, {echo[0], echo[1], echo[2], echo[3]});

        CHECK_OK(client.WriteBlock(0x000200, data, sizeof(data)));

        // Request: [B0 00 02 00 01 02 03] -> 7 bytes, SF prefix 0x07.
        CHECK(bus.tx.size() == 1);
        CHECK(bus.tx[0].data[0] == 0x07);
        CHECK(bus.tx[0].data[1] == static_cast<uint8_t>(eSsm2Cmd::WriteBlock));
        CHECK(bus.tx[0].data[2] == 0x00);
        CHECK(bus.tx[0].data[3] == 0x02);
        CHECK(bus.tx[0].data[4] == 0x00);
        CHECK(bus.tx[0].data[5] == 0x01);
        CHECK(bus.tx[0].data[6] == 0x02);
        CHECK(bus.tx[0].data[7] == 0x03);
    }

    // Pre-init guard: IsSupported() returns false before Init().
    void test_issupported_pre_init()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);
        CHECK(!client.IsInitialized());
        CHECK(!client.IsSupported(1, 1));
    }

    // Invalid args. Writes require unlock first so we actually exercise the
    // arg-validation paths (not the write-protection path).
    void test_invalid_args()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);
        CHECK(client.UnlockWrites("I UNDERSTAND THIS CAN DAMAGE MY VEHICLE"));

        uint8_t buf[4] = {};
        CHECK_STATUS(client.ReadAddresses(nullptr, 1, buf), eStatus::InvalidFrame);
        CHECK_STATUS(client.ReadBlock(0x100, nullptr, 4), eStatus::InvalidFrame);
        CHECK_STATUS(client.WriteBlock(0x100, nullptr, 4), eStatus::InvalidFrame);

        const uint32_t bigAddr[] = {0x01000000};  // > 24 bits
        CHECK_STATUS(client.ReadAddresses(bigAddr, 1, buf), eStatus::InvalidFrame);
        CHECK_STATUS(client.ReadBlock(0x01000000, buf, 4), eStatus::InvalidFrame);
        CHECK_STATUS(client.WriteAddress(0x01000000, 0), eStatus::InvalidFrame);
    }

    // Writes are blocked by default (no unlock).
    void test_writes_blocked_by_default()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);

        CHECK(!client.WritesUnlocked());
        CHECK_STATUS(client.WriteAddress(0x100, 0x55), eStatus::NotSupported);

        const uint8_t data[] = {0x01, 0x02};
        CHECK_STATUS(client.WriteBlock(0x100, data, sizeof(data)), eStatus::NotSupported);

        // Mock should NOT have seen any TX frames since writes never reached the bus.
        CHECK(bus.tx.empty());
    }

    // Wrong unlock string is a no-op.
    void test_writes_unlock_wrong_phrase()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);

        CHECK(!client.UnlockWrites("i understand this can damage my ecu"));  // wrong case
        CHECK(!client.UnlockWrites("I UNDERSTAND THIS"));                    // truncated
        CHECK(!client.UnlockWrites(""));                                     // empty
        CHECK(!client.UnlockWrites(nullptr));                                // null

        CHECK(!client.WritesUnlocked());
        CHECK_STATUS(client.WriteAddress(0x100, 0x55), eStatus::NotSupported);
    }

    // Correct phrase unlocks, then LockWrites() relocks.
    void test_writes_unlock_relock()
    {
        MockCanBus bus;
        auto       client = MakeClient(bus);

        CHECK(client.UnlockWrites("I UNDERSTAND THIS CAN DAMAGE MY VEHICLE"));
        CHECK(client.WritesUnlocked());

        client.LockWrites();
        CHECK(!client.WritesUnlocked());
        CHECK_STATUS(client.WriteAddress(0x100, 0x55), eStatus::NotSupported);
    }

    // Connect() should Open() the bus then Init(). Verify against MockCanBus.
    void test_connect_opens_and_inits()
    {
        MockCanBus bus;
        (void)bus.Close();  // start closed so we can verify Connect opens it
        CHECK(!bus.IsOpen());
        auto client = MakeClient(bus);

        // 105-byte init response (same shape as the real-car test).
        std::vector<uint8_t> payload = {0xEA, 0xA2, 0x10, 0x02, 0x51, 0x12, 0x18, 0x80, 0x07};
        while (payload.size() < 1 + c_ssmIdLen + c_romIdLen + c_capFlagsLen)
        {
            payload.push_back(0x00);
        }
        QueueIsoTpMultiFrame(bus, c_engineRespId, payload);

        tSsm2InitResponse init{};
        CHECK_OK(client.Connect(&init));
        CHECK(bus.IsOpen());
        CHECK(client.IsInitialized());
        CHECK(init.romId[0] == 0x51);
    }

    // Connect() with no bus returns BackendUnavailable, doesn't crash.
    void test_connect_no_bus()
    {
        Ssm2Client::tConfig cfg;
        cfg.bus = nullptr;
        Ssm2Client        client(cfg);
        tSsm2InitResponse init{};
        CHECK_STATUS(client.Connect(&init), eStatus::BackendUnavailable);
    }

    // Continuous-mode methods are placeholders for now.
    void test_continuous_not_supported()
    {
        MockCanBus bus;
        auto       client      = MakeClient(bus);

        const uint32_t addrs[] = {0x100};
        CHECK_STATUS(client.StartContinuous(addrs, 1), eStatus::NotSupported);

        uint8_t buf[16];
        size_t  recs = 0;
        CHECK_STATUS(client.PollContinuous(buf, sizeof(buf), &recs, 50), eStatus::NotSupported);
        CHECK_STATUS(client.StopContinuous(), eStatus::NotSupported);
    }

}  // namespace

int main()
{
    std::printf("ssm2 tests:\n");

    RUN(test_init_decodes_real_init_response);
    RUN(test_init_rejects_bad_response_code);
    RUN(test_read_addresses_single_addr_sf);
    RUN(test_read_block);
    RUN(test_read_block_protocol_error_on_short_response);
    RUN(test_write_address);
    RUN(test_write_address_echo_mismatch);
    RUN(test_write_block);
    RUN(test_writes_blocked_by_default);
    RUN(test_writes_unlock_wrong_phrase);
    RUN(test_writes_unlock_relock);
    RUN(test_connect_opens_and_inits);
    RUN(test_connect_no_bus);
    RUN(test_issupported_pre_init);
    RUN(test_invalid_args);
    RUN(test_continuous_not_supported);

    std::printf("  %d/%d checks passed\n", g_checks - g_failed, g_checks);
    return g_failed == 0 ? 0 : 1;
}
