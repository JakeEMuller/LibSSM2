// test_uds.cpp -- UdsClient end-to-end tests against MockCanBus.

#include "subiediag/Can.h"
#include "subiediag/Common.h"
#include "subiediag/Obd2PidTable.h"
#include "subiediag/Uds.h"

#include "MockCanBus.h"

#include <cstdio>
#include <string.h>

#include <vector>

namespace
{

    using subiediag::can::c_canMaxDataLen;
    using subiediag::can::c_engineReqId;
    using subiediag::can::c_engineRespId;
    using subiediag::DescribeStatus;
    using subiediag::eStatus;
    using subiediag::IsOk;
    using subiediag::isotp::IsoTpTransport;
    using subiediag::uds::c_nrcConditionsNotCorrect;
    using subiediag::uds::c_nrcRequestOutOfRange;
    using subiediag::uds::DecodeObdDid;
    using subiediag::uds::DidForObd2Pid;
    using subiediag::uds::eDiagnosticSession;
    using subiediag::uds::IsObdDid;
    using subiediag::uds::tDidRead;
    using subiediag::uds::tSessionTiming;
    using subiediag::uds::UdsClient;
    using subiediag_test::MockCanBus;

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

    // --- ISO-TP framing helpers (mirrors test_obd2) -----------------------------

    void QueueIsoTpSingleFrame(MockCanBus &bus, uint32_t id, const std::vector<uint8_t> &payload)
    {
        subiediag::can::tCanFrame f{};
        f.id      = id;
        f.dlc     = c_canMaxDataLen;
        f.data[0] = static_cast<uint8_t>(payload.size());
        for (size_t i = 0; i < payload.size(); ++i)
        {
            f.data[1 + i] = payload[i];
        }
        bus.rx.push_back(f);
    }

    void QueueIsoTpMultiFrame(MockCanBus &bus, uint32_t id, const std::vector<uint8_t> &payload)
    {
        const uint16_t total = static_cast<uint16_t>(payload.size());

        subiediag::can::tCanFrame ff{};
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
            subiediag::can::tCanFrame cf{};
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

    struct Fixture
    {
        MockCanBus     bus;
        IsoTpTransport transport{&bus, c_engineReqId, c_engineRespId};
        UdsClient      client{MakeConfig(&transport)};

        static UdsClient::tConfig MakeConfig(IsoTpTransport *t)
        {
            UdsClient::tConfig cfg;
            cfg.transport = t;
            return cfg;
        }
    };

    // --- tests ----------------------------------------------------------------

    // Connect opens the bus and probes with TesterPresent.
    void test_connect_probes_tester_present()
    {
        Fixture f;
        (void)f.bus.Close();

        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x7E, 0x00});

        CHECK_OK(f.client.Connect());
        CHECK(f.client.IsConnected());
        CHECK(f.bus.IsOpen());

        // Request bytes: SF len=2, [3E 00]
        CHECK(f.bus.tx.size() == 1);
        CHECK(f.bus.tx[0].data[0] == 0x02);
        CHECK(f.bus.tx[0].data[1] == 0x3E);
        CHECK(f.bus.tx[0].data[2] == 0x00);
    }

    // A car with no UDS server: the probe times out and Connect fails.
    void test_connect_no_uds_server_times_out()
    {
        Fixture f;
        CHECK_STATUS(f.client.Connect(), eStatus::Timeout);
        CHECK(!f.client.IsConnected());
    }

    // Single-DID read of the RPM OBD-mirror DID (0xF40C).
    void test_read_did_obd_mirror_rpm()
    {
        Fixture f;
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x62, 0xF4, 0x0C, 0x0F, 0xA0});

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_OK(f.client.ReadDid(DidForObd2Pid(0x0C), out, sizeof(out), &outLen));
        CHECK(outLen == 2);
        CHECK(out[0] == 0x0F);
        CHECK(out[1] == 0xA0);

        // Request: SF len=3, [22 F4 0C]
        CHECK(f.bus.tx.size() == 1);
        CHECK(f.bus.tx[0].data[0] == 0x03);
        CHECK(f.bus.tx[0].data[1] == 0x22);
        CHECK(f.bus.tx[0].data[2] == 0xF4);
        CHECK(f.bus.tx[0].data[3] == 0x0C);

        // Decode with the mirrored PID's linear formula: 0x0FA0 / 4 = 1000 rpm.
        double rpm = 0;
        CHECK(DecodeObdDid(DidForObd2Pid(0x0C), out, outLen, &rpm));
        CHECK(rpm > 999.9 && rpm < 1000.1);
    }

    // DID echo mismatch -> ProtocolError.
    void test_read_did_echo_mismatch()
    {
        Fixture f;
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x62, 0xF4, 0x0D, 0x40});

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_STATUS(f.client.ReadDid(DidForObd2Pid(0x0C), out, sizeof(out), &outLen),
                     eStatus::ProtocolError);
    }

    // NRC 0x31 (requestOutOfRange) -> NotSupported, with the NRC readable.
    // This is the per-DID support probe path.
    void test_read_did_out_of_range_maps_to_not_supported()
    {
        Fixture f;
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x7F, 0x22, c_nrcRequestOutOfRange});

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_STATUS(f.client.ReadDid(0x1234, out, sizeof(out), &outLen), eStatus::NotSupported);
        CHECK(f.client.LastNrc() == c_nrcRequestOutOfRange);
    }

    // Other NRCs (e.g. conditionsNotCorrect) -> ProtocolError, NRC readable.
    void test_read_did_other_nrc_maps_to_protocol_error()
    {
        Fixture f;
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x7F, 0x22, c_nrcConditionsNotCorrect});

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_STATUS(f.client.ReadDid(0x1234, out, sizeof(out), &outLen), eStatus::ProtocolError);
        CHECK(f.client.LastNrc() == c_nrcConditionsNotCorrect);
    }

    // A successful call after a negative one clears LastNrc.
    void test_last_nrc_clears_on_success()
    {
        Fixture f;
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x7F, 0x22, c_nrcRequestOutOfRange});
        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_STATUS(f.client.ReadDid(0x1234, out, sizeof(out), &outLen), eStatus::NotSupported);
        CHECK(f.client.LastNrc() == c_nrcRequestOutOfRange);

        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x62, 0xF4, 0x0C, 0x0F, 0xA0});
        CHECK_OK(f.client.ReadDid(DidForObd2Pid(0x0C), out, sizeof(out), &outLen));
        CHECK(f.client.LastNrc() == 0);
    }

    // NRC 0x78 (response pending) re-arms the wait; the real reply lands.
    void test_read_did_response_pending_then_ok()
    {
        Fixture f;
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x7F, 0x22, 0x78});
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x7F, 0x22, 0x78});
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x62, 0xF4, 0x05, 0x8C});

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_OK(f.client.ReadDid(DidForObd2Pid(0x05), out, sizeof(out), &outLen));
        CHECK(outLen == 1);
        CHECK(out[0] == 0x8C);  // coolant: 0x8C - 40 = 100 C
    }

    // Multi-DID read: three records split on the caller-supplied lengths.
    void test_read_dids_multi()
    {
        Fixture f;
        // [62] [F40C 0FA0] [F405 8C] [F40D 64]
        QueueIsoTpMultiFrame(f.bus, c_engineRespId,
                             {0x62, 0xF4, 0x0C, 0x0F, 0xA0, 0xF4, 0x05, 0x8C, 0xF4, 0x0D, 0x64});
        // The transport must see a flow-control frame to stream the CFs.
        // MockCanBus ignores tx during receive, so nothing extra needed --
        // IsoTpTransport sends FC into bus.tx and reads CFs from bus.rx.

        uint8_t rpm[2] = {}, clt[1] = {}, spd[1] = {};
        tDidRead reads[3] = {
            {DidForObd2Pid(0x0C), rpm, sizeof(rpm), 2, 0},
            {DidForObd2Pid(0x05), clt, sizeof(clt), 1, 0},
            {DidForObd2Pid(0x0D), spd, sizeof(spd), 1, 0},
        };
        CHECK_OK(f.client.ReadDids(reads, 3));
        CHECK(reads[0].len == 2);
        CHECK(reads[1].len == 1);
        CHECK(reads[2].len == 1);
        CHECK(rpm[0] == 0x0F && rpm[1] == 0xA0);
        CHECK(clt[0] == 0x8C);
        CHECK(spd[0] == 0x64);

        // Request: [22 F40C F405 F40D] = 7 bytes, single frame.
        CHECK(!f.bus.tx.empty());
        CHECK(f.bus.tx[0].data[0] == 0x07);
        CHECK(f.bus.tx[0].data[1] == 0x22);
    }

    // Server omits an unsupported DID mid-list: its len stays 0, the rest parse.
    void test_read_dids_server_omits_unsupported()
    {
        Fixture f;
        // Asked for F40C, F45E (unsupported), F40D -- server answers 2 of 3.
        // 8 payload bytes exceeds a single frame's 7, so it's a multi-frame.
        QueueIsoTpMultiFrame(f.bus, c_engineRespId, {0x62, 0xF4, 0x0C, 0x0F, 0xA0, 0xF4, 0x0D, 0x64});

        uint8_t rpm[2] = {}, fuel[2] = {}, spd[1] = {};
        tDidRead reads[3] = {
            {DidForObd2Pid(0x0C), rpm, sizeof(rpm), 2, 0},
            {DidForObd2Pid(0x5E), fuel, sizeof(fuel), 2, 0},
            {DidForObd2Pid(0x0D), spd, sizeof(spd), 1, 0},
        };
        CHECK_OK(f.client.ReadDids(reads, 3));
        CHECK(reads[0].len == 2);
        CHECK(reads[1].len == 0);  // omitted by the server
        CHECK(reads[2].len == 1);
        CHECK(spd[0] == 0x64);
    }

    // A record for a DID we never asked for fails the parse.
    void test_read_dids_unrequested_did_is_protocol_error()
    {
        Fixture f;
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x62, 0xF4, 0x11, 0x40});

        uint8_t rpm[2] = {};
        tDidRead reads[1] = {
            {DidForObd2Pid(0x0C), rpm, sizeof(rpm), 2, 0},
        };
        CHECK_STATUS(f.client.ReadDids(reads, 1), eStatus::ProtocolError);
    }

    // A truncated final record fails the parse.
    void test_read_dids_truncated_record_is_protocol_error()
    {
        Fixture f;
        // F40C claims 2 bytes but only 1 arrives.
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x62, 0xF4, 0x0C, 0x0F});

        uint8_t rpm[2] = {};
        tDidRead reads[1] = {
            {DidForObd2Pid(0x0C), rpm, sizeof(rpm), 2, 0},
        };
        CHECK_STATUS(f.client.ReadDids(reads, 1), eStatus::ProtocolError);
    }

    // DiagnosticSessionControl parses the timing record and re-arms the
    // post-0x78 wait from P2*server_max.
    void test_session_control_parses_timing()
    {
        Fixture f;
        // [50 03] P2=0x0032 (50 ms) P2*=0x01F4 (500 * 10 ms = 5 s)
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x50, 0x03, 0x00, 0x32, 0x01, 0xF4});

        tSessionTiming timing{};
        CHECK_OK(f.client.DiagnosticSessionControl(eDiagnosticSession::Extended, &timing));
        CHECK(timing.p2ServerMaxMs == 50);
        CHECK(timing.p2StarServerMax10 == 500);
        CHECK(f.client.LastSessionTiming().p2ServerMaxMs == 50);

        // Request: [10 03]
        CHECK(f.bus.tx.size() == 1);
        CHECK(f.bus.tx[0].data[0] == 0x02);
        CHECK(f.bus.tx[0].data[1] == 0x10);
        CHECK(f.bus.tx[0].data[2] == 0x03);
    }

    // Session echo mismatch -> ProtocolError.
    void test_session_control_echo_mismatch()
    {
        Fixture f;
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x50, 0x01, 0x00, 0x32, 0x01, 0xF4});
        CHECK_STATUS(f.client.DiagnosticSessionControl(eDiagnosticSession::Extended),
                     eStatus::ProtocolError);
    }

    // TesterPresent with suppressed response: one frame out, no receive.
    void test_tester_present_suppressed_sends_only()
    {
        Fixture f;
        CHECK_OK(f.client.TesterPresent(/*suppressResponse=*/true));
        CHECK(f.bus.tx.size() == 1);
        CHECK(f.bus.tx[0].data[0] == 0x02);
        CHECK(f.bus.tx[0].data[1] == 0x3E);
        CHECK(f.bus.tx[0].data[2] == 0x80);  // suppress bit set
        CHECK(f.bus.rx.empty());             // nothing was consumed / needed
    }

    // A long DID record (VIN, 17 bytes) arrives as an ISO-TP multi-frame.
    void test_read_did_multiframe_vin()
    {
        Fixture f;
        std::vector<uint8_t> payload = {0x62, 0xF1, 0x90};
        const char          *vin     = "JF1GH7E66CG123456";
        for (size_t i = 0; i < 17; ++i)
        {
            payload.push_back(static_cast<uint8_t>(vin[i]));
        }
        QueueIsoTpMultiFrame(f.bus, c_engineRespId, payload);

        uint8_t out[32] = {};
        size_t  outLen  = 0;
        CHECK_OK(f.client.ReadDid(0xF190, out, sizeof(out), &outLen));
        CHECK(outLen == 17);
        CHECK(memcmp(out, vin, 17) == 0);
    }

    // Undersized caller buffer -> Overrun.
    void test_read_did_overrun()
    {
        Fixture f;
        QueueIsoTpSingleFrame(f.bus, c_engineRespId, {0x62, 0xF4, 0x0C, 0x0F, 0xA0});

        uint8_t out[1] = {};
        size_t  outLen = 0;
        CHECK_STATUS(f.client.ReadDid(DidForObd2Pid(0x0C), out, sizeof(out), &outLen),
                     eStatus::Overrun);
    }

    // Helper coverage + bad-argument paths.
    void test_helpers_and_null_args()
    {
        CHECK(DidForObd2Pid(0x0C) == 0xF40C);
        CHECK(IsObdDid(0xF400));
        CHECK(IsObdDid(0xF4FF));
        CHECK(!IsObdDid(0xF500));
        CHECK(!IsObdDid(0x1234));

        double v = 0;
        const uint8_t raw[2] = {0x0F, 0xA0};
        CHECK(!DecodeObdDid(0xF190, raw, 2, &v));   // not an OBD-mirror DID
        CHECK(!DecodeObdDid(0xF4FF, raw, 2, &v));   // PID 0xFF not in the table
        CHECK(DecodeObdDid(0xF40C, raw, 2, &v));
        CHECK(v > 999.9 && v < 1000.1);

        Fixture f;
        size_t  len = 0;
        CHECK_STATUS(f.client.ReadDid(0xF40C, nullptr, 4, &len), eStatus::InvalidFrame);
        uint8_t buf[4] = {};
        CHECK_STATUS(f.client.ReadDid(0xF40C, buf, 4, nullptr), eStatus::InvalidFrame);
        CHECK_STATUS(f.client.ReadDids(nullptr, 1), eStatus::InvalidFrame);

        tDidRead zeroLen[1] = {{0xF40C, buf, sizeof(buf), 0, 0}};
        CHECK_STATUS(f.client.ReadDids(zeroLen, 1), eStatus::InvalidFrame);
    }

}  // namespace

int main()
{
    std::printf("uds tests:\n");

    RUN(test_connect_probes_tester_present);
    RUN(test_connect_no_uds_server_times_out);
    RUN(test_read_did_obd_mirror_rpm);
    RUN(test_read_did_echo_mismatch);
    RUN(test_read_did_out_of_range_maps_to_not_supported);
    RUN(test_read_did_other_nrc_maps_to_protocol_error);
    RUN(test_last_nrc_clears_on_success);
    RUN(test_read_did_response_pending_then_ok);
    RUN(test_read_dids_multi);
    RUN(test_read_dids_server_omits_unsupported);
    RUN(test_read_dids_unrequested_did_is_protocol_error);
    RUN(test_read_dids_truncated_record_is_protocol_error);
    RUN(test_session_control_parses_timing);
    RUN(test_session_control_echo_mismatch);
    RUN(test_tester_present_suppressed_sends_only);
    RUN(test_read_did_multiframe_vin);
    RUN(test_read_did_overrun);
    RUN(test_helpers_and_null_args);

    std::printf("  %d/%d checks passed\n", g_checks - g_failed, g_checks);
    return g_failed == 0 ? 0 : 1;
}
