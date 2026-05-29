// test_obd2.cpp -- Obd2Client end-to-end tests against MockCanBus.

#include "subiediag/Can.h"
#include "subiediag/Common.h"
#include "subiediag/Obd2.h"
#include "subiediag/Obd2PidTable.h"

#include "MockCanBus.h"

#include <cstdio>
#include <string.h>

#include <vector>

namespace
{

    using subiediag::c_canMaxDataLen;
    using subiediag::c_engineReqId;
    using subiediag::c_engineRespId;
    using subiediag::DescribeDtc;
    using subiediag::DescribeStatus;
    using subiediag::DtcToString;
    using subiediag::eDtcCategory;
    using subiediag::eObd2Mode;
    using subiediag::eObd2Rsp;
    using subiediag::ePid;
    using subiediag::eStatus;
    using subiediag::FindObd2Pid;
    using subiediag::IsOk;
    using subiediag::IsoTpTransport;
    using subiediag::Obd2Client;
    using subiediag::Obd2DecodePid;
    using subiediag::tDtc;
    using subiediag::tObd2PidInfo;
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

    // --- ISO-TP framing helpers (single-frame stubs) --------------------------

    void QueueIsoTpSingleFrame(MockCanBus &bus, uint32_t id, const std::vector<uint8_t> &payload)
    {
        subiediag::tCanFrame f{};
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

        subiediag::tCanFrame ff{};
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
            subiediag::tCanFrame cf{};
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

    // --- tests ----------------------------------------------------------------

    // Connect issues Mode 01 PID 00 and stores the supported-PIDs bitmap.
    void test_connect_populates_supported_pids()
    {
        MockCanBus bus;
        (void)bus.Close();
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);

        // Mode 01 PID 0x00 response with PIDs 04, 05, 0C, 0D supported
        // (bits set: 0x04=byte 0 bit 4, 0x05=byte 0 bit 3, 0x0C=byte 1 bit 4,
        // 0x0D=byte 1 bit 3, plus PID 0x20 bit indicating cascade.)
        // For test, encode bitmap = 0x18180001.
        QueueIsoTpSingleFrame(bus, c_engineRespId, {0x41, 0x00, 0x18, 0x18, 0x00, 0x01});

        CHECK_OK(client.Connect());
        CHECK(client.IsConnected());

        // Request bytes: SF len=2, [01 00] then pad
        CHECK(bus.tx.size() == 1);
        CHECK(bus.tx[0].data[0] == 0x02);
        CHECK(bus.tx[0].data[1] == static_cast<uint8_t>(eObd2Mode::CurrentData));
        CHECK(bus.tx[0].data[2] == 0x00);

        // 0x18 = 0001 1000 -> PIDs 0x04 (bit 4) and 0x05 (bit 3) supported
        CHECK(client.IsPidSupported(ePid::EngineLoad));   // 0x04
        CHECK(client.IsPidSupported(ePid::CoolantTemp));  // 0x05
        // 0x18 in byte 1 -> PIDs 0x0C, 0x0D
        CHECK(client.IsPidSupported(ePid::EngineRpm));     // 0x0C
        CHECK(client.IsPidSupported(ePid::VehicleSpeed));  // 0x0D
        // Not set: 0x06 (STFT bank 1)
        CHECK(!client.IsPidSupported(ePid::StftBank1));
        // PID 0x00 always reports supported
        CHECK(client.IsPidSupported(ePid::SupportedPids01_20));
        // Out-of-range PID (cascade) -> false in v1
        CHECK(!client.IsPidSupported(ePid::EngineFuelRate));  // 0x5E
    }

    // ReadPid for engine RPM. Response: 41 0C A B with raw 0x0FA0 -> 1000 rpm.
    void test_read_pid_engine_rpm()
    {
        MockCanBus          bus;
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);

        QueueIsoTpSingleFrame(bus, c_engineRespId, {0x41, 0x0C, 0x0F, 0xA0});

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_OK(client.ReadPid(ePid::EngineRpm, out, sizeof(out), &outLen));
        CHECK(outLen == 2);
        CHECK(out[0] == 0x0F);
        CHECK(out[1] == 0xA0);

        // Request: SF len=2, [01 0C]
        CHECK(bus.tx.size() == 1);
        CHECK(bus.tx[0].data[0] == 0x02);
        CHECK(bus.tx[0].data[1] == 0x01);
        CHECK(bus.tx[0].data[2] == 0x0C);

        // Decode: (0x0F * 256 + 0xA0) / 4 = 4000 / 4 = 1000.0
        double rpm = 0;
        CHECK(Obd2DecodePid(static_cast<uint8_t>(ePid::EngineRpm), out, outLen, &rpm));
        CHECK(rpm > 999.9 && rpm < 1000.1);
    }

    // ReadPid raw uint8 overload (caller has an unknown PID).
    void test_read_pid_raw_uint8()
    {
        MockCanBus          bus;
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);

        QueueIsoTpSingleFrame(bus, c_engineRespId, {0x41, 0x42, 0x36, 0xB0});  // CMV ~14V

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_OK(client.ReadPid(static_cast<uint8_t>(0x42), out, sizeof(out), &outLen));
        CHECK(outLen == 2);
        CHECK(out[0] == 0x36);
        CHECK(out[1] == 0xB0);

        // Decode control module voltage: (0x36B0) / 1000 = 14000/1000 = 14.0V
        double volts = 0;
        CHECK(Obd2DecodePid(0x42, out, outLen, &volts));
        CHECK(volts > 13.99 && volts < 14.01);
    }

    // PID echo mismatch -> ProtocolError.
    void test_read_pid_echo_mismatch()
    {
        MockCanBus          bus;
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);

        // Asked for 0x0C, ECU echoed 0x0D
        QueueIsoTpSingleFrame(bus, c_engineRespId, {0x41, 0x0D, 0x00, 0x40});

        uint8_t out[8] = {};
        size_t  outLen = 0;
        CHECK_STATUS(client.ReadPid(ePid::EngineRpm, out, sizeof(out), &outLen), eStatus::ProtocolError);
    }

    // Mode 03 with two DTCs: P0420 and U0101.
    void test_read_dtcs()
    {
        MockCanBus          bus;
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);

        // P0420: category P (00), code 0x0420 -> hi=0x04, lo=0x20
        // U0101: category U (11 -> top 2 bits), code 0x0101 -> hi=0xC1, lo=0x01
        QueueIsoTpSingleFrame(bus, c_engineRespId, {0x43, 0x04, 0x20, 0xC1, 0x01});

        tDtc   dtcs[8] = {};
        size_t count   = 0;
        CHECK_OK(client.ReadDtcs(dtcs, 8, &count));
        CHECK(count == 2);
        CHECK(dtcs[0].category == eDtcCategory::Powertrain);
        CHECK(dtcs[0].code == 0x0420);
        CHECK(dtcs[1].category == eDtcCategory::Network);
        CHECK(dtcs[1].code == 0x0101);

        // Format check
        char buf[6] = {};
        DtcToString(dtcs[0], buf, sizeof(buf));
        CHECK(strcmp(buf, "P0420") == 0);
        DtcToString(dtcs[1], buf, sizeof(buf));
        CHECK(strcmp(buf, "U0101") == 0);

        // Request: SF len=1, [03]
        CHECK(bus.tx.size() == 1);
        CHECK(bus.tx[0].data[0] == 0x01);
        CHECK(bus.tx[0].data[1] == static_cast<uint8_t>(eObd2Mode::ShowStoredDtcs));
    }

    // No DTCs stored: response is just [0x43].
    void test_read_dtcs_empty()
    {
        MockCanBus          bus;
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);

        QueueIsoTpSingleFrame(bus, c_engineRespId, {0x43});

        tDtc   dtcs[4] = {};
        size_t count   = 0;
        CHECK_OK(client.ReadDtcs(dtcs, 4, &count));
        CHECK(count == 0);
    }

    // Mode 04 clear: no unlock required.
    void test_clear_dtcs_no_unlock_needed()
    {
        MockCanBus          bus;
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);

        QueueIsoTpSingleFrame(bus, c_engineRespId, {0x44});

        CHECK_OK(client.ClearDtcs());

        // Request: SF len=1, [04]
        CHECK(bus.tx.size() == 1);
        CHECK(bus.tx[0].data[0] == 0x01);
        CHECK(bus.tx[0].data[1] == static_cast<uint8_t>(eObd2Mode::ClearDtcs));
    }

    // Mode 09 PID 02 - VIN over multi-frame.
    void test_get_vin()
    {
        MockCanBus          bus;
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);

        // Payload: 49 02 01 + 17-char VIN
        // Example VIN: "JF1GH7E69BG817403"
        std::vector<uint8_t> payload = {0x49, 0x02, 0x01};
        const char           vin[]   = "JF1GH7E69BG817403";
        for (size_t i = 0; i < 17; ++i)
        {
            payload.push_back(static_cast<uint8_t>(vin[i]));
        }
        QueueIsoTpMultiFrame(bus, c_engineRespId, payload);

        char   out[20] = {};
        size_t outLen  = 0;
        CHECK_OK(client.GetVin(out, sizeof(out), &outLen));
        CHECK(outLen == 17);
        CHECK(memcmp(out, vin, 17) == 0);
    }

    // PID table sanity: known PIDs are findable, decode formulas work.
    void test_pid_table_decode()
    {
        // Coolant temp 0x05: A - 40. Raw 0x5B -> 91 - 40 = 51 C
        const tObd2PidInfo *info = FindObd2Pid(0x05);
        CHECK(info != nullptr);
        if (info)
        {
            CHECK(info->bytes == 1);
            CHECK(info->name == "Coolant Temp");
        }

        const uint8_t raw[] = {0x5B};
        double        val   = 0;
        CHECK(Obd2DecodePid(0x05, raw, 1, &val));
        CHECK(val > 50.9 && val < 51.1);

        // Unknown PID -> false
        CHECK(!Obd2DecodePid(0xFF, raw, 1, &val));

        // Too few bytes for 2-byte PID -> false
        CHECK(!Obd2DecodePid(0x0C, raw, 1, &val));
    }

    // IsPidSupported returns false before Connect succeeds.
    void test_is_pid_supported_pre_connect()
    {
        MockCanBus          bus;
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);
        CHECK(!client.IsConnected());
        CHECK(!client.IsPidSupported(ePid::EngineRpm));
    }

    // Wrong response code on Mode 04 -> ProtocolError.
    void test_clear_dtcs_bad_response()
    {
        MockCanBus          bus;
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);

        // ECU returned 0x7F (negative response code) instead of 0x44.
        QueueIsoTpSingleFrame(bus, c_engineRespId, {0x7F, 0x04, 0x11});

        CHECK_STATUS(client.ClearDtcs(), eStatus::ProtocolError);
    }

    // DescribeDtc returns the right description for known codes.
    void test_describe_dtc_known()
    {
        // P0420: Catalyst System Efficiency Below Threshold (Bank 1)
        const tDtc dtc{eDtcCategory::Powertrain, 0x0420};
        const auto desc = DescribeDtc(dtc);
        CHECK(!desc.empty());
        CHECK(desc.find("Catalyst") != std::string_view::npos);
        CHECK(desc.find("Bank 1") != std::string_view::npos);

        // U0100: Lost Communication with ECM/PCM
        const tDtc u{eDtcCategory::Network, 0x0100};
        const auto u_desc = DescribeDtc(u);
        CHECK(u_desc == "Lost Communication with ECM/PCM");
    }

    // Unknown DTC returns empty string_view.
    void test_describe_dtc_unknown()
    {
        // P1234 -- manufacturer-specific, not in generic DB.
        const tDtc dtc{eDtcCategory::Powertrain, 0x1234};
        const auto desc = DescribeDtc(dtc);
        CHECK(desc.empty());
    }

    // Buffer-fill variant truncates correctly.
    void test_describe_dtc_buffer_fill()
    {
        const tDtc dtc{eDtcCategory::Powertrain, 0x0171};  // "System Too Lean (Bank 1)"

        char         buf[64] = {};
        const size_t n       = DescribeDtc(dtc, buf, sizeof(buf));
        CHECK(n > 0);
        CHECK(strcmp(buf, "System Too Lean (Bank 1)") == 0);

        // Truncate to 10 chars (+ NUL).
        char         small[10] = {};
        const size_t n2        = DescribeDtc(dtc, small, sizeof(small));
        CHECK(n2 == 9);  // outCapacity 10 - 1 NUL
        CHECK(small[9] == '\0');
        CHECK(strncmp(small, "System To", 9) == 0);

        // Unknown -> empty string in buffer, returns 0.
        const tDtc   unk{eDtcCategory::Powertrain, 0xFFFF};
        char         buf2[32] = {'X'};
        const size_t n3       = DescribeDtc(unk, buf2, sizeof(buf2));
        CHECK(n3 == 0);
        CHECK(buf2[0] == '\0');
    }

    // Null arg handling.
    void test_null_args()
    {
        MockCanBus          bus;
        IsoTpTransport      transport(&bus, c_engineReqId, c_engineRespId);
        Obd2Client::tConfig cfg;
        cfg.transport = &transport;
        Obd2Client client(cfg);

        size_t len = 0;
        CHECK_STATUS(client.ReadPid(ePid::EngineRpm, nullptr, 4, &len), eStatus::InvalidFrame);
        uint8_t buf[4] = {};
        CHECK_STATUS(client.ReadPid(ePid::EngineRpm, buf, 4, nullptr), eStatus::InvalidFrame);
        CHECK_STATUS(client.ReadDtcs(nullptr, 4, &len), eStatus::InvalidFrame);
        CHECK_STATUS(client.GetVin(nullptr, 20, &len), eStatus::InvalidFrame);
    }

}  // namespace

int main()
{
    std::printf("obd2 tests:\n");

    RUN(test_connect_populates_supported_pids);
    RUN(test_read_pid_engine_rpm);
    RUN(test_read_pid_raw_uint8);
    RUN(test_read_pid_echo_mismatch);
    RUN(test_read_dtcs);
    RUN(test_read_dtcs_empty);
    RUN(test_clear_dtcs_no_unlock_needed);
    RUN(test_get_vin);
    RUN(test_pid_table_decode);
    RUN(test_is_pid_supported_pre_connect);
    RUN(test_clear_dtcs_bad_response);
    RUN(test_describe_dtc_known);
    RUN(test_describe_dtc_unknown);
    RUN(test_describe_dtc_buffer_fill);
    RUN(test_null_args);

    std::printf("  %d/%d checks passed\n", g_checks - g_failed, g_checks);
    return g_failed == 0 ? 0 : 1;
}
