// subiediag/Obd2.h
//
// OBD-II (SAE J1979 / ISO 15031) client on top of the existing ISO-TP
// transport. Read-only by design: OBD-II does not have an arbitrary RAM
// write capability (unlike SSM2), so this client has no unlock mechanism.
// Mode 04 (ClearDtcs) is a standardized reset, not a write; its only side
// effect is erasing diagnostic history and resetting the I/M readiness
// monitors -- documented at the method.
//
// Supported services in v1: 01 (current data), 03 (read DTCs), 04 (clear
// DTCs), 09 (vehicle info). Modes 02, 05-08, 0A deferred. Mode 08 would
// need a write-protection unlock if/when added.
//
// Threading: not thread-safe. Each Obd2Client is single-owner; the app
// handles whatever threading model it wants on top of synchronous calls.
//
// Timeouts: every command accepts a timeoutMs argument. Pass 0 to use
// tConfig::defaultTimeoutMs.

#pragma once

#include "subiediag/Can.h"
#include "subiediag/Common.h"
#include "subiediag/IsoTp.h"

#include <stddef.h>
#include <stdint.h>

namespace subiediag::obd2
{

    // ---------------------------------------------------------------------------
    // Protocol constants
    // ---------------------------------------------------------------------------

    // OBD-II service ("mode") bytes carried as the first byte of the request.
    enum class eObd2Mode : uint8_t
    {
        CurrentData     = 0x01,
        FreezeFrameData = 0x02,  // not implemented in v1
        ShowStoredDtcs  = 0x03,
        ClearDtcs       = 0x04,
        VehicleInfo     = 0x09,
    };

    // Positive-response codes (mode | 0x40).
    enum class eObd2Rsp : uint8_t
    {
        CurrentData     = 0x41,
        FreezeFrameData = 0x42,
        ShowStoredDtcs  = 0x43,
        ClearDtcs       = 0x44,
        VehicleInfo     = 0x49,
    };

    static_assert((static_cast<uint8_t>(eObd2Mode::CurrentData) | 0x40) == static_cast<uint8_t>(eObd2Rsp::CurrentData));
    static_assert((static_cast<uint8_t>(eObd2Mode::ShowStoredDtcs) | 0x40) == static_cast<uint8_t>(eObd2Rsp::ShowStoredDtcs));
    static_assert((static_cast<uint8_t>(eObd2Mode::ClearDtcs) | 0x40) == static_cast<uint8_t>(eObd2Rsp::ClearDtcs));
    static_assert((static_cast<uint8_t>(eObd2Mode::VehicleInfo) | 0x40) == static_cast<uint8_t>(eObd2Rsp::VehicleInfo));

    // Common Mode 01 PIDs. Not exhaustive -- use the raw uint8_t overload of
    // ReadPid() to access PIDs not listed here.
    enum class ePid : uint8_t
    {
        SupportedPids01_20   = 0x00,
        MonitorStatus        = 0x01,
        FreezeDtc            = 0x02,
        FuelSystemStatus     = 0x03,
        EngineLoad           = 0x04,
        CoolantTemp          = 0x05,
        StftBank1            = 0x06,
        LtftBank1            = 0x07,
        StftBank2            = 0x08,
        LtftBank2            = 0x09,
        FuelPressure         = 0x0A,
        Map                  = 0x0B,
        EngineRpm            = 0x0C,
        VehicleSpeed         = 0x0D,
        TimingAdvance        = 0x0E,
        IntakeAirTemp        = 0x0F,
        Maf                  = 0x10,
        ThrottlePosition     = 0x11,
        SecondaryAirStatus   = 0x12,
        O2SensorsPresent     = 0x13,
        O2Bank1Sensor1       = 0x14,
        O2Bank1Sensor2       = 0x15,
        ObdStandards         = 0x1C,
        RunTime              = 0x1F,
        SupportedPids21_40   = 0x20,
        DistanceWithMil      = 0x21,
        FuelTankLevel        = 0x2F,
        BarometricPressure   = 0x33,
        ControlModuleVoltage = 0x42,
        AmbientAirTemp       = 0x46,
        EthanolFuelPct       = 0x52,
        EngineOilTemp        = 0x5C,
        EngineFuelRate       = 0x5E,
    };

    // Mode 09 InfoType bytes.
    constexpr uint8_t c_infoVin = 0x02;

    // The VIN is always 17 ASCII characters per ISO 3779.
    constexpr size_t c_vinLength = 17;

    // ISO 15031-5 / ISO 14229-2 negative-response markers.
    constexpr uint8_t c_negRespSid = 0x7F;
    constexpr uint8_t c_nrcRcrRp   = 0x78;  // requestCorrectlyReceived-ResponsePending

    // P2*CAN_max (ISO 15765-4) -- timeout the client extends to after seeing
    // NRC 0x78. Spec defines 5 000 ms.
    constexpr uint32_t c_rcrRpTimeoutMs = 5000;

    // ---------------------------------------------------------------------------
    // DTC type and formatter
    // ---------------------------------------------------------------------------

    // DTC category (the first character of an SAE J2012-formatted code).
    enum class eDtcCategory : uint8_t
    {
        Powertrain = 0,  // 'P'
        Chassis    = 1,  // 'C'
        Body       = 2,  // 'B'
        Network    = 3,  // 'U'
    };

    // A decoded OBD-II DTC. `code` is 14 bits (0x0000..0x3FFF) and is rendered
    // as 4 hex digits in the standard format (P0420, C1234, etc.).
    struct tDtc
    {
        eDtcCategory category;
        uint16_t     code;
    };

    // Format `dtc` into a NUL-terminated 5-char string ("P0420\0") in `out`.
    // `outCapacity` must be at least 6. No-op if buffer is too small.
    void DtcToString(tDtc dtc, char *out, size_t outCapacity) noexcept;

    // Look up a verbose description for `dtc` in the generic OBD-II DTC
    // database (Obd2DtcDb.h, generated from obd2_generic_dtcs.csv).
    //
    // Returns a view into a static string literal owned by the DTC table;
    // valid for the entire program lifetime. Returns an empty view ({}) if
    // the DTC isn't in the table -- the database covers common SAE-defined
    // generic codes only; manufacturer-specific (P1xxx, P30xx+) codes are
    // not included.
    [[nodiscard]] std::string_view DescribeDtc(tDtc dtc) noexcept;

    // Buffer-fill variant: writes a NUL-terminated description into `out`
    // (truncating to outCapacity-1 chars if needed). Returns the number of
    // chars written excluding the NUL. Writes 0 and returns 0 for unknown
    // DTCs (and for outCapacity == 0).
    size_t DescribeDtc(tDtc dtc, char *out, size_t outCapacity) noexcept;

    // ---------------------------------------------------------------------------
    // Client
    // ---------------------------------------------------------------------------

    class Obd2Client
    {
    public:

        struct tConfig
        {
            // Non-owning pointer to the ISO-TP transport bound to the target
            // ECU's (reqId, respId, padByte). App constructs the transport
            // and keeps it alive for the client's lifetime. The same
            // transport can be shared with an Ssm2Client addressing the same
            // ECU.
            isotp::IsoTpTransport *transport        = nullptr;
            uint32_t               defaultTimeoutMs = c_defaultTimeoutMs;
        };

        explicit Obd2Client(const tConfig &cfg) noexcept;

        Obd2Client(const Obd2Client &)            = delete;
        Obd2Client(Obd2Client &&)                 = delete;
        Obd2Client &operator=(const Obd2Client &) = delete;
        Obd2Client &operator=(Obd2Client &&)      = delete;

        // -------- one-shot commands --------------------------------------------

        // Open the underlying bus (via the transport) and query Mode 01
        // PID 0x00 to populate the supported-PIDs bitmap. Equivalent to
        // transport->Bus()->Open() + one ReadPid(SupportedPids01_20).
        [[nodiscard]] eStatus Connect(uint32_t timeoutMs = 0);

        // Mode 01 - read one PID. `out` receives the raw data bytes (without
        // the [0x41 PID] response header); *outLen is set to the byte count.
        [[nodiscard]] eStatus ReadPid(ePid pid, uint8_t *out, size_t outCapacity, size_t *outLen, uint32_t timeoutMs = 0);
        [[nodiscard]] eStatus ReadPid(uint8_t pid, uint8_t *out, size_t outCapacity, size_t *outLen, uint32_t timeoutMs = 0);

        // Mode 03 - read stored DTCs. Writes up to `outCapacity` decoded DTCs
        // into `out` and sets *outCount. Returns Overrun if the ECU reports
        // more DTCs than outCapacity.
        [[nodiscard]] eStatus ReadDtcs(tDtc *out, size_t outCapacity, size_t *outCount, uint32_t timeoutMs = 0);

        // Mode 04 - Clear emissions-related DTCs.
        //
        // WARNING: this erases all stored DTCs and freeze-frame data, AND
        // resets the I/M readiness monitors to "not ready". A drive cycle of
        // varied driving (~50-100 miles) is required for monitors to set
        // again; during that period the vehicle will FAIL an emissions
        // inspection. This does NOT modify calibration or RAM -- the ECU
        // will re-detect any real fault next time conditions trigger it.
        //
        // No unlock required. OBD-II has no arbitrary RAM-write capability,
        // so this is the most "writy" thing standard OBD-II can do.
        [[nodiscard]] eStatus ClearDtcs(uint32_t timeoutMs = 0);

        // Mode 09 PID 0x02 - read the Vehicle Identification Number.
        //
        // Writes the 17-char VIN into `out` and appends a NUL terminator,
        // so `outCapacity` must be at least c_vinLength + 1 = 18. Returns
        // Overrun if the buffer is smaller. *outLen is set to the VIN
        // character count (17), matching strlen() of the returned C string;
        // the NUL is not counted.
        [[nodiscard]] eStatus GetVin(char *out, size_t outCapacity, size_t *outLen, uint32_t timeoutMs = 0);

        // -------- post-Connect accessors ---------------------------------------

        bool IsConnected() const noexcept { return m_connected; }

        // True iff PID is in the Mode 01 PID 0x00 supported-PIDs bitmap.
        // Always returns false before Connect() succeeds. Always returns
        // false for PIDs > 0x20 in v1 (cascade discovery beyond 0x20 is
        // deferred); call ReadPid() directly to access those.
        bool IsPidSupported(ePid pid) const noexcept;
        bool IsPidSupported(uint8_t pid) const noexcept;

        // Raw supported-PIDs bitmap for PIDs 0x01..0x20.
        // Bit (32 - N) corresponds to PID N. Bit 31 = PID 0x01.
        uint32_t SupportedPids01_20() const noexcept { return m_supportedPids00; }

        const tConfig &Config() const noexcept { return m_cfg; }

    private:

        uint32_t EffectiveTimeoutMs(uint32_t timeoutMs) const noexcept;

        // True iff cfg.transport is a usable non-null pointer.
        bool HasTransport() const noexcept { return m_cfg.transport != nullptr; }

        // Wraps transport->SendRequest + ReceiveResponse with NRC 0x78
        // (RCR-RP / response-pending) handling: on a negative response whose
        // NRC is 0x78, re-arms the receive timer to P2*CAN_max and waits for
        // the real reply, per ISO 15765-4. Any other negative response is
        // returned to the caller as-is in *resp / *respLen.
        eStatus ExchangeWithPendingRetry(const uint8_t *req,
                                         size_t         reqLen,
                                         uint8_t       *resp,
                                         size_t         respCap,
                                         size_t        *respLen,
                                         uint32_t       timeoutMs);

        tConfig  m_cfg;
        bool     m_connected       = false;
        uint32_t m_supportedPids00 = 0;  // PIDs 0x01..0x20 bitmap, MSB = 0x01
    };

}  // namespace subiediag::obd2
