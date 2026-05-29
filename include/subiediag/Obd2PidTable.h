// subiediag/Obd2PidTable.h
//
// Hand-coded metadata table for the most common OBD-II Mode 01 PIDs.
//
// Coverage is intentional, not exhaustive. PIDs in here decode via a simple
// linear formula (raw_int * scale + offset) where raw_int is the response
// payload parsed as a big-endian unsigned of `bytes` bytes. PIDs whose
// decoding isn't expressible as scale+offset (O2 sensor multi-channel,
// fuel system status enumerations, etc.) are intentionally absent -- read
// those raw via Obd2Client::ReadPid() and decode in the application.
//
// To add a PID: append a row, keep entries sorted by PID for readability.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <string_view>

namespace subiediag
{

    struct tObd2PidInfo
    {
        uint8_t          pid;      // OBD-II PID byte
        uint8_t          bytes;    // response data byte count (after [0x41 PID] header)
        float            scale;    // multiplier in (raw_int * scale + offset)
        float            offset;   // additive offset
        std::string_view name;     // human-readable name
        std::string_view metric;   // unit string for display
        std::string_view formula;  // human-readable formula text (documentation)
    };

    // Common Mode 01 PIDs with linear decode formulas. Sorted by PID.
    inline constexpr std::array<tObd2PidInfo, 23> c_obd2PidTable{{
        {0x04, 1, 100.0f / 255.0f, 0.0f, "Engine Load", "%", "A * 100 / 255"},
        {0x05, 1, 1.0f, -40.0f, "Coolant Temp", "C", "A - 40"},
        {0x06, 1, 100.0f / 128.0f, -100.0f, "STFT Bank 1", "%", "(A - 128) * 100 / 128"},
        {0x07, 1, 100.0f / 128.0f, -100.0f, "LTFT Bank 1", "%", "(A - 128) * 100 / 128"},
        {0x08, 1, 100.0f / 128.0f, -100.0f, "STFT Bank 2", "%", "(A - 128) * 100 / 128"},
        {0x09, 1, 100.0f / 128.0f, -100.0f, "LTFT Bank 2", "%", "(A - 128) * 100 / 128"},
        {0x0A, 1, 3.0f, 0.0f, "Fuel Pressure", "kPa", "A * 3"},
        {0x0B, 1, 1.0f, 0.0f, "MAP", "kPa", "A"},
        {0x0C, 2, 0.25f, 0.0f, "Engine RPM", "rpm", "(A * 256 + B) / 4"},
        {0x0D, 1, 1.0f, 0.0f, "Vehicle Speed", "km/h", "A"},
        {0x0E, 1, 0.5f, -64.0f, "Timing Advance", "deg", "A / 2 - 64"},
        {0x0F, 1, 1.0f, -40.0f, "Intake Air Temp", "C", "A - 40"},
        {0x10, 2, 1.0f / 100.0f, 0.0f, "MAF", "g/s", "(A * 256 + B) / 100"},
        {0x11, 1, 100.0f / 255.0f, 0.0f, "Throttle Position", "%", "A * 100 / 255"},
        {0x1F, 2, 1.0f, 0.0f, "Run Time", "s", "A * 256 + B"},
        {0x21, 2, 1.0f, 0.0f, "Distance with MIL", "km", "A * 256 + B"},
        {0x2F, 1, 100.0f / 255.0f, 0.0f, "Fuel Tank Level", "%", "A * 100 / 255"},
        {0x33, 1, 1.0f, 0.0f, "Barometric Pressure", "kPa", "A"},
        {0x42, 2, 1.0f / 1000.0f, 0.0f, "Control Module V", "V", "(A * 256 + B) / 1000"},
        {0x46, 1, 1.0f, -40.0f, "Ambient Air Temp", "C", "A - 40"},
        {0x52, 1, 100.0f / 255.0f, 0.0f, "Ethanol Fuel %", "%", "A * 100 / 255"},
        {0x5C, 1, 1.0f, -40.0f, "Engine Oil Temp", "C", "A - 40"},
        {0x5E, 2, 0.05f, 0.0f, "Engine Fuel Rate", "L/h", "(A * 256 + B) * 0.05"},
    }};

    // Lookup a PID in the table. Returns null if not present (does NOT mean
    // the PID is unsupported -- many OBD-II PIDs don't have linear decoders
    // and so don't appear here).
    [[nodiscard]] const tObd2PidInfo *FindObd2Pid(uint8_t pid) noexcept;

    // Decode `raw[0..rawLen)` according to the PID's entry in c_obd2PidTable.
    // Returns true and writes *outValue on success. Returns false if the PID
    // isn't in the table or rawLen is too small for the PID's byte count.
    [[nodiscard]] bool Obd2DecodePid(uint8_t pid, const uint8_t *raw, size_t rawLen, double *outValue) noexcept;

}  // namespace subiediag
