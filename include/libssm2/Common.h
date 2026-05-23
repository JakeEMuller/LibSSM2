// libssm2/Common.h
//
// Shared types used across the library.
//
// The eStatus enum is the universal return value: callers should treat any
// value other than Ok as a failure, optionally translating to a human-
// readable form via DescribeStatus(). The enum is one byte wide so it fits
// in registers and can be returned cheaply from every API call.

#pragma once

#include <stdint.h>
#include <string_view>

namespace libssm2
{

    // Library-wide return code. Granular by design so future logging can surface
    // the precise failure mode without changing the API.
    enum class eStatus : uint8_t
    {
        Ok = 0,
        Timeout,             // bus or response did not arrive within the deadline
        NotOpen,             // operation attempted on a closed / not-yet-opened bus
        BackendUnavailable,  // hardware or driver could not be reached
        InvalidFrame,        // malformed ISO-TP or SSM2 framing on the wire
        SequenceError,       // ISO-TP consecutive-frame sequence number broken
        FlowControlAbort,    // peer indicated overflow / refused continuation
        ChecksumError,       // reserved for K-line; unused on CAN
        NotInitialized,      // Ssm2Client method called before Init()
        NotSupported,        // capability flag not set, or unknown command
        Overrun,             // caller-provided buffer too small for response
        ProtocolError,       // ECU returned a negative or malformed reply
        BusError,            // CAN bus-off / error frame
    };

    [[nodiscard]] constexpr bool IsOk(eStatus s) noexcept
    {
        return s == eStatus::Ok;
    }

    // Returns a view onto a static string literal. Never empty.
    [[nodiscard]] constexpr std::string_view DescribeStatus(eStatus s) noexcept
    {
        switch (s)
        {
        case eStatus::Ok:
            return "ok";
        case eStatus::Timeout:
            return "timeout";
        case eStatus::NotOpen:
            return "bus not open";
        case eStatus::BackendUnavailable:
            return "backend unavailable";
        case eStatus::InvalidFrame:
            return "invalid frame";
        case eStatus::SequenceError:
            return "ISO-TP sequence error";
        case eStatus::FlowControlAbort:
            return "ISO-TP flow control abort";
        case eStatus::ChecksumError:
            return "checksum error";
        case eStatus::NotInitialized:
            return "client not initialized";
        case eStatus::NotSupported:
            return "not supported";
        case eStatus::Overrun:
            return "buffer overrun";
        case eStatus::ProtocolError:
            return "protocol error";
        case eStatus::BusError:
            return "bus error";
        }
        return "unknown";
    }

}  // namespace libssm2
