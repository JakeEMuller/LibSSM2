// subiediag/backends/Kvaser.h
//
// CANLIB-backed ICanBus implementation for Kvaser hardware (USBcan II,
// Memorator, Leaf, ...). Windows-only.
//
// Build: gated by the CMake option SUBIEDIAG_BACKEND_KVASER. CMake auto-
// detects the Kvaser SDK under C:\Program Files (x86)\Kvaser\Canlib.
//
// This header intentionally does NOT include canlib.h or windows.h --
// consumers see only a forward-declared opaque handle. CANLIB internals
// stay confined to the .cpp.

#pragma once

#include "subiediag/Can.h"
#include "subiediag/Common.h"

#include <stddef.h>
#include <stdint.h>

namespace subiediag::backends
{

    // Returned by KvaserCanBus::ListChannels(). One per CANLIB-visible channel.
    struct tChannelInfo
    {
        int  channel;    // index to pass to KvaserCanBus::tConfig::channel
        char name[128];  // NUL-terminated device description from CANLIB
    };

    class KvaserCanBus : public subiediag::ICanBus
    {
    public:

        struct tConfig
        {
            int  channel     = 0;      // CANLIB channel index (0, 1, ...)
            int  bitrateKbps = 500;    // SSM2-over-CAN is universally 500
            bool exclusive   = false;  // pass canOPEN_EXCLUSIVE to canOpenChannel
        };

        explicit KvaserCanBus(const tConfig &cfg) noexcept;
        ~KvaserCanBus() override;

        // ICanBus overrides
        [[nodiscard]] subiediag::eStatus Open() override;
        [[nodiscard]] subiediag::eStatus Close() override;
        bool                           IsOpen() const noexcept override;

        // No-op on Kvaser by design. The IsoTp layer filters by respId
        // already; hardware filtering on Kvaser adds complexity without
        // material benefit for our use case.
        [[nodiscard]] subiediag::eStatus SetRxFilter(const uint32_t *ids, size_t count) override;

        [[nodiscard]] subiediag::eStatus Send(const subiediag::tCanFrame &frame, uint32_t timeoutMs) override;
        [[nodiscard]] subiediag::eStatus Receive(subiediag::tCanFrame *out, uint32_t timeoutMs) override;

        // Enumerate CANLIB-visible channels. Writes up to `maxChannels`
        // entries into `out` and returns the total CANLIB reports. If
        // total > maxChannels, the extras aren't written. `out` may be
        // null when maxChannels == 0 (counts only).
        static size_t ListChannels(tChannelInfo *out, size_t maxChannels) noexcept;

    private:

        tConfig m_cfg;
        int     m_handle = -1;  // canHandle, -1 when closed
        bool    m_onBus  = false;
    };

}  // namespace subiediag::backends
