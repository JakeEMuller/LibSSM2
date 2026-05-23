// libssm2/backends/Kvaser.h
//
// CANLIB-backed ICanBus implementation for Kvaser hardware (USBcan II,
// Memorator, Leaf, ...). Windows-only.
//
// Build: gated by the CMake option LIBSSM2_BACKEND_KVASER. CMake auto-
// detects the Kvaser SDK under C:\Program Files (x86)\Kvaser\Canlib.
//
// This header intentionally does NOT include canlib.h or windows.h --
// consumers see only a forward-declared opaque handle. CANLIB internals
// stay confined to the .cpp.

#pragma once

#include "libssm2/Can.h"
#include "libssm2/Common.h"

#include <stddef.h>
#include <stdint.h>

namespace libssm2::backends
{

    // Returned by KvaserCanBus::ListChannels(). One per CANLIB-visible channel.
    struct tChannelInfo
    {
        int  channel;    // index to pass to KvaserCanBus::tConfig::channel
        char name[128];  // NUL-terminated device description from CANLIB
    };

    class KvaserCanBus : public libssm2::ICanBus
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
        [[nodiscard]] libssm2::eStatus Open() override;
        [[nodiscard]] libssm2::eStatus Close() override;
        bool                           IsOpen() const noexcept override;

        // No-op on Kvaser by design. The IsoTp layer filters by respId
        // already; hardware filtering on Kvaser adds complexity without
        // material benefit for our use case.
        [[nodiscard]] libssm2::eStatus SetRxFilter(const uint32_t *ids, size_t count) override;

        [[nodiscard]] libssm2::eStatus Send(const libssm2::tCanFrame &frame, uint32_t timeoutMs) override;
        [[nodiscard]] libssm2::eStatus Receive(libssm2::tCanFrame *out, uint32_t timeoutMs) override;

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

}  // namespace libssm2::backends
