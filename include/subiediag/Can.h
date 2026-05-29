// subiediag/Can.h
//
// Raw CAN abstraction. One tCanFrame POD and one pure-virtual ICanBus.
// Concrete backends (Kvaser, SocketCAN, mocks) live in their own headers
// under subiediag/backends/ and inherit from ICanBus.
//
// Threading: a single ICanBus instance is owned by one thread at a time.
// No internal synchronization. Sharing across threads is the caller's job.
//
// Lifetime: instances passed into IsoTpTransport / Ssm2Client are held by
// non-owning pointer. The owning code (typically the app) must keep the bus
// alive for as long as any client references it.

#pragma once

#include "subiediag/Common.h"

#include <stddef.h>
#include <stdint.h>

namespace subiediag
{

    // Maximum payload size of a classic CAN 2.0 frame. CAN-FD (up to 64) is not
    // modelled; SSM2-over-CAN runs at standard 500 kbps with 8-byte payloads.
    constexpr size_t c_canMaxDataLen = 8;

    // Classic CAN 2.0 frame.
    struct tCanFrame
    {
        uint64_t timestampUs;  // monotonic microseconds, per-backend epoch
        uint32_t id;           // 11-bit standard, or 29-bit when `extended`
        uint8_t  dlc;          // 0..c_canMaxDataLen valid bytes in `data`
        uint8_t  data[c_canMaxDataLen];
        bool     extended;
    };

    // Abstract CAN bus. Implementations open a real (or mock) channel, optionally
    // install an accept-list filter, and expose blocking send / receive
    // primitives with per-call millisecond deadlines.
    class ICanBus
    {
    public:

        virtual ~ICanBus() = default;

        // Open the channel using whatever configuration the concrete backend was
        // constructed with. Idempotent: calling Open() on an already-open bus
        // returns Ok without re-acquiring resources.
        [[nodiscard]] virtual eStatus Open()               = 0;

        [[nodiscard]] virtual eStatus Close()              = 0;

        [[nodiscard]] virtual bool IsOpen() const noexcept = 0;

        // Accept-list filter on CAN IDs. count == 0 means accept all.
        // Backends may downscope to coarser hardware filter masks; any leftover
        // filtering happens in software at higher layers.
        [[nodiscard]] virtual eStatus SetRxFilter(const uint32_t *ids, size_t count) = 0;

        // Blocking send. Returns eStatus::Timeout if the frame did not make it
        // onto the wire (or into the driver tx queue, backend dependent) within
        // the deadline.
        [[nodiscard]] virtual eStatus Send(const tCanFrame &frame, uint32_t timeoutMs) = 0;

        // Blocking receive. Writes the next matching frame to *out. `out` must
        // not be null. *out is overwritten only on eStatus::Ok.
        [[nodiscard]] virtual eStatus Receive(tCanFrame *out, uint32_t timeoutMs) = 0;

    protected:

        ICanBus() = default;

        // Backends are non-copyable / non-movable; they typically hold OS handles
        // that cannot be duplicated. App owns them and passes by pointer.
        ICanBus(const ICanBus &)            = delete;
        ICanBus(ICanBus &&)                 = delete;
        ICanBus &operator=(const ICanBus &) = delete;
        ICanBus &operator=(ICanBus &&)      = delete;
    };

}  // namespace subiediag
