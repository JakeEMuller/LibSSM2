// Kvaser.cpp -- CANLIB-backed ICanBus implementation.
//
// All canlib.h symbols stay confined to this translation unit. The header
// only forward-declares the integer handle so consumers don't pull in
// windows.h transitively.

#include "subiediag/backends/Kvaser.h"

#include <canlib.h>

#include <mutex>
#include <string.h>

namespace subiediag::backends
{

    namespace
    {

        // canInitializeLibrary() must run once per process before any other
        // CANLIB call. There's no documented downside to calling it again,
        // but call_once keeps things clean.
        std::once_flag g_canlibInitFlag;

        void EnsureCanlibInit() noexcept
        {
            std::call_once(g_canlibInitFlag, []() { canInitializeLibrary(); });
        }

        subiediag::eStatus FromCanlibStatus(canStatus s) noexcept
        {
            switch (s)
            {
            case canOK:
                return subiediag::eStatus::Ok;
            case canERR_TIMEOUT:
                return subiediag::eStatus::Timeout;
            case canERR_NOMSG:
                return subiediag::eStatus::Timeout;
            case canERR_PARAM:
                return subiediag::eStatus::InvalidFrame;
            case canERR_NOMEM:
                return subiediag::eStatus::BackendUnavailable;
            case canERR_HARDWARE:
                return subiediag::eStatus::BackendUnavailable;
            case canERR_NOTFOUND:
                return subiediag::eStatus::BackendUnavailable;
            case canERR_NOCHANNELS:
                return subiediag::eStatus::BackendUnavailable;
            case canERR_NOCARD:
                return subiediag::eStatus::BackendUnavailable;
            case canERR_NOTINITIALIZED:
                return subiediag::eStatus::NotOpen;
            case canERR_INVHANDLE:
                return subiediag::eStatus::NotOpen;
            case canERR_INTERRUPTED:
                return subiediag::eStatus::Timeout;
            case canERR_DRIVER:
                return subiediag::eStatus::BackendUnavailable;
            case canERR_DRIVERLOAD:
                return subiediag::eStatus::BackendUnavailable;
            case canERR_DRIVERFAILED:
                return subiediag::eStatus::BackendUnavailable;
            case canERR_TXBUFOFL:
                return subiediag::eStatus::BusError;
            case canERR_NOT_SUPPORTED:
                return subiediag::eStatus::NotSupported;
            default:
                return subiediag::eStatus::BackendUnavailable;
            }
        }

        // Map a kbps value to CANLIB's bitrate constant. Falls back to 500K.
        long BitrateConstant(int kbps) noexcept
        {
            switch (kbps)
            {
            case 1000:
                return canBITRATE_1M;
            case 500:
                return canBITRATE_500K;
            case 250:
                return canBITRATE_250K;
            case 125:
                return canBITRATE_125K;
            case 100:
                return canBITRATE_100K;
            case 83:
                return canBITRATE_83K;
            case 62:
                return canBITRATE_62K;
            case 50:
                return canBITRATE_50K;
            case 10:
                return canBITRATE_10K;
            default:
                return canBITRATE_500K;
            }
        }

    }  // namespace

    // ---------------------------------------------------------------------------

    KvaserCanBus::KvaserCanBus(const tConfig &cfg) noexcept
        : m_cfg(cfg)
    {
    }

    KvaserCanBus::~KvaserCanBus()
    {
        if (m_handle >= 0)
        {
            (void)Close();
        }
    }

    bool KvaserCanBus::IsOpen() const noexcept
    {
        return m_handle >= 0 && m_onBus;
    }

    subiediag::eStatus KvaserCanBus::Open()
    {
        if (IsOpen())
        {
            return subiediag::eStatus::Ok;
        }
        EnsureCanlibInit();

        const int       flags = m_cfg.exclusive ? canOPEN_EXCLUSIVE : 0;
        const canHandle h     = canOpenChannel(m_cfg.channel, flags);
        if (h < 0)
        {
            return FromCanlibStatus(static_cast<canStatus>(h));
        }
        m_handle    = static_cast<int>(h);

        canStatus s = canSetBusParams(h, BitrateConstant(m_cfg.bitrateKbps), 0, 0, 0, 0, 0);
        if (s != canOK)
        {
            (void)canClose(h);
            m_handle = -1;
            return FromCanlibStatus(s);
        }

        s = canSetBusOutputControl(h, canDRIVER_NORMAL);
        if (s != canOK)
        {
            (void)canClose(h);
            m_handle = -1;
            return FromCanlibStatus(s);
        }

        s = canBusOn(h);
        if (s != canOK)
        {
            (void)canClose(h);
            m_handle = -1;
            return FromCanlibStatus(s);
        }

        m_onBus = true;
        return subiediag::eStatus::Ok;
    }

    subiediag::eStatus KvaserCanBus::Close()
    {
        if (m_handle < 0)
        {
            return subiediag::eStatus::Ok;
        }
        if (m_onBus)
        {
            (void)canBusOff(static_cast<canHandle>(m_handle));
            m_onBus = false;
        }
        const canStatus s = canClose(static_cast<canHandle>(m_handle));
        m_handle          = -1;
        return FromCanlibStatus(s);
    }

    subiediag::eStatus KvaserCanBus::SetRxFilter(const uint32_t * /*ids*/, size_t /*count*/)
    {
        // Soft-only filtering. IsoTp drops frames whose ID doesn't match its
        // respId. Hardware filtering on Kvaser stays disabled.
        return subiediag::eStatus::Ok;
    }

    subiediag::eStatus KvaserCanBus::Send(const subiediag::tCanFrame &frame, uint32_t timeoutMs)
    {
        if (!IsOpen())
        {
            return subiediag::eStatus::NotOpen;
        }
        const unsigned int flags = frame.extended ? canMSG_EXT : canMSG_STD;
        const canStatus    s     = canWriteWait(static_cast<canHandle>(m_handle),
                                         static_cast<long>(frame.id),
                                         const_cast<uint8_t *>(frame.data),
                                         frame.dlc,
                                         flags,
                                         timeoutMs);
        return FromCanlibStatus(s);
    }

    subiediag::eStatus KvaserCanBus::Receive(subiediag::tCanFrame *out, uint32_t timeoutMs)
    {
        if (out == nullptr)
        {
            return subiediag::eStatus::InvalidFrame;
        }
        if (!IsOpen())
        {
            return subiediag::eStatus::NotOpen;
        }

        long          id      = 0;
        unsigned int  dlc     = 0;
        unsigned int  flags   = 0;
        unsigned long ticks   = 0;
        uint8_t       data[8] = {};

        const canStatus s     = canReadWait(static_cast<canHandle>(m_handle), &id, data, &dlc, &flags, &ticks, timeoutMs);
        if (s != canOK)
        {
            return FromCanlibStatus(s);
        }

        out->id       = static_cast<uint32_t>(id);
        out->dlc      = static_cast<uint8_t>(dlc > subiediag::c_canMaxDataLen ? subiediag::c_canMaxDataLen : dlc);
        out->extended = (flags & canMSG_EXT) != 0;
        // CANLIB's default time resolution is 10us per tick. Multiply to land
        // in microseconds. Not calibrated against the wall clock -- use as a
        // monotonic timestamp only.
        out->timestampUs = static_cast<uint64_t>(ticks) * 10ULL;
        memcpy(out->data, data, out->dlc);
        return subiediag::eStatus::Ok;
    }

    size_t KvaserCanBus::ListChannels(tChannelInfo *out, size_t maxChannels) noexcept
    {
        EnsureCanlibInit();

        int count = 0;
        if (canGetNumberOfChannels(&count) != canOK || count < 0)
        {
            return 0;
        }
        const size_t total = static_cast<size_t>(count);
        if (out == nullptr || maxChannels == 0)
        {
            return total;
        }

        const size_t write = total < maxChannels ? total : maxChannels;
        for (size_t i = 0; i < write; ++i)
        {
            char            buf[256] = {};
            const canStatus s        = canGetChannelData(static_cast<int>(i), canCHANNELDATA_DEVDESCR_ASCII, buf, sizeof(buf));
            if (s != canOK)
            {
                buf[0] = '\0';
            }
            out[i].channel      = static_cast<int>(i);
            const size_t srcLen = strlen(buf);
            const size_t take   = srcLen < sizeof(out[i].name) - 1 ? srcLen : sizeof(out[i].name) - 1;
            memcpy(out[i].name, buf, take);
            out[i].name[take] = '\0';
        }
        return total;
    }

}  // namespace subiediag::backends
