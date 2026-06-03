// kvaser_hello.cpp -- read-only hardware smoke test for the Kvaser backend.
//
// Opens a Kvaser CAN channel, connects to the engine ECU on 0x7E0/0x7E8,
// runs an SSM2 init, and prints the SSM ID + ROM ID + count of supported
// cap-flag bits.
//
// Usage: subiediag_kvaser_hello [channel=0]
//
// SAFETY: this program is read-only by construction. It never calls any
// Write* method or UnlockWrites(). The library default-locked state would
// reject writes anyway -- this is belt-and-suspenders.

#include "subiediag/Common.h"
#include "subiediag/Ssm2.h"
#include "subiediag/backends/Kvaser.h"

#include <cstdio>
#include <cstdlib>

int main(int argc, char **argv)
{
    using subiediag::can::c_engineReqId;
    using subiediag::can::c_engineRespId;
    using subiediag::can::KvaserCanBus;
    using subiediag::can::tChannelInfo;
    using subiediag::isotp::IsoTpTransport;
    using subiediag::ssm2::c_capFlagsLen;
    using subiediag::ssm2::Ssm2Client;
    using subiediag::ssm2::tInitResponse;

    int channel = 0;
    if (argc >= 2)
    {
        channel = std::atoi(argv[1]);
    }

    // Enumerate channels for context.
    tChannelInfo channels[8] = {};
    const size_t total       = KvaserCanBus::ListChannels(channels, 8);
    std::printf("CANLIB reports %zu channel(s):\n", total);
    for (size_t i = 0; i < total && i < 8; ++i)
    {
        std::printf("  [%d] %s\n", channels[i].channel, channels[i].name);
    }
    if (total == 0)
    {
        std::printf("(no channels found; is the Kvaser device plugged in?)\n");
        return 1;
    }

    std::printf("\nOpening channel %d at 500 kbps...\n", channel);

    KvaserCanBus::tConfig busCfg;
    busCfg.channel = channel;
    KvaserCanBus bus(busCfg);

    IsoTpTransport transport(&bus, c_engineReqId, c_engineRespId);

    Ssm2Client::tConfig cfg;
    cfg.transport = &transport;
    Ssm2Client client(cfg);

    tInitResponse            init{};
    const subiediag::eStatus s = client.Connect(&init);
    if (!subiediag::IsOk(s))
    {
        const auto desc = subiediag::DescribeStatus(s);
        std::printf("Connect failed: %.*s\n", static_cast<int>(desc.size()), desc.data());
        return 2;
    }

    std::printf("\nSSM ID: %02X %02X %02X\n", init.ssmId[0], init.ssmId[1], init.ssmId[2]);
    std::printf("ROM ID: %02X %02X %02X %02X %02X\n", init.romId[0], init.romId[1], init.romId[2], init.romId[3], init.romId[4]);

    int supported = 0;
    for (size_t b = 0; b < c_capFlagsLen; ++b)
    {
        uint8_t v = init.capFlags[b];
        while (v != 0)
        {
            supported += v & 1;
            v >>= 1;
        }
    }
    std::printf("Cap-flag bits set: %d (out of 768 possible)\n", supported);

    return 0;
}
