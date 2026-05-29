// Smoke test for the generated SSM2 base parameter table + core headers.
//
// Purpose: confirm the codegen + include wiring is correct before any of the
// real library code exists. Compiles, runs, prints a few entries. Also
// includes the four core headers to parse-check them; nothing is constructed
// from those types here (their implementations don't exist yet).

#include "subiediag/Can.h"
#include "subiediag/Common.h"
#include "subiediag/IsoTp.h"
#include "subiediag/Ssm2.h"
#include "subiediag/SsmBaseTable.h"

#include <cstdio>

namespace
{

    const char *StorageName(subiediag::eStorageType s) noexcept
    {
        using S = subiediag::eStorageType;
        switch (s)
        {
        case S::Uint8:
            return "u8";
        case S::Uint16:
            return "u16";
        case S::Uint32:
            return "u32";
        case S::Int8:
            return "i8";
        case S::Int16:
            return "i16";
        case S::Int32:
            return "i32";
        case S::Float:
            return "f32";
        case S::Unknown:
            return "?";
        }
        return "?";
    }

}  // namespace

int main()
{
    using namespace subiediag;

    static_assert(c_ssmBaseTable.size() == 156, "ssmbase parameter count changed");
    static_assert(c_ssmBaseTable[0].cap.Gated(), "first entry must be flag-gated");
    static_assert(c_ssmBaseTable[0].offset == 0x000E, "first entry should be Engine Speed");

    // Compile-time touch of core types -- verifies the headers parse and the
    // include directory is wired correctly. No constructors invoked; nothing
    // in the core library has an implementation yet.
    static_assert(IsOk(eStatus::Ok));
    static_assert(!IsOk(eStatus::Timeout));
    static_assert(DescribeStatus(eStatus::Ok) == "ok");
    static_assert(DescribeStatus(eStatus::Timeout) == "timeout");
    static_assert(sizeof(tCanFrame) > 0);
    static_assert(sizeof(tSsm2InitResponse) > 0);
    static_assert(sizeof(Ssm2Client::tConfig) > 0);

    // Protocol constants should match the SSM2 / ISO 15765-4 spec values.
    static_assert(c_canMaxDataLen == 8);
    static_assert(c_ssmIdLen == 3);
    static_assert(c_romIdLen == 5);
    static_assert(c_capFlagsLen == 96);
    static_assert(c_engineReqId == 0x7E0);
    static_assert(c_engineRespId == 0x7E8);
    static_assert(c_transmissionReqId == 0x7E1);
    static_assert(c_transmissionRespId == 0x7E9);
    static_assert(static_cast<uint8_t>(eSsm2Cmd::Init) == 0xAA);
    static_assert(static_cast<uint8_t>(eSsm2Cmd::ReadAddresses) == 0xA8);
    static_assert(static_cast<uint8_t>(eSsm2Cmd::ReadBlock) == 0xA0);
    static_assert(static_cast<uint8_t>(eSsm2Cmd::WriteAddress) == 0xB8);
    static_assert(static_cast<uint8_t>(eSsm2Cmd::WriteBlock) == 0xB0);
    static_assert(static_cast<uint8_t>(eSsm2Rsp::Init) == 0xEA);  // observed init response code

    std::printf("subiediag generated-table smoke test\n");
    std::printf("  entries:   %zu\n", c_ssmBaseTable.size());

    int gated = 0;
    for (const auto &p : c_ssmBaseTable)
    {
        if (p.cap.Gated())
        {
            ++gated;
        }
    }
    std::printf("  gated:     %d\n", gated);

    std::printf("\n  first 5 entries:\n");
    for (size_t i = 0; i < 5 && i < c_ssmBaseTable.size(); ++i)
    {
        const auto &p = c_ssmBaseTable[i];
        std::printf("    %-30.*s  0x%04X  cap[%u.%u]  %-3s  %.*s\n",
                    static_cast<int>(p.name.size()),
                    p.name.data(),
                    p.offset,
                    static_cast<unsigned>(p.cap.byte),
                    static_cast<unsigned>(p.cap.bit),
                    StorageName(p.storage),
                    static_cast<int>(p.metric.size()),
                    p.metric.data());
    }
    return 0;
}
