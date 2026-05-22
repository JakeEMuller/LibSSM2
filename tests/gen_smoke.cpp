// Smoke test for the generated SSM2 base parameter table.
//
// Purpose: confirm the codegen + include wiring is correct before any of the
// real library code exists. Compiles, runs, prints a few entries. Will be
// replaced by proper unit tests once the library is scaffolded.

#include "libssm2/SsmBaseTable.h"

#include <cstdio>
#include <string_view>

namespace {

const char* storage_name(libssm2::StorageType s) noexcept {
    using S = libssm2::StorageType;
    switch (s) {
        case S::Uint8:  return "u8";
        case S::Uint16: return "u16";
        case S::Uint32: return "u32";
        case S::Int8:   return "i8";
        case S::Int16:  return "i16";
        case S::Int32:  return "i32";
        case S::Float:  return "f32";
        case S::Unknown:return "?";
    }
    return "?";
}

}  // namespace

int main() {
    using namespace libssm2;

    static_assert(kSsmBaseTable.size() == 156, "ssmbase parameter count changed");
    static_assert(kSsmBaseTable[0].cap.gated(), "first entry must be flag-gated");
    static_assert(kSsmBaseTable[0].offset == 0x000E, "first entry should be Engine Speed");

    std::printf("libssm2 generated-table smoke test\n");
    std::printf("  entries:   %zu\n", kSsmBaseTable.size());

    int gated = 0;
    for (const auto& p : kSsmBaseTable) {
        if (p.cap.gated()) ++gated;
    }
    std::printf("  gated:     %d\n", gated);

    std::printf("\n  first 5 entries:\n");
    for (std::size_t i = 0; i < 5 && i < kSsmBaseTable.size(); ++i) {
        const auto& p = kSsmBaseTable[i];
        std::printf("    %-30.*s  0x%04X  cap[%u.%u]  %-3s  %.*s\n",
                    static_cast<int>(p.name.size()),   p.name.data(),
                    p.offset,
                    static_cast<unsigned>(p.cap.byte), static_cast<unsigned>(p.cap.bit),
                    storage_name(p.storage),
                    static_cast<int>(p.metric.size()), p.metric.data());
    }
    return 0;
}
