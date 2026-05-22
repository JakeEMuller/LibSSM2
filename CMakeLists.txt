cmake_minimum_required(VERSION 3.20)

project(libssm2
    VERSION   0.1.0
    LANGUAGES CXX
    DESCRIPTION "SSM2-over-CAN diagnostic library for Subaru ECUs"
)

# Standalone vs. add_subdirectory(): only set globals at the top level.
if (PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# ----------------------------------------------------------------------------
# Codegen (Python -> SsmBaseTable.h). Lives in data/ so the Python pipeline
# stays out of the way of the C++ library targets.
#
# Exports: libssm2_codegen target, libssm2::generated INTERFACE library.
# ----------------------------------------------------------------------------
add_subdirectory(data)

# ----------------------------------------------------------------------------
# C++ library targets go below. (Real library sources will be added here when
# include/libssm2/Ssm2.h, IsoTp.h, Can.h and their .cpp files exist.)
# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# Build-time validation: compiles the generated header in isolation so the
# codegen + include wiring is end-to-end testable before the rest of the
# library exists.
# ----------------------------------------------------------------------------
option(LIBSSM2_BUILD_GEN_SMOKE_TEST
    "Build a tiny executable that links against the generated table"
    ${PROJECT_IS_TOP_LEVEL})

if (LIBSSM2_BUILD_GEN_SMOKE_TEST)
    add_executable(libssm2_gen_smoke tests/gen_smoke.cpp)
    target_link_libraries(libssm2_gen_smoke PRIVATE libssm2::generated)
endif()
