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
# Core C++ library.
#
# Declarations in include/libssm2/{Common,Can,IsoTp,Ssm2}.h, implementations
# in src/. Backend implementations (Kvaser, SocketCAN) live in separate
# libraries and link against libssm2::core.
# ----------------------------------------------------------------------------
add_library(libssm2_core STATIC
    src/IsoTp.cpp
)
add_library(libssm2::core ALIAS libssm2_core)

target_include_directories(libssm2_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_compile_features(libssm2_core PUBLIC cxx_std_17)

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
    target_link_libraries(libssm2_gen_smoke PRIVATE
        libssm2::core
        libssm2::generated
    )
endif()

# ----------------------------------------------------------------------------
# Unit tests.
# ----------------------------------------------------------------------------
option(LIBSSM2_BUILD_TESTS
    "Build unit tests for libssm2"
    ${PROJECT_IS_TOP_LEVEL})

if (LIBSSM2_BUILD_TESTS)
    enable_testing()

    add_executable(libssm2_test_isotp tests/test_isotp.cpp)
    target_link_libraries(libssm2_test_isotp PRIVATE libssm2::core)
    target_include_directories(libssm2_test_isotp PRIVATE tests)
    add_test(NAME isotp COMMAND libssm2_test_isotp)
endif()
