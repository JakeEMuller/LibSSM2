cmake_minimum_required(VERSION 3.20)

project(subiediag
    VERSION   0.1.0
    LANGUAGES CXX
    DESCRIPTION "SSM2-over-CAN diagnostic library for Subaru ECUs"
)

# Standalone vs. add_subdirectory(): only set globals at the top level.
if (PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    # MinGW dynamically links libgcc_s, libstdc++, libwinpthread by default.
    # Those DLLs only exist on MSYS2-equipped machines. Statically link them
    # so executables built here can be copied to a fresh Windows box and run
    # standalone. (canlib32.dll stays external -- it's closed-source 3rd party.)
    if (MINGW)
        add_link_options(
            -static-libgcc
            -static-libstdc++
            -static            # also includes libwinpthread (std::call_once / std::mutex)
        )
    endif()
endif()

# ----------------------------------------------------------------------------
# Codegen (Python -> SsmBaseTable.h). Lives in data/ so the Python pipeline
# stays out of the way of the C++ library targets.
#
# Exports: subiediag_codegen target, subiediag::generated INTERFACE library.
# ----------------------------------------------------------------------------
add_subdirectory(data)

# ----------------------------------------------------------------------------
# Core C++ library.
#
# Declarations in include/subiediag/{Common,Can,IsoTp,Ssm2}.h, implementations
# in src/. Backend implementations (Kvaser, SocketCAN) live in separate
# libraries and link against subiediag::core.
# ----------------------------------------------------------------------------
add_library(subiediag_core STATIC
    src/IsoTp.cpp
    src/Ssm2Client.cpp
)
add_library(subiediag::core ALIAS subiediag_core)

target_include_directories(subiediag_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_compile_features(subiediag_core PUBLIC cxx_std_17)

# ----------------------------------------------------------------------------
# Kvaser CANLIB backend (Windows-only). Auto-detects the SDK at the default
# install path; can be overridden via -DSUBIEDIAG_KVASER_SDK_DIR=...
# ----------------------------------------------------------------------------
set(SUBIEDIAG_KVASER_SDK_DIR "C:/Program Files (x86)/Kvaser/Canlib"
    CACHE PATH "Path to Kvaser CANLIB SDK")

find_path(KVASER_INCLUDE_DIR
    NAMES canlib.h
    HINTS "${SUBIEDIAG_KVASER_SDK_DIR}/INC"
    NO_DEFAULT_PATH
)
find_library(KVASER_CANLIB_LIB
    NAMES canlib32
    HINTS "${SUBIEDIAG_KVASER_SDK_DIR}/Lib/x64"
    NO_DEFAULT_PATH
)
find_file(KVASER_CANLIB_DLL
    NAMES canlib32.dll
    HINTS "${SUBIEDIAG_KVASER_SDK_DIR}/bin_x64"
          "${SUBIEDIAG_KVASER_SDK_DIR}/Bin"
    NO_DEFAULT_PATH
)

if (KVASER_INCLUDE_DIR AND KVASER_CANLIB_LIB)
    set(SUBIEDIAG_KVASER_AVAILABLE TRUE)
else()
    set(SUBIEDIAG_KVASER_AVAILABLE FALSE)
endif()

option(SUBIEDIAG_BACKEND_KVASER
    "Build the Kvaser CANLIB backend"
    ${SUBIEDIAG_KVASER_AVAILABLE})

if (SUBIEDIAG_BACKEND_KVASER)
    if (NOT SUBIEDIAG_KVASER_AVAILABLE)
        message(FATAL_ERROR "SUBIEDIAG_BACKEND_KVASER=ON but CANLIB SDK was not found. "
                            "Set SUBIEDIAG_KVASER_SDK_DIR or install the SDK.")
    endif()

    add_library(subiediag_kvaser STATIC
        src/backends/Kvaser.cpp
    )
    add_library(subiediag::kvaser ALIAS subiediag_kvaser)

    target_link_libraries(subiediag_kvaser
        PUBLIC  subiediag::core
        PRIVATE "${KVASER_CANLIB_LIB}"
    )
    target_include_directories(subiediag_kvaser PRIVATE
        "${KVASER_INCLUDE_DIR}"
    )

    message(STATUS "subiediag: Kvaser backend enabled (SDK: ${SUBIEDIAG_KVASER_SDK_DIR})")
endif()

# ----------------------------------------------------------------------------
# Optional hardware smoke test (read-only). Off by default since it requires
# a Kvaser device on the bus.
# ----------------------------------------------------------------------------
option(SUBIEDIAG_BUILD_KVASER_HELLO
    "Build the subiediag_kvaser_hello hardware smoke-test executable"
    OFF)

if (SUBIEDIAG_BUILD_KVASER_HELLO)
    if (NOT SUBIEDIAG_BACKEND_KVASER)
        message(FATAL_ERROR "SUBIEDIAG_BUILD_KVASER_HELLO requires SUBIEDIAG_BACKEND_KVASER=ON")
    endif()
    add_executable(subiediag_kvaser_hello tools/kvaser_hello.cpp)
    target_link_libraries(subiediag_kvaser_hello PRIVATE subiediag::core subiediag::kvaser)

    # Stage canlib32.dll next to the exe so it's findable without changing PATH.
    if (KVASER_CANLIB_DLL)
        add_custom_command(TARGET subiediag_kvaser_hello POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${KVASER_CANLIB_DLL}"
                    "$<TARGET_FILE_DIR:subiediag_kvaser_hello>/canlib32.dll"
            VERBATIM
        )
    else()
        message(WARNING "canlib32.dll not found under the SDK -- kvaser_hello.exe "
                        "will fail to run unless you add the DLL to PATH manually.")
    endif()
endif()

# ----------------------------------------------------------------------------
# Build-time validation: compiles the generated header in isolation so the
# codegen + include wiring is end-to-end testable before the rest of the
# library exists.
# ----------------------------------------------------------------------------
option(SUBIEDIAG_BUILD_GEN_SMOKE_TEST
    "Build a tiny executable that links against the generated table"
    ${PROJECT_IS_TOP_LEVEL})

if (SUBIEDIAG_BUILD_GEN_SMOKE_TEST)
    add_executable(subiediag_gen_smoke tests/gen_smoke.cpp)
    target_link_libraries(subiediag_gen_smoke PRIVATE
        subiediag::core
        subiediag::generated
    )
endif()

# ----------------------------------------------------------------------------
# Unit tests.
# ----------------------------------------------------------------------------
option(SUBIEDIAG_BUILD_TESTS
    "Build unit tests for subiediag"
    ${PROJECT_IS_TOP_LEVEL})

if (SUBIEDIAG_BUILD_TESTS)
    enable_testing()

    add_executable(subiediag_test_isotp tests/test_isotp.cpp)
    target_link_libraries(subiediag_test_isotp PRIVATE subiediag::core)
    target_include_directories(subiediag_test_isotp PRIVATE tests)
    add_test(NAME isotp COMMAND subiediag_test_isotp)

    add_executable(subiediag_test_ssm2 tests/test_ssm2.cpp)
    target_link_libraries(subiediag_test_ssm2 PRIVATE subiediag::core)
    target_include_directories(subiediag_test_ssm2 PRIVATE tests)
    add_test(NAME ssm2 COMMAND subiediag_test_ssm2)
endif()
