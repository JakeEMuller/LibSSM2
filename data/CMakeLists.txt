# ----------------------------------------------------------------------------
# data/CMakeLists.txt
#
# Code generation for libssm2: parses log_defs.xml (RomRaider's SSM2 parameter
# definitions) into a constexpr C++ table via the Python script in this folder.
#
# Exports:
#   libssm2_codegen    - custom target; runs gen_params.py
#   libssm2::generated - INTERFACE library; consumers link against it to get
#                        the include path and a dependency on the codegen
# ----------------------------------------------------------------------------

find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)

set(LIBSSM2_LOG_DEFS_XML  "${CMAKE_CURRENT_SOURCE_DIR}/log_defs.xml")
set(LIBSSM2_GEN_SCRIPT    "${CMAKE_CURRENT_SOURCE_DIR}/gen_params.py")

# Output mirrors the eventual include/libssm2/ layout so generated and
# hand-written headers share the same include prefix.
set(LIBSSM2_GEN_ROOT      "${CMAKE_CURRENT_BINARY_DIR}/gen")
set(LIBSSM2_GEN_HEADER    "${LIBSSM2_GEN_ROOT}/libssm2/SsmBaseTable.h")

file(MAKE_DIRECTORY "${LIBSSM2_GEN_ROOT}/libssm2")

add_custom_command(
    OUTPUT  "${LIBSSM2_GEN_HEADER}"
    COMMAND "${Python3_EXECUTABLE}"
            "${LIBSSM2_GEN_SCRIPT}"
            "${LIBSSM2_LOG_DEFS_XML}"
            --emit-cpp
            --out "${LIBSSM2_GEN_HEADER}"
    DEPENDS "${LIBSSM2_LOG_DEFS_XML}"
            "${LIBSSM2_GEN_SCRIPT}"
    VERBATIM
    COMMENT "Generating libssm2/SsmBaseTable.h from log_defs.xml"
)

# Standalone target so the codegen can be triggered without building
# anything else:  cmake --build <build> --target libssm2_codegen
add_custom_target(libssm2_codegen
    DEPENDS "${LIBSSM2_GEN_HEADER}"
    SOURCES "${LIBSSM2_LOG_DEFS_XML}" "${LIBSSM2_GEN_SCRIPT}"
)

# Header-only INTERFACE library that exposes the generated table.
# Consumers:  target_link_libraries(my_target PRIVATE libssm2::generated)
add_library(libssm2_generated INTERFACE)
add_library(libssm2::generated ALIAS libssm2_generated)

target_include_directories(libssm2_generated INTERFACE
    "$<BUILD_INTERFACE:${LIBSSM2_GEN_ROOT}>"
)
target_sources(libssm2_generated INTERFACE
    "$<BUILD_INTERFACE:${LIBSSM2_GEN_HEADER}>"
)
target_compile_features(libssm2_generated INTERFACE cxx_std_17)
add_dependencies(libssm2_generated libssm2_codegen)
