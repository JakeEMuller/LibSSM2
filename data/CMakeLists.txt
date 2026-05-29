# ----------------------------------------------------------------------------
# data/CMakeLists.txt
#
# Code generation for subiediag: parses log_defs.xml (RomRaider's SSM2 parameter
# definitions) into a constexpr C++ table via the Python script in this folder.
#
# Exports:
#   subiediag_codegen    - custom target; runs gen_params.py
#   subiediag::generated - INTERFACE library; consumers link against it to get
#                        the include path and a dependency on the codegen
# ----------------------------------------------------------------------------

find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)

set(SUBIEDIAG_LOG_DEFS_XML  "${CMAKE_CURRENT_SOURCE_DIR}/log_defs.xml")
set(SUBIEDIAG_GEN_SCRIPT    "${CMAKE_CURRENT_SOURCE_DIR}/gen_params.py")

# Output mirrors the eventual include/subiediag/ layout so generated and
# hand-written headers share the same include prefix.
set(SUBIEDIAG_GEN_ROOT      "${CMAKE_CURRENT_BINARY_DIR}/gen")
set(SUBIEDIAG_GEN_HEADER    "${SUBIEDIAG_GEN_ROOT}/subiediag/SsmBaseTable.h")

file(MAKE_DIRECTORY "${SUBIEDIAG_GEN_ROOT}/subiediag")

add_custom_command(
    OUTPUT  "${SUBIEDIAG_GEN_HEADER}"
    COMMAND "${Python3_EXECUTABLE}"
            "${SUBIEDIAG_GEN_SCRIPT}"
            "${SUBIEDIAG_LOG_DEFS_XML}"
            --emit-cpp
            --out "${SUBIEDIAG_GEN_HEADER}"
    DEPENDS "${SUBIEDIAG_LOG_DEFS_XML}"
            "${SUBIEDIAG_GEN_SCRIPT}"
    VERBATIM
    COMMENT "Generating subiediag/SsmBaseTable.h from log_defs.xml"
)

# Standalone target so the codegen can be triggered without building
# anything else:  cmake --build <build> --target subiediag_codegen
add_custom_target(subiediag_codegen
    DEPENDS "${SUBIEDIAG_GEN_HEADER}"
    SOURCES "${SUBIEDIAG_LOG_DEFS_XML}" "${SUBIEDIAG_GEN_SCRIPT}"
)

# Header-only INTERFACE library that exposes the generated table.
# Consumers:  target_link_libraries(my_target PRIVATE subiediag::generated)
add_library(subiediag_generated INTERFACE)
add_library(subiediag::generated ALIAS subiediag_generated)

target_include_directories(subiediag_generated INTERFACE
    "$<BUILD_INTERFACE:${SUBIEDIAG_GEN_ROOT}>"
)
target_sources(subiediag_generated INTERFACE
    "$<BUILD_INTERFACE:${SUBIEDIAG_GEN_HEADER}>"
)
target_compile_features(subiediag_generated INTERFACE cxx_std_17)
add_dependencies(subiediag_generated subiediag_codegen)
