# ----------------------------------------------------------------------------
# data/CMakeLists.txt
#
# Code generation for subiediag. Two pipelines today:
#   1. log_defs.xml          -> SsmBaseTable.h    (gen_params.py)
#   2. obd2_generic_dtcs.csv -> Obd2DtcDb.h       (gen_obd2_dtcs.py)
#
# Exports:
#   subiediag_codegen    - custom target; runs all generators
#   subiediag::generated - INTERFACE library; consumers link against it to get
#                          the include path and a dependency on the codegen
# ----------------------------------------------------------------------------

find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)

# Output mirrors the eventual include/subiediag/ layout so generated and
# hand-written headers share the same include prefix.
set(SUBIEDIAG_GEN_ROOT "${CMAKE_CURRENT_BINARY_DIR}/gen")
file(MAKE_DIRECTORY "${SUBIEDIAG_GEN_ROOT}/subiediag")

# --- SSM2 parameter table (log_defs.xml -> SsmBaseTable.h) ------------------

set(SUBIEDIAG_LOG_DEFS_XML "${CMAKE_CURRENT_SOURCE_DIR}/log_defs.xml")
set(SUBIEDIAG_PARAMS_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/gen_params.py")
set(SUBIEDIAG_SSM_HEADER   "${SUBIEDIAG_GEN_ROOT}/subiediag/SsmBaseTable.h")

add_custom_command(
    OUTPUT  "${SUBIEDIAG_SSM_HEADER}"
    COMMAND "${Python3_EXECUTABLE}"
            "${SUBIEDIAG_PARAMS_SCRIPT}"
            "${SUBIEDIAG_LOG_DEFS_XML}"
            --emit-cpp
            --out "${SUBIEDIAG_SSM_HEADER}"
    DEPENDS "${SUBIEDIAG_LOG_DEFS_XML}"
            "${SUBIEDIAG_PARAMS_SCRIPT}"
    VERBATIM
    COMMENT "Generating subiediag/SsmBaseTable.h from log_defs.xml"
)

# --- OBD-II generic DTC database (obd2_generic_dtcs.csv -> Obd2DtcDb.h) -----

set(SUBIEDIAG_DTC_CSV    "${CMAKE_CURRENT_SOURCE_DIR}/obd2_generic_dtcs.csv")
set(SUBIEDIAG_DTC_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/gen_obd2_dtcs.py")
set(SUBIEDIAG_DTC_HEADER "${SUBIEDIAG_GEN_ROOT}/subiediag/Obd2DtcDb.h")

add_custom_command(
    OUTPUT  "${SUBIEDIAG_DTC_HEADER}"
    COMMAND "${Python3_EXECUTABLE}"
            "${SUBIEDIAG_DTC_SCRIPT}"
            "${SUBIEDIAG_DTC_CSV}"
            --out "${SUBIEDIAG_DTC_HEADER}"
    DEPENDS "${SUBIEDIAG_DTC_CSV}"
            "${SUBIEDIAG_DTC_SCRIPT}"
    VERBATIM
    COMMENT "Generating subiediag/Obd2DtcDb.h from obd2_generic_dtcs.csv"
)

# Standalone target so the codegen can be triggered without building
# anything else:  cmake --build <build> --target subiediag_codegen
add_custom_target(subiediag_codegen
    DEPENDS "${SUBIEDIAG_SSM_HEADER}"
            "${SUBIEDIAG_DTC_HEADER}"
    SOURCES "${SUBIEDIAG_LOG_DEFS_XML}" "${SUBIEDIAG_PARAMS_SCRIPT}"
            "${SUBIEDIAG_DTC_CSV}" "${SUBIEDIAG_DTC_SCRIPT}"
)

# Header-only INTERFACE library that exposes the generated tables.
# Consumers:  target_link_libraries(my_target PRIVATE subiediag::generated)
add_library(subiediag_generated INTERFACE)
add_library(subiediag::generated ALIAS subiediag_generated)

target_include_directories(subiediag_generated INTERFACE
    "$<BUILD_INTERFACE:${SUBIEDIAG_GEN_ROOT}>"
)
target_sources(subiediag_generated INTERFACE
    "$<BUILD_INTERFACE:${SUBIEDIAG_SSM_HEADER}>"
    "$<BUILD_INTERFACE:${SUBIEDIAG_DTC_HEADER}>"
)
target_compile_features(subiediag_generated INTERFACE cxx_std_17)
add_dependencies(subiediag_generated subiediag_codegen)
