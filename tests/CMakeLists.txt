cmake_minimum_required(VERSION 3.20)

project(AirQualitySensorsTests LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(air_quality_tests
    test_main.c
    ../Core/Src/bsec_state_record.c
    ../Core/Src/measurement_format.c
    ../Core/Src/measurement_view.c
    ../Core/Src/protocol.c
    ../Core/Src/task_health.c
    ../libs/circbuff/circbuff.c
    ../libs/hpma/hpma_parser.c
    # The same formatter the firmware links, so a passing float-formatting
    # test proves what the target actually emits.
    ../libs/newlib_fixes/printf.c)

target_compile_definitions(air_quality_tests PRIVATE CIRCBUFF_HOST_TEST)
target_include_directories(air_quality_tests PRIVATE
    ../Core/Inc
    ../libs)

if(MSVC)
    target_compile_options(air_quality_tests PRIVATE /W4 /WX)
    # The vendored tiny printf is third-party code and is not warning-clean.
    set_source_files_properties(../libs/newlib_fixes/printf.c PROPERTIES
        COMPILE_OPTIONS "/W0")
else()
    target_compile_options(air_quality_tests PRIVATE
        -Wall -Wextra -Werror -Wformat=2 -Wshadow -Wundef)
    set_source_files_properties(../libs/newlib_fixes/printf.c PROPERTIES
        COMPILE_OPTIONS "-w")
endif()

add_test(NAME air_quality_tests COMMAND air_quality_tests)
