cmake_minimum_required(VERSION 3.20)

project(AirQualitySensorsTests LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(air_quality_tests
    test_main.c
    ../Core/Src/measurement_format.c
    ../Core/Src/protocol.c
    ../libs/circbuff/circbuff.c
    ../libs/hpma/hpma_parser.c)

target_compile_definitions(air_quality_tests PRIVATE CIRCBUFF_HOST_TEST)
target_include_directories(air_quality_tests PRIVATE
    ../Core/Inc
    ../libs)

if(MSVC)
    target_compile_options(air_quality_tests PRIVATE /W4 /WX)
else()
    target_compile_options(air_quality_tests PRIVATE
        -Wall -Wextra -Werror -Wformat=2 -Wshadow -Wundef)
endif()

add_test(NAME air_quality_tests COMMAND air_quality_tests)
