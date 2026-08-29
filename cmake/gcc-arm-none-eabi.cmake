set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_TOOLCHAIN_PATH "" CACHE PATH
    "Directory containing the arm-none-eabi compiler executables")

set(_arm_toolchain_hints)
if(ARM_TOOLCHAIN_PATH)
    list(APPEND _arm_toolchain_hints "${ARM_TOOLCHAIN_PATH}")
endif()
if(DEFINED ENV{ARM_TOOLCHAIN_PATH})
    list(APPEND _arm_toolchain_hints "$ENV{ARM_TOOLCHAIN_PATH}")
endif()

if(WIN32)
    file(GLOB _stm32cubeide_toolchains LIST_DIRECTORIES true
        "C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin")
    list(SORT _stm32cubeide_toolchains COMPARE NATURAL ORDER DESCENDING)
    list(APPEND _arm_toolchain_hints ${_stm32cubeide_toolchains})
elseif(APPLE)
    file(GLOB _stm32cubeide_toolchains LIST_DIRECTORIES true
        "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin")
    list(APPEND _arm_toolchain_hints ${_stm32cubeide_toolchains})
else()
    file(GLOB _stm32cubeide_toolchains LIST_DIRECTORIES true
        "/opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin")
    list(APPEND _arm_toolchain_hints ${_stm32cubeide_toolchains})
endif()

find_program(CMAKE_C_COMPILER
    NAMES arm-none-eabi-gcc
    HINTS ${_arm_toolchain_hints}
    REQUIRED)
find_program(CMAKE_ASM_COMPILER
    NAMES arm-none-eabi-gcc
    HINTS ${_arm_toolchain_hints}
    REQUIRED)
find_program(CMAKE_AR
    NAMES arm-none-eabi-ar
    HINTS ${_arm_toolchain_hints}
    REQUIRED)
find_program(CMAKE_OBJCOPY
    NAMES arm-none-eabi-objcopy
    HINTS ${_arm_toolchain_hints}
    REQUIRED)
find_program(CMAKE_SIZE
    NAMES arm-none-eabi-size
    HINTS ${_arm_toolchain_hints}
    REQUIRED)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")
