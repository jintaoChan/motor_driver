cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME motor_driver)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# SOES integration (external source tree)
set(SOES_DIR "C:/Users/10100/Documents/Projects/SOES/soes")
set(SOES_HAL_DIR "${SOES_DIR}/hal/stm32g474-lan9252")

# SOES expects these variables to be defined by the parent project.
set(HAL_SOURCES
    ${SOES_HAL_DIR}/esc_hw.c
)
set(HAL_INCLUDES
    ${SOES_DIR}
    ${SOES_HAL_DIR}
    ${SOES_DIR}/include/sys/gcc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/ethercat
)

add_subdirectory(${SOES_DIR} ${CMAKE_BINARY_DIR}/soes)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Src/ethercat/soes_lan9252_port.c
    Src/ethercat/slave_objectlist.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${SOES_DIR}
    ${SOES_HAL_DIR}
    ${SOES_DIR}/include/sys/gcc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/ethercat
)

# Supplement the stm32cubemx interface library with full HAL/LL and MCSDK headers
# from the STM32Cube repository. CubeMX regeneration produces incomplete copies of
# both Drivers and MCSDK; adding the repository paths here ensures all sub-targets
# (MotorControl, STM32_Drivers) always resolve missing headers without needing to
# manually copy files after every regeneration.
target_include_directories(stm32cubemx INTERFACE
    D:/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.2/Drivers/STM32G4xx_HAL_Driver/Inc
    D:/STM32Cube/Repository/MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Inc
    D:/STM32Cube/Repository/MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/G4xx/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    soes

    # Add user defined libraries
)
