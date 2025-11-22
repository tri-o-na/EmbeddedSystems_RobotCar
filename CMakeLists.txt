# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(RobotCar C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

# Initialise the Pico SDK
pico_sdk_init()

# Add include/ directory for headers
include_directories(include)

# Add src directory
add_subdirectory(src)

# Main executable
add_executable(RobotCar_Integration
    main.c
    src/imu_lis3dh.c
    src/motor_encoder.c
    src/pid.c
    src/line_follow.c
    src/barcode.c
    src/servo.c
    src/ultrasonic.c
)

# Add include paths for src/
target_include_directories(RobotCar_Integration PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

# Link Pico SDK libraries
target_link_libraries(RobotCar_Integration
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_irq
    hardware_timer
    hardware_adc
)

pico_enable_stdio_usb(RobotCar_Integration 1)
pico_enable_stdio_uart(RobotCar_Integration 0)
pico_add_extra_outputs(RobotCar_Integration)
