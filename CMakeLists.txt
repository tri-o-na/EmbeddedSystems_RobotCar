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
add_subdirectory(week6demo/WiFi)

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


# =========================
# ---- WiFi demo ----
# =========================
add_executable(pico_w_udp_byte
    week6demo/WiFi/src/main.c
)
target_include_directories(pico_w_udp_byte PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/week6demo/WiFi/include
)
pico_enable_stdio_usb(pico_w_udp_byte 1)
pico_enable_stdio_uart(pico_w_udp_byte 0)
target_link_libraries(pico_w_udp_byte
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)
pico_add_extra_outputs(pico_w_udp_byte)