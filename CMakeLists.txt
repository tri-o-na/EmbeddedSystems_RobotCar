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
# Pico W board (set this for builds that use WiFi)
set(PICO_BOARD pico_w)

# Pull in the Pico SDK
include(pico_sdk_import.cmake)

project(week6demo_all C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK (must be after project())
pico_sdk_init()

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

# =========================
# ---- Other4 demos ----
# =========================

# Motor encoder
add_executable(motor_encoder
    week6demo/other4/src/motor_encoder.c
)
pico_enable_stdio_usb(motor_encoder 1)
pico_enable_stdio_uart(motor_encoder 0)
target_link_libraries(motor_encoder
    pico_stdlib
    hardware_pwm
    hardware_timer
    hardware_irq
)
pico_add_extra_outputs(motor_encoder)

# IR line sensor
add_executable(ir_line_sensor
    week6demo/other4/src/ir_line_sensor.c
)
pico_enable_stdio_usb(ir_line_sensor 1)
pico_enable_stdio_uart(ir_line_sensor 0)
target_link_libraries(ir_line_sensor
    pico_stdlib
    hardware_adc
    hardware_timer
)
pico_add_extra_outputs(ir_line_sensor)

# Accelerometer (MPU6050 over I2C)
add_executable(accel_raw_mpu6050
    week6demo/other4/src/accel_raw_mpu6050.c
)
pico_enable_stdio_usb(accel_raw_mpu6050 1)
pico_enable_stdio_uart(accel_raw_mpu6050 0)
target_link_libraries(accel_raw_mpu6050
    pico_stdlib
    hardware_i2c
)
pico_add_extra_outputs(accel_raw_mpu6050)

# Ultrasonic sensor
add_executable(ultrasonic_hcsr04
    week6demo/other4/src/ultrasonic_hcsr04.c
)
pico_enable_stdio_usb(ultrasonic_hcsr04 1)
pico_enable_stdio_uart(ultrasonic_hcsr04 0)
target_link_libraries(ultrasonic_hcsr04
    pico_stdlib
    hardware_timer
)
pico_add_extra_outputs(ultrasonic_hcsr04)



# --- lab ultrasonic ---
add_executable(ultrasonic
    week6demo/ultrasonic/main.c
    week6demo/ultrasonic/ultrasonic.c
    week6demo/ultrasonic/ultrasonicObject.cpp   # <-- include if you actually use it
)

target_include_directories(ultrasonic PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/week6demo/ultrasonic
)

pico_enable_stdio_usb(ultrasonic 1)
pico_enable_stdio_uart(ultrasonic 0)

target_link_libraries(ultrasonic
    pico_stdlib
    hardware_gpio
    hardware_timer
)

pico_add_extra_outputs(ultrasonic)

# IMU test (MPU6050)
add_executable(imu_test
    week6demo/imu/src/imu_test.c
    week6demo/imu/src/imu.c
)
target_include_directories(imu_test PRIVATE week6demo/imu/include)
target_link_libraries(imu_test pico_stdlib hardware_i2c)
pico_enable_stdio_usb(imu_test 1)
pico_enable_stdio_uart(imu_test 0)
pico_add_extra_outputs(imu_test)

# I2C Scanner
add_executable(i2c_scanner
    week6demo/imu/src/i2c_scanner.c
)
target_include_directories(i2c_scanner PRIVATE week6demo/imu/include)
target_link_libraries(i2c_scanner pico_stdlib hardware_i2c)
pico_enable_stdio_usb(i2c_scanner 1)
pico_enable_stdio_uart(i2c_scanner 0)
pico_add_extra_outputs(i2c_scanner)



# =========================
# ---- Integration Demo ----
# =========================
# Integration Demo (LIS3DH)
add_executable(Integration
    week6demo/Integration/main.c
    week6demo/Integration/imu_lis3dh.c
    week6demo/Integration/motor_encoder.c
    week6demo/Integration/pid.c
)

target_include_directories(Integration PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/week6demo/Integration
)

target_link_libraries(Integration
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_irq
    hardware_timer
)

pico_enable_stdio_usb(Integration 1)
pico_enable_stdio_uart(Integration 0)
pico_add_extra_outputs(Integration)



