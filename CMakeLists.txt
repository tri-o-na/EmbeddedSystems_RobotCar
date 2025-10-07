# set(PICO_BOARD pico_w)
# cmake_minimum_required(VERSION 3.12)

# # Pull in SDK (must be before project)
# include(pico_sdk_import.cmake)

# include(pico_extras_import_optional.cmake)

# project(pico_examples C CXX ASM)
# set(CMAKE_C_STANDARD 11)
# set(CMAKE_CXX_STANDARD 17)

# if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
#     message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
# endif()

# set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# # Initialize the SDK
# pico_sdk_init()

# include(example_auto_set_url.cmake)
# # Add blink example
# add_subdirectory(blink)

# # Add hello world example
# add_subdirectory(hello_world)

# add_compile_options(-Wall
#         -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
#         -Wno-unused-function # we have some for the docs that aren't called
#         )
# if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
#     add_compile_options(-Wno-maybe-uninitialized)
# endif()

# # Hardware-specific examples in subdirectories:
# # add_subdirectory(adc)
# # add_subdirectory(clocks)
# # add_subdirectory(cmake)
# # add_subdirectory(divider)
# # add_subdirectory(dma)
# # add_subdirectory(flash)
# # add_subdirectory(gpio)
# # add_subdirectory(i2c)
# # add_subdirectory(interp)
# # add_subdirectory(multicore)
# # add_subdirectory(picoboard)
# # add_subdirectory(pico_w)
# # add_subdirectory(pio)
# # add_subdirectory(pwm)
# # add_subdirectory(reset)
# # add_subdirectory(rtc)
# # add_subdirectory(spi)
# # add_subdirectory(system)
# # add_subdirectory(timer)
# # add_subdirectory(uart)
# # add_subdirectory(usb)
# # add_subdirectory(watchdog)
# # add_subdirectory(my_button_led)
# add_subdirectory(week6demo)



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
