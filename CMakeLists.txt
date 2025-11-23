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

# Pull in the Pico SDK
include(pico_sdk_import.cmake)

project(RobotCar C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Ensure correct board type
set(PICO_BOARD pico_w CACHE STRING "Board type" FORCE)

# Initialise the SDK
pico_sdk_init()

# lwIP include directories (needed because VSCode sometimes breaks auto-includes)
set(LWIP_INCLUDE_DIRS
    ${PICO_SDK_PATH}/lib/lwip/src/include
    ${PICO_SDK_PATH}/lib/lwip/src/include/lwip
)

# WiFi driver include paths (required for pico/cyw43_arch.h and cyw43.h)
set(CYW43_INCLUDE_DIRS
    ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch/include
    ${PICO_SDK_PATH}/lib/cyw43-driver/src
)

# Add your project source folder
include_directories(include)

# ============================================================
# === RobotCar_Integration Build
# ============================================================

add_executable(RobotCar_Integration
    main.c
    src/imu_lis3dh.c
    src/motor_encoder.c
    src/pid.c
    src/line_follow.c
    src/barcode.c
    src/servo.c
    src/ultrasonic.c
    src/mqtt_robot/src/mqtt_pico.c
)

target_include_directories(RobotCar_Integration PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/mqtt_robot/include
    ${LWIP_INCLUDE_DIRS}
    ${CYW43_INCLUDE_DIRS}
)

target_link_libraries(RobotCar_Integration
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip
    pico_lwip_mqtt
    hardware_i2c
    hardware_pwm
    hardware_irq
    hardware_timer
    hardware_adc
)

pico_enable_stdio_usb(RobotCar_Integration 1)
pico_enable_stdio_uart(RobotCar_Integration 0)
pico_add_extra_outputs(RobotCar_Integration)

# ============================================================
# === mqtt_robot Build
# ============================================================

add_executable(mqtt_robot
    src/mqtt_robot/src/main.c
    src/mqtt_robot/src/mqtt_pico.c
    src/motor_encoder.c
    src/barcode.c
    src/line_follow.c
    src/pid.c
    src/imu_lis3dh.c
    src/ultrasonic.c
    src/servo.c
)

target_include_directories(mqtt_robot PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/mqtt_robot/include
    ${LWIP_INCLUDE_DIRS}
    ${CYW43_INCLUDE_DIRS}
)

target_link_libraries(mqtt_robot
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip
    pico_lwip_mqtt
    hardware_pwm
    hardware_irq
    hardware_timer
    hardware_adc
    hardware_i2c
)

pico_enable_stdio_usb(mqtt_robot 1)
pico_enable_stdio_uart(mqtt_robot 0)
pico_add_extra_outputs(mqtt_robot)