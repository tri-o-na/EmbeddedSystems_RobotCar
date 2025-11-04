# ----- Libraries -----
add_library(pico_ultrasonic ultrasonic.c ultrasonic.h)
target_link_libraries(pico_ultrasonic pico_stdlib hardware_gpio hardware_timer)
target_include_directories(pico_ultrasonic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(pico_ultrasonic_object ultrasonic.c ultrasonic.h ultrasonicObject.cpp ultrasonicObject.h)
target_link_libraries(pico_ultrasonic_object pico_stdlib hardware_gpio hardware_timer)
target_include_directories(pico_ultrasonic_object PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ----- Executable -----
add_executable(ultrasonic 
    main.c 
    ultrasonic.c
    servo.c        
    motor_encoder.c 
    imu_lis3dh.c
    pid.c
    mqtt_pico.c    # Add this
)

target_link_libraries(ultrasonic
    pico_stdlib
    hardware_gpio
    hardware_timer
    hardware_i2c
    hardware_pwm
    hardware_adc
    pico_cyw43_arch_lwip_threadsafe_background  # Add this
    pico_lwip_mqtt                              # Add this
)

pico_enable_stdio_usb(ultrasonic 1)
pico_enable_stdio_uart(ultrasonic 0)
pico_add_extra_outputs(ultrasonic)

