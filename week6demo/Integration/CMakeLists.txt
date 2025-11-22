add_executable(Integration
    main.c
    imu_mpu6050.c
    motor_encoder.c
    pid.c
    line_follow.c
    barcode.c
)

target_link_libraries(Integration
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_adc
    hardware_pwm
)

pico_enable_stdio_usb(Integration 1)
pico_enable_stdio_uart(Integration 0)
pico_add_extra_outputs(Integration)
