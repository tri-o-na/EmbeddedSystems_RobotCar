add_executable(robot_demo
    main.c
    imu_mpu6050.c
    motor_encoder.c
    ir_line_sensor.c
    ultrasonic_hcsr04.c
    pid.c
)
target_link_libraries(robot_demo
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_adc
    hardware_timer
    hardware_irq
)
pico_enable_stdio_usb(robot_demo 1)
pico_add_extra_outputs(robot_demo)
