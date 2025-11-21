add_executable(Integration
    main.c
)

target_link_libraries(Integration
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_adc
    motor_encoder
    pid
    line_follow
    barcode
    servo
    ultrasonic
)

pico_enable_stdio_usb(Integration 1)
pico_enable_stdio_uart(Integration 0)

pico_add_extra_outputs(Integration)
