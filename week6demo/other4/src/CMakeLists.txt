# week6demo/other4/CMakeLists.txt
project(other4_demos C CXX ASM)

# ---------- Motor + wheel encoder (PWM + timer/IRQ) ----------
add_executable(motor_encoder
    src/motor_encoder.c
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

# ---------- IR line sensor (ADC + digital pulse width) ----------
add_executable(ir_line_sensor
    src/ir_line_sensor.c
)
pico_enable_stdio_usb(ir_line_sensor 1)
pico_enable_stdio_uart(ir_line_sensor 0)
target_link_libraries(ir_line_sensor
    pico_stdlib
    hardware_adc
    hardware_timer
)
pico_add_extra_outputs(ir_line_sensor)

# ---------- Accelerometer raw (MPU6050 over I2C) ----------
add_executable(accel_raw_mpu6050
    src/accel_raw_mpu6050.c
)
pico_enable_stdio_usb(accel_raw_mpu6050 1)
pico_enable_stdio_uart(accel_raw_mpu6050 0)
target_link_libraries(accel_raw_mpu6050
    pico_stdlib
    hardware_i2c
)
pico_add_extra_outputs(accel_raw_mpu6050)

# ---------- Ultrasonic HC-SR04 (echo pulse timing) ----------
add_executable(ultrasonic_hcsr04
    src/ultrasonic_hcsr04.c
)
pico_enable_stdio_usb(ultrasonic_hcsr04 1)
pico_enable_stdio_uart(ultrasonic_hcsr04 0)
target_link_libraries(ultrasonic_hcsr04
    pico_stdlib
    hardware_timer
)
pico_add_extra_outputs(ultrasonic_hcsr04)
