add_executable(imu_test
    src/imu_test.c
    src/imu.c
)

target_include_directories(imu_test PRIVATE include)
target_link_libraries(imu_test pico_stdlib hardware_i2c)
pico_enable_stdio_usb(imu_test 1)
pico_enable_stdio_uart(imu_test 0)
pico_add_extra_outputs(imu_test)
