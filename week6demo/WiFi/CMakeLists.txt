# WiFi/CMakeLists.txt
project(wifi_demo C CXX ASM)

# Create executable from your main.c
add_executable(wifi_demo
    src/main.c
)

# Add include path for config.h and lwipopts.h
target_include_directories(wifi_demo PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# Enable USB stdio for debugging (disable UART if you like)
pico_enable_stdio_usb(wifi_demo 1)
pico_enable_stdio_uart(wifi_demo 0)

# Link against required libraries:
# - pico_stdlib             : basic I/O support
# - pico_cyw43_arch_lwip_threadsafe_background : WiFi driver + lwIP
target_link_libraries(wifi_demo
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)

# Generate UF2, bin, and hex files
pico_add_extra_outputs(wifi_demo)
