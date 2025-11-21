cmake_minimum_required(VERSION 3.13)

# Root project definition
project(week6demo C CXX ASM)

# Import Pico SDK (from env var or local copy)
if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake")
    include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)
else()
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

# Initialise the Pico SDK
pico_sdk_init()

# Add subdirectories for each demo set
add_subdirectory(other4)
add_subdirectory(WiFi)
add_subdirectory(imu)
add_subdirectory(Integration)

# You can add more demo folders the same way if you make them
