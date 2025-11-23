cmake_minimum_required(VERSION 3.13)

# Test configuration
project(RobotCar_Tests C CXX)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Add Unity test framework (simple embedded-friendly test framework)
# For now, we'll use a simple test framework without external dependencies
# Unity can be added later if needed

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Test source files
set(TEST_SOURCES
    test_runner.c
    test_pid.c
    test_utils.c
)

# Include test headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Source files to test (without hardware dependencies)
set(SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pid.c
)

# Create test executable
add_executable(robotcar_tests
    ${TEST_SOURCES}
    ${SRC_FILES}
)

# Compiler flags for tests
target_compile_options(robotcar_tests PRIVATE
    -Wall
    -Wextra
    -Werror
    -g
    -O0
)

# Link math library if needed
target_link_libraries(robotcar_tests m)

# Add test target
enable_testing()
add_test(NAME RobotCar_Tests COMMAND robotcar_tests)

