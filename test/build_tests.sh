#!/bin/bash

# Build script for RobotCar unit tests

echo "Building RobotCar Unit Tests..."
echo "================================"

# Create build directory
mkdir -p test_build
cd test_build

# Run CMake
echo "Running CMake..."
cmake .. || {
    echo "CMake failed. Trying manual compilation..."
    cd ..
    
    # Manual compilation fallback
    echo "Compiling manually..."
    gcc -std=c11 -Wall -Wextra -g -O0 \
        test_runner.c \
        test_pid.c \
        test_utils.c \
        ../src/pid.c \
        -I. \
        -I../src \
        -o robotcar_tests \
        -lm
    
    if [ $? -eq 0 ]; then
        echo "Build successful!"
        echo "Run tests with: ./robotcar_tests"
    else
        echo "Build failed!"
        exit 1
    fi
    exit 0
}

# Build
echo "Building..."
make || {
    echo "Make failed!"
    exit 1
}

echo ""
echo "Build successful!"
echo "Run tests with: cd test_build && ./robotcar_tests"

