## Robot Car (Raspberry Pi Pico W)

### Goal
- Control two DC motors with variable speed and capture wheel encoder pulse width data while rotating.
- Execute a repeatable motion sequence (loops forever):
  - Rotate clockwise 2 s
  - Rotate counter-clockwise 2 s
  - Drive forward 2 s
  - Drive backward 2 s
  - Pause 1 s

### Hardware
- Dual H-bridge motor driver (e.g., L298N / TB6612FNG / DRV8833).
- Common ground shared between Pico W, driver, and motor power supply.
- Wheel encoders available (channel A used; channel B optional).

### Pin Mapping
- Motor 1 (left) — Cytron Robo Pico
  - M1A (PWM): GP8
  - M1B (PWM): GP9
- Motor 2 (right) — Cytron Robo Pico
  - M2A (PWM): GP10
  - M2B (PWM): GP11
- Encoders
  - ENC1_A: GP14
  - ENC2_A: GP15

If the driver has a STBY/EN pin (e.g., TB6612FNG), assign it a GPIO and set high to enable.

### Behavior
- The program in `week6demo/other4/src/motor_encoder.c` performs the motion loop above.
- It ramps speed within each 2 s segment (about 30% → 90%) and prints encoder pulse widths over USB.

### Build and Flash (generic)
```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j
# After build, copy the generated .uf2 to the RPI-RP2 drive while holding BOOTSEL
```

### Next Steps
- If your wiring differs, update the pin macros at the top of `week6demo/other4/src/motor_encoder.c`.
- Ensure the H-bridge enable/STBY (if present) is asserted.


