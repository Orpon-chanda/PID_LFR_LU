# PID_LFR_LU — PID Line Follower for Leading University

> Competition-grade PID line follower robot firmware. 8-sensor array, T-section and 90° turn logic, endpoint detection, PlatformIO + Arduino Nano.

![Language](https://img.shields.io/badge/language-C%20%2F%20C%2B%2B-blue) ![Platform](https://img.shields.io/badge/platform-Arduino%20Nano-orange) ![Framework](https://img.shields.io/badge/framework-PlatformIO-9cf) ![Status](https://img.shields.io/badge/status-active-success) ![Updated](https://img.shields.io/badge/updated-2025--12--26-lightgrey)

**Author:** Orpon Chanda — Lab Assistant, Leading University, Sylhet  
**Repo:** `Orpon-chanda/PID_LFR_LU`

---

## Overview

`PID_LFR_LU` is the most recent and structured project in this profile (Dec 2025). It implements a full PID controller for a line-following robot (LFR) designed for inter-university competitions. Unlike the earlier single-file sketches, this repo is organized as a proper PlatformIO project with modular headers and deterministic turn logic.

**What it does:** Reads an 8-channel IR sensor array, computes weighted line position, applies `Kp·P + Ki·∫ + Kd·D`, and drives differential motors with constrained PWM (-255..255). Handles real-world track features: T-junctions, 90° corners, cross-intersections, and end-stop.

## Hardware

| Part | Notes |
|------|-------|
| Arduino Nano (ATmega328P) | `env:nanoatmega328new` |
| 8× IR reflectance sensors | `s[0..7]`, mapped via `sensorRead.h` |
| 2× DC motors + L298N / TB6612 driver | `RIGHT_DIR1/DIR2/EN`, `LEFT_DIR1/DIR2/EN` |
| Push button | `ButtonPin` (INPUT_PULLUP) for mode switch |
| 7.4–12V battery | via motor driver |

Wiring is defined in `include/variable.h` and `include/motor.h` — check those before flashing.

## Project Structure

```
PID_LFR_LU/
├── platformio.ini
├── include/
│   ├── variable.h    # pins, gains Kp/Ki/Kd, baseSpeed, sensorPos[]
│   ├── sensorRead.h  # sensorsGlobal(), sum, sensor bitmask
│   ├── pid.h         # pidControl()
│   ├── motor.h       # motor(left,right)
│   ├── SectionT.h    # tsection(isLR) — T-junction handler
│   ├── endStop.h     # endpoint() — end-of-track stop
│   └── sum.h         # sumsGlobal() helper
├── src/
│   └── main.cpp      # setup(), loop() → LFR() state machine
├── lib/              # (empty, for local libs)
└── test/
```

## How It Works

1. **Sensing** — `sensorsGlobal()` samples 8 sensors, builds `sensor` bitmask (`0b01111110` etc.) and `sum` (active sensors count).
2. **State machine** — `LFR()` loop:
   - `turnLogic()` — debounces turn flags (`flag`, `k90`, `cross`) with 15 ms reset window.
   - `sum == 8` → `endpoint()` (all sensors on line — stop).
   - `sum >=6 && (s[3]||s[4])` → `tsection()` (T-junction, decides left/right).
   - `sum == 1 || sum == 2` → `pidControl()` (normal line tracking).
   - `sum == 0 && flag!=0` → return / sharp turn recovery.
3. **PID** — weighted average of active sensor positions → error → P/I/D → differential output:
   ```cpp
   output = Kp*P + Ki*errorSum + Kd*D;
   motor(baseSpeed - output, baseSpeed + output);
   ```
4. **Turns** — 90° and cross detection via bitmask patterns (`0b11111100`, `0b00111111`, …).

## Getting Started

```bash
# Clone
git clone https://github.com/Orpon-chanda/PID_LFR_LU.git
cd PID_LFR_LU

# Install PlatformIO (once)
pip install platformio

# Build
pio run -e nanoatmega328new

# Upload (adjust port)
pio run -e nanoatmega328new -t upload --upload-port /dev/ttyUSB0

# Serial monitor
pio device monitor -b 9600
```

> No external libraries beyond `Arduino.h`. Tuning is in `variable.h`: adjust `Kp`, `Ki`, `Kd`, `baseSpeed` for your track surface and battery voltage.

## Configuration & Tuning

Open `include/variable.h`:

```cpp
float Kp = ...;  // proportional — start 0.8–1.5
float Ki = ...;  // integral — start 0
float Kd = ...;  // derivative — start 3–8
int baseSpeed = 150; // 80–200 for testing
```

Calibrate sensors on white/black surface, note thresholds. The `sum` and `sensor` debug prints in `sensorRead.h` help.

## Usage

1. Place robot on line, press button. Short press (<250 ms) vs long press (>250 ms) handled by `switchR()`.
2. Observe serial at 9600 baud for sensor diagnostics.
3. Tune PID: increase `Kp` until oscillation, then add `Kd` to dampen, add tiny `Ki` if steady-state offset remains.

## Roadmap

- [ ] Add calibration routine (auto min/max per sensor)
- [ ] OLED debug screen for Kp/Ki/Kd on the fly
- [ ] Battery voltage compensation

## License

No license file — consider adding `MIT` if you want others to reuse.

## Contact

**Orpon Chanda** — Lab Assistant, Leading University, Sylhet  
Email: `934906Rj` (please confirm full address before publishing)  
GitHub: [@Orpon-chanda](https://github.com/Orpon-chanda)

---
*Generated README — original repo had no documentation. Code referenced from `src/main.cpp` and `src/pid.h` as of 2025-12-26.*
