# FluidSim — ESP32 / Arduino

A single-pool 2D liquid simulation for the ESP32-D0WD-V3 (4 MB flash) and 240 × 240 ST7789 display. The separate `/home/user/imu_spectrum_demo.ino` reference sketch is not modified.

## Build dependencies and board

- Arduino IDE 1.9.8 and the existing `esp32-bluepad32` ESP32 core 4.1.0 profile
- Arduino_GFX
- FastIMU with its MPU6500 driver
- Open `FluidSim.ino` from this folder and use the same board/profile and wiring as the known-working reference sketch.

## Simulation and controls

- The 240 × 240 display is the domain: the screen edges contain the liquid, but no cup outline is drawn. The pool is one connected body, seeded with 126 particles in an 18 × 7 lattice; it settles to the bottom at the calibrated reference pose.
- The fluid kernel is a small 2D adaptation of Clavet double-density relaxation, using the particle-neighbor approach demonstrated by [V4C38/esp32-fluidbox](https://github.com/V4C38/esp32-fluidbox). It uses a 20 × 20 spatial hash, density and near-density pressure, approaching-pair viscosity, and a fixed 20 ms physics step. The fixed particle count avoids the old mass-lattice saturation problem. A 20 × 20 particle-density mask is sampled and bilinearly expanded to the 40 × 40 LED grid.
- IMU startup calibration establishes the gravity-relative zero and gyro bias. The existing complementary filter, sensor axes/signs, and I2C polling/recovery path are retained. Gravity follows the full `(sin(pitch), cos(pitch))` direction through 360°; the existing roll/Z vertical weighting and gyro-derived movement impulse remain in place. Gyro-X yaw does not drive the 2D pool.
- Hold touch GPIO 32 for 3 seconds to cycle **Particles → Water → Ink**. All views render the same simulation snapshot and never reset the liquid.
- I2S microphone samples are captured on a low-priority task only; they do not affect the physics or renderers.

## Rendering and scheduling

- Core 0, priority 4: fluid physics at a 50 Hz target; it consumes the latest IMU sample without waiting for I2C.
- Core 0, priority 2: IMU polling/filter updates at a 100 Hz target. Core 0, priority 1: microphone capture.
- Core 1: touch handling and RGB565 rendering at a roughly 30 FPS target.
- Core 0 publishes through a compact two-slot snapshot exchange. Core 1 copies a snapshot and releases the slot before display work, so TFT rendering cannot block the solver.
- Each frame is rendered in 24-pixel RGB565 bands using an 11,520-byte staging buffer. **Every band is filled with its background and all LEDs are redrawn before that band is sent**, including bands where liquid disappeared; this removes stale-pixel trails. The Arduino_GFX SPI bitmap transfer is used synchronously. The ESP32-S3 reference project's ESP-IDF/QSPI DMA driver is not transplanted to this different ESP32/Arduino/ST7789 setup.
- Highlights are calculated from the current particle-density mask rather than the old stationary crossing-wave/advected-caustic effect. TFT SPI is configured at 60 MHz in `FluidConfig.h`; if this panel/wiring is unstable, lower it to a tested value such as 40 MHz.
- Short scheduling gaps are caught up in fixed 20 ms physics steps, with one display snapshot published per batch.

The Serial Monitor runs at 115200 baud and reports five-second physics/publication rates, maximum physics gap, IMU successes/failures and read time, render FPS, CPU preparation time, and TFT transfer time. The `[tilt]` line reports filtered roll/pitch, gravity target angle, and particle RMS flow speed.

## Pin assignments

Pins are centralized in `FluidConfig.h`:

- TFT SPI: MOSI 13, SCK 14, CS 15, DC 2, backlight 19, power 21
- IMU I2C: SDA 16, SCL 17
- Touch: GPIO 32
- I2S microphone: BCLK 26, WS 25, data 22

## Validation status

Host-only checks were run with a C++11 compiler and a minimal Arduino type stub. The new solver object is 9,132 bytes; `FluidSnapshot` is 1,604 bytes. The host simulation kept the rendered pool in one connected component at representative gravity directions from 0° through 359°, and AddressSanitizer/UndefinedBehaviorSanitizer reported no issue in the tested run. After a 90° tilt and a 20-second return to the flat reference, the normalized fill-mask L1 difference was 0.0065. The three view headers also compiled with `-Wall -Wextra -Werror`.

These are code/host checks only. No ESP32 Arduino toolchain or device is available in this environment, so target compilation, upload, actual 50 Hz physics, 30 FPS rendering, display trail removal, and visual fluid feel are **not yet verified on hardware**. The reference repository's performance numbers apply to its ESP32-S3/QSPI hardware, not this board.
