#pragma once
#include <stdint.h>

// Hardware pins: tested board layout; IMU stays on SDA=16 / SCL=17.
constexpr int HW_TFT_MOSI = 13;
constexpr int HW_TFT_SCK = 14;
constexpr int HW_TFT_CS = 15;
constexpr int HW_TFT_DC = 2;
constexpr int HW_TFT_LIGHT = 19;
constexpr int HW_TFT_POWER = 21;
constexpr uint32_t HW_TFT_SPI_HZ = 80000000;  // test this clock; lower it if this panel/wiring is unstable

constexpr int HW_I2S_BCLK = 26;
constexpr int HW_I2S_WS = 25;
constexpr int HW_I2S_MIC = 22;

constexpr int HW_IMU_SDA = 16;
constexpr int HW_IMU_SCL = 17;

constexpr int HW_TOUCH = 32;

constexpr int SCREEN_W = 240;
constexpr int SCREEN_H = 240;

// The screen/dot matrix itself contains one connected pool; no separate vessel is drawn.
constexpr int FLUID_GRID_W = 60;
constexpr int FLUID_GRID_H = 60;
constexpr int FLUID_GRID_CELLS = FLUID_GRID_W * FLUID_GRID_H;
constexpr float FLUID_FILL_FRACTION = 0.38f;
constexpr int FLUID_SOLVER_SCALE = 1;  // 20x20 particle spatial hash/density samples
constexpr int FLUID_SOLVER_W = FLUID_GRID_W / FLUID_SOLVER_SCALE;
constexpr int FLUID_SOLVER_H = FLUID_GRID_H / FLUID_SOLVER_SCALE;
constexpr int FLUID_SOLVER_CELLS = FLUID_SOLVER_W * FLUID_SOLVER_H;
constexpr int FLUID_PARTICLE_COUNT = 400;  // 18 x 7 starting lattice, about 38% of the 2D domain
constexpr int FLUID_PARTICLE_MAX = 448;
static_assert(FLUID_GRID_W % FLUID_SOLVER_SCALE == 0 &&
              FLUID_GRID_H % FLUID_SOLVER_SCALE == 0,
              "Fluid grid dimensions must be divisible by the solver scale");
static_assert(FLUID_PARTICLE_COUNT <= FLUID_PARTICLE_MAX &&
              FLUID_PARTICLE_MAX <= 65535,
              "Particle indices must fit in the compact neighbor-pair list");
constexpr int FLUID_CELL_PIXELS = SCREEN_W / FLUID_GRID_W;  // 6 pixels per cell
constexpr int RENDER_BAND_CELL_ROWS = 4;
constexpr int RENDER_BAND_HEIGHT = RENDER_BAND_CELL_ROWS * FLUID_CELL_PIXELS;

constexpr uint32_t IMU_PERIOD_MS = 10;       // 100 Hz
constexpr uint32_t FLUID_STEP_MS = 12;       // 50 Hz
constexpr uint32_t RENDER_PERIOD_MS = 27;    // target ~30 FPS
constexpr uint32_t VIEW_HOLD_MS = 500;      // one view change per 3-second touch

constexpr float ACCEL_LSB_PER_G = 16384.0f;  // ±2 g
constexpr float GYRO_LSB_PER_DPS = 131.0f;   // ±250 deg/s
constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr float GYRO_DEADBAND_DPS = 0.45f;
constexpr float MOTION_IMPULSE_THRESHOLD_DPS = 14.0f;
constexpr uint32_t MOTION_IMPULSE_INTERVAL_MS = 20;
constexpr float MOTION_IMPULSE_GAIN = 0.07f;
constexpr float MOTION_IMPULSE_MAX = 16.0f;

constexpr int AUDIO_SAMPLE_RATE = 16000;
constexpr int AUDIO_SAMPLES = 128;
