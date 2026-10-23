#pragma once
#include <Arduino.h>
#include "FluidConfig.h"

enum FluidViewMode : uint8_t {
  VIEW_PARTICLES = 0,
  VIEW_WATER = 1,
  VIEW_INK = 2,
  VIEW_COUNT = 3
};

struct ImuState {
  int16_t accelRaw[3];
  int16_t gyroRaw[3];
  float accelG[3];
  float gyroDps[3];
  float temperatureC;
  float rollDeg;   // gravity-relative, gyro-unwrapped; may exceed ±180°
  float pitchDeg;  // gravity-relative, gyro-unwrapped; may exceed ±180°
  float yawRelativeDeg;  // Gyro-integrated separately; not used for the 2D pool surface.
  bool valid;
};

struct FluidControl {
  float gravityX;   // screen-plane gravity/down vector component; flat pose is (0, 1)
  float gravityY;
  float impulseX;   // angular-rate-derived kick, consumed by one solver step
  float impulseY;
};

// One connected 2D liquid volume, represented by particles in the solver and
// by a 40x40 density/coverage mask in the render snapshot.
struct FluidSnapshot {
  uint8_t fill[FLUID_GRID_CELLS];
  float flowSpeed;  // particle RMS speed, in display-grid cells per second
};
