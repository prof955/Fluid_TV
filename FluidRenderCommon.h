#pragma once
#include "FluidTypes.h"

namespace FluidRender {
inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

inline uint8_t clampByte(int value) {
  return (uint8_t)(value < 0 ? 0 : (value > 255 ? 255 : value));
}

inline float clampUnit(float value) {
  return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

inline bool isLiquidSurfaceCell(const FluidSnapshot &state, int x, int y) {
  const int i = y * FLUID_GRID_W + x;
  if (state.fill[i] < 12) return false;
  const int left = x > 0 ? i - 1 : -1;
  const int right = x + 1 < FLUID_GRID_W ? i + 1 : -1;
  const int top = y > 0 ? i - FLUID_GRID_W : -1;
  const int bottom = y + 1 < FLUID_GRID_H ? i + FLUID_GRID_W : -1;
  return left < 0 || right < 0 || top < 0 || bottom < 0 ||
         state.fill[left] < 12 || state.fill[right] < 12 ||
         state.fill[top] < 12 || state.fill[bottom] < 12;
}

// Moving particle density drives this facet lighting. It replaces the old
// stationary crossing-wave pattern, so highlights follow the simulated pool.
inline float densityLightingAt(const FluidSnapshot &state, int x, int y) {
  const int i = y * FLUID_GRID_W + x;
  const int left = x > 0 ? (int)state.fill[i - 1] : (int)state.fill[i];
  const int right = x + 1 < FLUID_GRID_W ? (int)state.fill[i + 1] : (int)state.fill[i];
  const int top = y > 0 ? (int)state.fill[i - FLUID_GRID_W] : (int)state.fill[i];
  const int bottom = y + 1 < FLUID_GRID_H ?
                     (int)state.fill[i + FLUID_GRID_W] : (int)state.fill[i];
  const float gx = (float)(right - left) * (1.0f / 255.0f);
  const float gy = (float)(bottom - top) * (1.0f / 255.0f);
  const float edge = (gx < 0.0f ? -gx : gx) + (gy < 0.0f ? -gy : gy);
  float motion = state.flowSpeed * 0.0025f;
  if (motion > 0.12f) motion = 0.12f;
  return clampUnit(0.69f + gx * 0.20f - gy * 0.24f + edge * 0.10f + motion);
}

inline void fillBandBackground(uint16_t *frame, int cellRows, uint16_t color) {
  const int pixelRows = cellRows * FLUID_CELL_PIXELS;
  for (int i = 0; i < pixelRows * SCREEN_W; ++i) frame[i] = color;
}

// Draw one physical-looking diamond LED inside one 6 x 6 grid cell.
inline void drawMatrixLed(uint16_t *frame, int cellX, int cellY, int firstCellRow,
                          uint16_t ledColor, uint16_t ledHighlight) {
  const int originX = cellX * FLUID_CELL_PIXELS;
  const int originY = (cellY - firstCellRow) * FLUID_CELL_PIXELS;

  // Four short integer spans: no per-pixel floating-point distance tests.
  for (int py = 1; py <= 4; ++py) {
    const int firstPx = (py == 1 || py == 4) ? 2 : 1;
    const int pixelCount = (py == 1 || py == 4) ? 2 : 4;
    uint16_t *row = frame + (originY + py) * SCREEN_W + originX;
    for (int i = 0; i < pixelCount; ++i) {
      const int px = firstPx + i;
      const bool highlight = py == 1 && px >= 2 && px <= 3;
      row[px] = highlight ? ledHighlight : ledColor;
    }
  }
}
}  // namespace FluidRender
