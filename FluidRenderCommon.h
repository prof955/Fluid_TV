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

//inline bool isLiquidSurfaceCell(const FluidSnapshot &state, int x, int y) {
//  const int i = y * FLUID_GRID_W + x;
//  if (state.fill[i] < 12) return false;
//  const int left = x > 0 ? i - 1 : -1;
//  const int right = x + 1 < FLUID_GRID_W ? i + 1 : -1;
//  const int top = y > 0 ? i - FLUID_GRID_W : -1;
//  const int bottom = y + 1 < FLUID_GRID_H ? i + FLUID_GRID_W : -1;
//  return left < 0 || right < 0 || top < 0 || bottom < 0 ||
//         state.fill[left] < 12 || state.fill[right] < 12 ||
//         state.fill[top] < 12 || state.fill[bottom] < 12;
//}
// Her yön için ayrı yüzey algılama derinliği (topDepth, bottomDepth, leftDepth, rightDepth)
//inline bool isCustomLiquidSurfaceCell(const FluidSnapshot &state, int x, int y,
//                                       int topDepth = 2, int bottomDepth = 2,
//                                       int leftDepth = 2, int rightDepth = 2) {
//  const int i = y * FLUID_GRID_W + x;
//  if (state.fill[i] < 12) return false;
//
//  // 1. ÜST YÜZEY KONTROLÜ (topDepth kadar yukarıya bakar)
//  for (int dy = 1; dy <= topDepth; ++dy) {
//    const int ny = y - dy; // Yukarı yön (-y)
//    if (ny < 0) return true; // Ekranın üst sınırına geldiyse yüzeydir
//    if (state.fill[ny * FLUID_GRID_W + x] < 12) return true; // Üstte boşluk var
//  }
//
//  // 2. ALT YÜZEY KONTROLÜ (bottomDepth kadar aşağıya bakar)
//  for (int dy = 1; dy <= bottomDepth; ++dy) {
//    const int ny = y + dy; // Aşağı yön (+y)
//    if (ny >= FLUID_GRID_H) return true;
//    if (state.fill[ny * FLUID_GRID_W + x] < 12) return true;
//  }
//
//  // 3. SOL YÜZEY KONTROLÜ (leftDepth kadar sola bakar)
//  for (int dx = 1; dx <= leftDepth; ++dx) {
//    const int nx = x - dx; // Sol yön (-x)
//    if (nx < 0) return true;
//    if (state.fill[y * FLUID_GRID_W + nx] < 12) return true;
//  }
//
//  // 4. SAĞ YÜZEY KONTROLÜ (rightDepth kadar sağa bakar)
//  for (int dx = 1; dx <= rightDepth; ++dx) {
//    const int nx = x + dx; // Sağ yön (+x)
//    if (nx >= FLUID_GRID_W) return true;
//    if (state.fill[y * FLUID_GRID_W + nx] < 12) return true;
//  }
//
//  return false;
//}
// FluidRenderCommon.h
inline bool isCustomLiquidSurfaceCell(const FluidSnapshot &state, int x, int y,
                                       int topDepth = 1, int bottomDepth = 1,
                                       int leftDepth = 1, int rightDepth = 1) {
  const int i = y * FLUID_GRID_W + x;
  if (state.fill[i] < 12) return false;

  // 1. ÜST YÜZEY KONTROLÜ
  for (int dy = 1; dy <= topDepth; ++dy) {
    const int ny = y - dy;
    // Ekranın en üst duvarına değiyorsa parlak yüzey YAPMA (duvara yapışık sudur)
    if (ny < 0) break; 
    // Gerçek bir hava/boşluk varsa parlak yüzeydir!
    if (state.fill[ny * FLUID_GRID_W + x] < 12) return true; 
  }

  // 2. ALT YÜZEY KONTROLÜ
  for (int dy = 1; dy <= bottomDepth; ++dy) {
    const int ny = y + dy;
    if (ny >= FLUID_GRID_H) break; // Alt duvara değiyorsa parlak yapma
    if (state.fill[ny * FLUID_GRID_W + x] < 12) return true;
  }

  // 3. SOL YÜZEY KONTROLÜ
  for (int dx = 1; dx <= leftDepth; ++dx) {
    const int nx = x - dx;
    if (nx < 0) break; // Sol duvara değiyorsa parlak yapma
    if (state.fill[y * FLUID_GRID_W + nx] < 12) return true;
  }

  // 4. SAĞ YÜZEY KONTROLÜ
  for (int dx = 1; dx <= rightDepth; ++dx) {
    const int nx = x + dx;
    if (nx >= FLUID_GRID_W) break; // Sağ duvara değiyorsa parlak yapma
    if (state.fill[y * FLUID_GRID_W + nx] < 12) return true;
  }

  return false;
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
//inline void drawMatrixLed(uint16_t *frame, int cellX, int cellY, int firstCellRow,
//                          uint16_t ledColor, uint16_t ledHighlight) {
//  const int originX = cellX * FLUID_CELL_PIXELS;
//  const int originY = (cellY - firstCellRow) * FLUID_CELL_PIXELS;
//
//  // Four short integer spans: no per-pixel floating-point distance tests.
//  for (int py = 1; py <= 4; ++py) {
//    const int firstPx = (py == 1 || py == 4) ? 2 : 1;
//    const int pixelCount = (py == 1 || py == 4) ? 2 : 4;
//    uint16_t *row = frame + (originY + py) * SCREEN_W + originX;
//    for (int i = 0; i < pixelCount; ++i) {
//      const int px = firstPx + i;
//      const bool highlight = py == 1 && px >= 2 && px <= 3;
//      row[px] = highlight ? ledHighlight : ledColor;
//    }
//  }
//}
inline void drawMatrixLed(uint16_t *frame, int cellX, int cellY, int firstCellRow,
                          uint16_t ledColor, uint16_t ledHighlight) {
  const int originX = cellX * FLUID_CELL_PIXELS;
  const int originY = (cellY - firstCellRow) * FLUID_CELL_PIXELS;
  const int cellPx = FLUID_CELL_PIXELS;

  // Hücre boyutuna (3px, 4px, 6px vb.) tam uyum sağlayan dinamik çizim
  for (int py = 0; py < cellPx; ++py) {
    uint16_t *row = frame + (originY + py) * SCREEN_W + originX;
    for (int px = 0; px < cellPx; ++px) {
      // Piksel kenarlarında hafif boşluk/vurgu bırak
      const bool isEdge = (py == 0 || py == cellPx - 1 || px == 0 || px == cellPx - 1);
      if (isEdge && cellPx > 3) {
        row[px] = (py == 0 && px > 0 && px < cellPx - 1) ? ledHighlight : ledColor;
      } else {
        row[px] = (py == 0 && px == 0) ? ledHighlight : ledColor;
      }
    }
  }
}
}  // namespace FluidRender
