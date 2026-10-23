#pragma once
#include "FluidRenderCommon.h"

namespace FluidRender {
inline void renderWater(const FluidSnapshot &state, uint16_t *frame,
                        int firstCellRow, int cellRowCount) {
  fillBandBackground(frame, cellRowCount, rgb565(1, 8, 12));
  const uint16_t unlit = rgb565(32, 45, 47);
  const uint16_t unlitHighlight = rgb565(70, 86, 88);
  const int lastCellRow = firstCellRow + cellRowCount;

  for (int cy = firstCellRow; cy < lastCellRow; ++cy) {
    for (int cx = 0; cx < FLUID_GRID_W; ++cx) {
      const uint8_t fill = state.fill[cy * FLUID_GRID_W + cx];
      if (fill < 3) {
        drawMatrixLed(frame, cx, cy, firstCellRow, unlit, unlitHighlight);
        continue;
      }

      const bool surface = isLiquidSurfaceCell(state, cx, cy);
      const float fillAmount = (float)fill * (1.0f / 255.0f);
      const float light = densityLightingAt(state, cx, cy);
      const float level = 0.39f + fillAmount * 0.40f +
                          (light - 0.69f) * 0.72f +
                          (surface ? 0.19f : 0.0f);
      const uint16_t led = rgb565(clampByte((int)(2.0f + level * 11.0f)),
                                  clampByte((int)(58.0f + level * 172.0f)),
                                  clampByte((int)(20.0f + level * 80.0f)));
      const uint16_t highlight = surface ? rgb565(150, 255, 195) : rgb565(65, 235, 125);
      drawMatrixLed(frame, cx, cy, firstCellRow, led, highlight);
    }
  }
}
}  // namespace FluidRender
