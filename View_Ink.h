#pragma once
#include "FluidRenderCommon.h"

namespace FluidRender {
inline void renderInk(const FluidSnapshot &state, uint16_t *frame,
                      int firstCellRow, int cellRowCount) {
  fillBandBackground(frame, cellRowCount, rgb565(3, 6, 5));
  const uint16_t unlit = rgb565(0, 0, 0);
  const uint16_t unlitHighlight = rgb565(70, 78, 73);
  const int lastCellRow = firstCellRow + cellRowCount;

  for (int cy = firstCellRow; cy < lastCellRow; ++cy) {
    for (int cx = 0; cx < FLUID_GRID_W; ++cx) {
      const uint8_t fill = state.fill[cy * FLUID_GRID_W + cx];
      if (fill < 3) {
        drawMatrixLed(frame, cx, cy, firstCellRow, unlit, unlitHighlight);
        continue;
      }

//      const bool surface = isLiquidSurfaceCell(state, cx, cy);
//      const bool surface = isCustomLiquidSurfaceCell(state, cx, cy, 3, 0, 1, 1);
      const bool surface = isCustomLiquidSurfaceCell(state, cx, cy);
      const float fillAmount = (float)fill * (1.0f / 255.0f);
      const float light = densityLightingAt(state, cx, cy);
      const float pigment = 0.31f + fillAmount * 0.43f +
                            (light - 0.69f) * 0.54f +
                            (surface ? 0.08f : 0.0f);
      const uint16_t led = rgb565(clampByte((int)(2.0f + pigment * 15.0f)),
                                  clampByte((int)(35.0f + pigment * 120.0f)),
                                  clampByte((int)(12.0f + pigment * 35.0f)));
      const uint16_t highlight = surface ? rgb565(75, 255, 95) : rgb565(30, 170, 65);
      drawMatrixLed(frame, cx, cy, firstCellRow, led, highlight);
    }
  }
}
}  // namespace FluidRender
