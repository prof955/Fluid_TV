#pragma once
#include "FluidRenderCommon.h"

namespace FluidRender {
inline void renderParticles(const FluidSnapshot &state, uint16_t *frame,
                            int firstCellRow, int cellRowCount) {
  fillBandBackground(frame, cellRowCount, rgb565(2, 7, 6));
  const uint16_t unlit = rgb565(39, 48, 45);
  const uint16_t unlitHighlight = rgb565(78, 88, 82);
  const int lastCellRow = firstCellRow + cellRowCount;

  for (int cy = firstCellRow; cy < lastCellRow; ++cy) {
    for (int cx = 0; cx < FLUID_GRID_W; ++cx) {
      const uint8_t fill = state.fill[cy * FLUID_GRID_W + cx];
      if (fill < 3) {
        drawMatrixLed(frame, cx, cy, firstCellRow, unlit, unlitHighlight);
        continue;
      }

      const bool surface = isCustomLiquidSurfaceCell(state, cx, cy);
      const float fillAmount = (float)fill * (1.0f / 255.0f);
      const float light = densityLightingAt(state, cx, cy);
      const float brightness = 0.43f + fillAmount * 0.40f +
                               (light - 0.69f) * 0.62f +
                               (surface ? 0.13f : 0.0f);
      const uint16_t led = rgb565(clampByte((int)(2.0f + brightness * 13.0f)),
                                  clampByte((int)(52.0f + brightness * 185.0f)),
                                  clampByte((int)(10.0f + brightness * 39.0f)));
      const uint16_t highlight = rgb565(100, 255, 150);
      drawMatrixLed(frame, cx, cy, firstCellRow, led, highlight);
    }
  }
}
}  // namespace FluidRender
