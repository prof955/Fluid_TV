#pragma once
#include "FluidRenderCommon.h"

namespace FluidRender {
inline void renderWater(const FluidSnapshot &state, uint16_t *frame,
                        int firstCellRow, int cellRowCount) {
  // 1. Arka planı TAM SIYAH yapıyoruz
  fillBandBackground(frame, cellRowCount, rgb565(0, 0, 0));
  
  // Sönük/boş LED'leri çok koyu lacivert yapalım ki matris dokusu belli olsun
  const uint16_t unlit = rgb565(0, 0, 0);
  const uint16_t unlitHighlight = rgb565(0, 0, 0);
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
      const float level = 0.75f + fillAmount * 0.45f +
                          (light - 0.69f) * 0.65f +
                          (surface ? 0.20f : 0.0f);

      // 2. SU MAVİSİ Renk Paleti (R, G, B):
      // Derin alanlar: Canlı Okyanus Mavisi
      // Yüzey / Parlamalar: Açık Turkuaz / Beyazımsı Mavi
//      const uint8_t r = clampByte((int)(10.0f + level * 40.0f));
//      const uint8_t g = clampByte((int)(90.0f + level * 145.0f));
//      const uint8_t b = clampByte((int)(160.0f + level * 95.0f));
//
//      const uint16_t led = rgb565(r, g, b);
      const uint16_t led = rgb565(clampByte((int)(0.0f + level * 20.0f)),
                            clampByte((int)(60.0f + level * 60.0f)),
                            clampByte((int)(140.0f + level * 55.0f)));
      
      // Yüzey köpüğü/parlaması için açık buz mavisi
      //const uint16_t highlight = surface ? rgb565(180, 245, 255) : rgb565(90, 200, 255);
      const uint16_t highlight = surface ? rgb565(240, 255, 255) : rgb565(40, 40, 40);
      
      drawMatrixLed(frame, cx, cy, firstCellRow, led, highlight);
    }
  }
}
}  // namespace FluidRender
