#pragma once
#include <Arduino.h>
#include "driver/i2s.h"
#include "FluidConfig.h"

// Low-priority microphone capture only. Captured samples are intentionally not
// fed into the solver or any renderer at this stage.
class AudioCapture {
public:
  bool begin() {
    i2s_config_t config = {};
    config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    config.sample_rate = AUDIO_SAMPLE_RATE;
    config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    config.intr_alloc_flags = 0;
    config.dma_buf_count = 4;
    config.dma_buf_len = AUDIO_SAMPLES;
    config.use_apll = false;
    config.tx_desc_auto_clear = false;

    if (i2s_driver_install(I2S_NUM_0, &config, 0, nullptr) != ESP_OK) return false;

    i2s_pin_config_t pins = {};
    pins.bck_io_num = HW_I2S_BCLK;
    pins.ws_io_num = HW_I2S_WS;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num = HW_I2S_MIC;
    if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) {
      i2s_driver_uninstall(I2S_NUM_0);
      return false;
    }
    initialized_ = true;
    return true;
  }

  bool readBlock(int32_t *buffer, size_t bufferBytes) {
    if (!initialized_) return false;
    size_t bytesRead = 0;
    const esp_err_t result = i2s_read(I2S_NUM_0, buffer, bufferBytes,
                                      &bytesRead, 12 / portTICK_PERIOD_MS);
    if (result == ESP_OK && bytesRead > 0) {
      capturedBytes_ += bytesRead;
      ++capturedBlocks_;
      return true;
    }
    ++readErrors_;
    return false;
  }

  bool initialized() const { return initialized_; }
  uint32_t capturedBytes() const { return capturedBytes_; }
  uint32_t capturedBlocks() const { return capturedBlocks_; }
  uint32_t readErrors() const { return readErrors_; }

private:
  bool initialized_ = false;
  volatile uint32_t capturedBytes_ = 0;
  volatile uint32_t capturedBlocks_ = 0;
  volatile uint32_t readErrors_ = 0;
};
