#pragma once
#include <Arduino.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include "FluidTypes.h"

// Two-slot exchange. Core 1 copies a snapshot into its own small struct and
// releases the slot before any TFT work. If that brief copy overlaps a reuse,
// Core 0 skips only that publication; it never waits for drawing.
class FluidSnapshotExchange {
public:
  FluidSnapshotExchange() : publishedSlot_(0), readerSlot_(NO_SLOT) {
    memset(slots_, 0, sizeof(slots_));
  }

  bool beginWrite(uint8_t &slot) {
    portENTER_CRITICAL(&mux_);
    const uint8_t target = (uint8_t)(publishedSlot_ ^ 1u);
    if (readerSlot_ == target) {
      portEXIT_CRITICAL(&mux_);
      return false;
    }
    slot = target;
    portEXIT_CRITICAL(&mux_);
    return true;
  }

  FluidSnapshot &writeBuffer(uint8_t slot) {
    return slots_[slot & 1u];
  }

  void commitWrite(uint8_t slot) {
    portENTER_CRITICAL(&mux_);
    publishedSlot_ = (uint8_t)(slot & 1u);
    portEXIT_CRITICAL(&mux_);
  }

  void copyLatest(FluidSnapshot &destination) {
    uint8_t source;
    portENTER_CRITICAL(&mux_);
    source = publishedSlot_;
    readerSlot_ = source;
    portEXIT_CRITICAL(&mux_);

    memcpy(&destination, &slots_[source], sizeof(FluidSnapshot));

    portENTER_CRITICAL(&mux_);
    if (readerSlot_ == source) readerSlot_ = NO_SLOT;
    portEXIT_CRITICAL(&mux_);
  }

private:
  static const uint8_t SLOT_COUNT = 2;
  static const uint8_t NO_SLOT = 0xFF;
  FluidSnapshot slots_[SLOT_COUNT];
  volatile uint8_t publishedSlot_;
  volatile uint8_t readerSlot_;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
