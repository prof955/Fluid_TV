#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <math.h>
#include "FluidConfig.h"
#include "FluidTypes.h"
#include "FluidKernel.h"
#include "FluidShared.h"
#include "ImuInput.h"
#include "AudioCapture.h"
#include "View_Particles.h"
#include "View_Water.h"
#include "View_Ink.h"

Arduino_DataBus *displayBus = new Arduino_ESP32SPI(
  HW_TFT_DC, HW_TFT_CS, HW_TFT_SCK, HW_TFT_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *display = new Arduino_ST7789(
  displayBus, GFX_NOT_DEFINED, 0, true, SCREEN_W, SCREEN_H, 0, 0, 0, 0);

static FluidSimulation gFluid;
static FluidSnapshotExchange gSnapshots;
static FluidSnapshot gRenderSnapshot;
static ImuInput gImu;
static AudioCapture gMicrophone;
static uint16_t gFramebuffer[SCREEN_W * RENDER_BAND_HEIGHT];

struct ImuTelemetry {
  uint32_t attempts;
  uint32_t successes;
  uint32_t failures;
  uint32_t totalReadUs;
  uint32_t maxReadUs;
};
static portMUX_TYPE gImuMux = portMUX_INITIALIZER_UNLOCKED;
static ImuState gImuSnapshot = {};
static ImuTelemetry gImuTelemetry = {};
static bool gImuReady = false;
static uint32_t gImuSequence = 0;
static uint32_t gImuLastGoodMs = 0;

static FluidViewMode gView = VIEW_PARTICLES;
static bool gTouchHeld = false;
static bool gTouchTriggered = false;
static uint32_t gTouchStartedMs = 0;
static uint32_t gLastRenderMs = 0;
static uint32_t gLastRenderReportMs = 0;
static uint32_t gRenderFrames = 0;
static uint32_t gRenderTimeTotalUs = 0;
static uint32_t gRenderPrepTotalUs = 0;
static uint32_t gRenderTransferTotalUs = 0;
static uint32_t gRenderTimeMaxUs = 0;

static TaskHandle_t gFluidTaskHandle = nullptr;
static TaskHandle_t gImuTaskHandle = nullptr;
static TaskHandle_t gAudioTaskHandle = nullptr;

void fluidTaskCode(void *parameter);
void imuTaskCode(void *parameter);
void audioTaskCode(void *parameter);
static void configureImuBus();
static void recoverImuBus();
static bool copyLatestImu(ImuState &destination, uint32_t &sequence,
                          uint32_t &sampleMs);
static ImuTelemetry takeImuTelemetry();
static bool touchedNow();
static void updateTouch();
static void renderCurrentView(const FluidSnapshot &snapshot, int firstCellRow,
                              int cellRowCount);
static void renderFrame(const FluidSnapshot &snapshot, uint32_t &prepUs, uint32_t &transferUs);

void setup() {
  Serial.begin(115200);

  pinMode(HW_TFT_POWER, OUTPUT);
  pinMode(HW_TFT_LIGHT, OUTPUT);
  // Preserve the tested board's TFT power/backlight active level.
  digitalWrite(HW_TFT_POWER, LOW);
  digitalWrite(HW_TFT_LIGHT, LOW);

  display->begin(HW_TFT_SPI_HZ);
  display->fillScreen(0x0000);

  configureImuBus();

  gFluid.begin();
  uint8_t initialSnapshotSlot = 0;
  if (gSnapshots.beginWrite(initialSnapshotSlot)) {
    gFluid.makeSnapshot(gSnapshots.writeBuffer(initialSnapshotSlot));
    gSnapshots.commitWrite(initialSnapshotSlot);
  }
  gSnapshots.copyLatest(gRenderSnapshot);
  uint32_t initialPrepUs = 0;
  uint32_t initialTransferUs = 0;
  renderFrame(gRenderSnapshot, initialPrepUs, initialTransferUs);

  const bool micReady = gMicrophone.begin();
  Serial.printf("FluidSim boot | grid %dx%d | initial view Particles | microphone %s\n",
                FLUID_GRID_W, FLUID_GRID_H, micReady ? "ready (capture only)" : "not found");

  const BaseType_t fluidCreated = xTaskCreatePinnedToCore(
    fluidTaskCode, "FluidTask", 16384, nullptr, 4, &gFluidTaskHandle, 0);
  if (fluidCreated != pdPASS) {
    gFluidTaskHandle = nullptr;
    Serial.println("ERROR: Core 0 fluid task could not be created");
  }

  const BaseType_t imuCreated = xTaskCreatePinnedToCore(
    imuTaskCode, "ImuRead", 4096, nullptr, 5, &gImuTaskHandle, 0);
  if (imuCreated != pdPASS) {
    gImuTaskHandle = nullptr;
    Serial.println("ERROR: Core 0 IMU task could not be created");
  }

  if (micReady) {
    const BaseType_t audioCreated = xTaskCreatePinnedToCore(
      audioTaskCode, "MicCapture", 4096, nullptr, 1, &gAudioTaskHandle, 0);
    if (audioCreated != pdPASS) {
      gAudioTaskHandle = nullptr;
      Serial.println("WARNING: microphone capture task could not be created");
    }
  }
}

void loop() {
  updateTouch();

  const uint32_t now = millis();
  if ((uint32_t)(now - gLastRenderMs) < RENDER_PERIOD_MS) {
    delay(1);
    return;
  }
  gLastRenderMs = now;

  const uint32_t renderStartUs = micros();
  uint32_t prepUs = 0;
  uint32_t transferUs = 0;
  uint32_t copyStartUs = micros();
  gSnapshots.copyLatest(gRenderSnapshot);
  prepUs += (uint32_t)(micros() - copyStartUs);
  renderFrame(gRenderSnapshot, prepUs, transferUs);
  const uint32_t renderUs = (uint32_t)(micros() - renderStartUs);

  ++gRenderFrames;
  gRenderTimeTotalUs += renderUs;
  gRenderPrepTotalUs += prepUs;
  gRenderTransferTotalUs += transferUs;
  if (renderUs > gRenderTimeMaxUs) gRenderTimeMaxUs = renderUs;

  if ((uint32_t)(now - gLastRenderReportMs) >= 5000) {
    const uint32_t reportWindowMs = gLastRenderReportMs == 0 ? now : now - gLastRenderReportMs;
    const uint32_t fpsX10 = reportWindowMs > 0 ? (gRenderFrames * 10000UL) / reportWindowMs : 0;
    const uint32_t avgUs = gRenderFrames ? gRenderTimeTotalUs / gRenderFrames : 0;
    const uint32_t avgPrepUs = gRenderFrames ? gRenderPrepTotalUs / gRenderFrames : 0;
    const uint32_t avgTransferUs = gRenderFrames ? gRenderTransferTotalUs / gRenderFrames : 0;
    Serial.printf("[render] %lu.%01lu fps | avg %lu us (prep %lu, TFT %lu) | max %lu us | mode %u\n",
                  (unsigned long)(fpsX10 / 10), (unsigned long)(fpsX10 % 10),
                  (unsigned long)avgUs, (unsigned long)avgPrepUs,
                  (unsigned long)avgTransferUs, (unsigned long)gRenderTimeMaxUs,
                  (unsigned int)gView);
    gLastRenderReportMs = now;
    gRenderFrames = 0;
    gRenderTimeTotalUs = 0;
    gRenderPrepTotalUs = 0;
    gRenderTransferTotalUs = 0;
    gRenderTimeMaxUs = 0;
  }
}

static void configureImuBus() {
  Wire.begin(HW_IMU_SDA, HW_IMU_SCL);
  Wire.setTimeOut(5);
  // A standard-rate bus is more tolerant of longer board wiring and small
  // signal-integrity problems; 14-byte IMU reads still fit comfortably at 100 Hz.
  Wire.setClock(400000);//400000 can be tried!
}

static void recoverImuBus() {
  // Wire.end()/begin() resets the ESP32 peripheral, but cannot free a slave
  // that is left holding SDA low. Release the pins and clock the bus 9 times,
  // then synthesize STOP before handing the pins back to Wire.
  Wire.end();
  pinMode(HW_IMU_SDA, INPUT_PULLUP);
  pinMode(HW_IMU_SCL, INPUT_PULLUP);
  delayMicroseconds(5);

  for (uint8_t pulse = 0; pulse < 9 && digitalRead(HW_IMU_SDA) == LOW; ++pulse) {
    digitalWrite(HW_IMU_SCL, LOW);  // Set output latch before enabling the driver.
    pinMode(HW_IMU_SCL, OUTPUT);
    delayMicroseconds(5);
    pinMode(HW_IMU_SCL, INPUT_PULLUP);  // Open-drain release, never drive SCL high.
    delayMicroseconds(5);
  }

  // STOP: SDA low-to-released while SCL is released high.
  digitalWrite(HW_IMU_SDA, LOW);  // Set output latch before enabling the driver.
  pinMode(HW_IMU_SDA, OUTPUT);
  delayMicroseconds(5);
  pinMode(HW_IMU_SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(HW_IMU_SDA, INPUT_PULLUP);
  delayMicroseconds(5);

  configureImuBus();
}

static bool copyLatestImu(ImuState &destination, uint32_t &sequence,
                          uint32_t &sampleMs) {
  portENTER_CRITICAL(&gImuMux);
  const bool ready = gImuReady;
  destination = gImuSnapshot;
  sequence = gImuSequence;
  sampleMs = gImuLastGoodMs;
  portEXIT_CRITICAL(&gImuMux);
  return ready;
}

static ImuTelemetry takeImuTelemetry() {
  ImuTelemetry result;
  portENTER_CRITICAL(&gImuMux);
  result = gImuTelemetry;
  memset(&gImuTelemetry, 0, sizeof(gImuTelemetry));
  portEXIT_CRITICAL(&gImuMux);
  return result;
}

void imuTaskCode(void *parameter) {
  (void)parameter;
  const bool ready = gImu.begin();
  portENTER_CRITICAL(&gImuMux);
  gImuReady = ready;
  gImuSnapshot.valid = false;
  gImuSequence = 0;
  gImuLastGoodMs = 0;
  portEXIT_CRITICAL(&gImuMux);

  if (ready) {
    Serial.printf("[imu] ready: %s @ 0x%02X | averaged startup gravity reference %s\n",
                  gImu.modelName(), gImu.address(),
                  gImu.accelReferenceReady() ? "valid" : "pending");
  } else {
    Serial.println("[imu] not found; fluid continues without tilt");
    for (;;) vTaskDelay(1000);
  }

  uint32_t lastUpdateMs = millis();
  for (;;) {
    const uint32_t now = millis();
    if ((uint32_t)(now - lastUpdateMs) >= IMU_PERIOD_MS) {
      lastUpdateMs = now;
      const uint32_t readStartUs = micros();
      const bool updated = gImu.update();
      const uint32_t readUs = (uint32_t)(micros() - readStartUs);
      const ImuState sample = gImu.state();

      portENTER_CRITICAL(&gImuMux);
      ++gImuTelemetry.attempts;
      gImuTelemetry.totalReadUs += readUs;
      if (readUs > gImuTelemetry.maxReadUs) gImuTelemetry.maxReadUs = readUs;
      if (updated) {
        ++gImuTelemetry.successes;
        gImuSnapshot = sample;
        gImuLastGoodMs = millis();
        ++gImuSequence;
      } else {
        ++gImuTelemetry.failures;
      }
      portEXIT_CRITICAL(&gImuMux);

      // A long controller/bus stall is isolated from physics. Restart the I2C
      // peripheral after the call returns so the next IMU poll can recover.
      if (readUs > 100000UL) {
        Serial.printf("[imu] I2C read outlier %lu us; recovering SDA/SCL bus\n",
                      (unsigned long)readUs);
        recoverImuBus();
      }
    }
    vTaskDelay(1);
  }
}

void fluidTaskCode(void *parameter) {
  (void)parameter;
  uint32_t lastFluidMs = millis();
  uint32_t lastStatsMs = lastFluidMs;
  uint32_t lastProcessedImuSequence = 0;
  uint32_t fluidSteps = 0;
  uint32_t fluidBatches = 0;
  uint32_t snapshotPublishes = 0;
  uint32_t fluidTimeTotalUs = 0;
  uint32_t fluidTimeMaxUs = 0;
  uint32_t maxFluidGapMs = 0;
  float pendingImpulseX = 0.0f;
  float pendingImpulseY = 0.0f;
  uint32_t lastImpulseMs = 0;

  FluidControl control = {0.0f, 1.0f, 0.0f, 0.0f};
  for (;;) {
    uint32_t now = millis();

    // IMU polling runs in its own lower-priority task. Consume each fresh
    // sample once, so a slow I2C transaction cannot stall fluid simulation.
    ImuState imu;
    uint32_t imuSequence = 0;
    uint32_t imuSampleMs = 0;
    if (copyLatestImu(imu, imuSequence, imuSampleMs) && imu.valid &&
        imuSequence != lastProcessedImuSequence) {
      (void)imuSampleMs;
      lastProcessedImuSequence = imuSequence;
      const float rollRad = imu.rollDeg / RAD_TO_DEG_F;
      const float pitchRad = imu.pitchDeg / RAD_TO_DEG_F;

      const float rollScale = 1.0f + 0.35f * sinf(rollRad);
      control.gravityX = sinf(pitchRad);
      control.gravityY = cosf(pitchRad) * rollScale;

      const float pitchRate = imu.gyroDps[1];
      const float rollRate = -imu.gyroDps[2];
      const float angularSpeed = sqrtf(pitchRate * pitchRate + rollRate * rollRate);
      if (angularSpeed > MOTION_IMPULSE_THRESHOLD_DPS &&
          (uint32_t)(now - lastImpulseMs) >= MOTION_IMPULSE_INTERVAL_MS) {
        pendingImpulseX += pitchRate * MOTION_IMPULSE_GAIN;
        pendingImpulseY += rollRate * MOTION_IMPULSE_GAIN;
        if (pendingImpulseX > MOTION_IMPULSE_MAX) pendingImpulseX = MOTION_IMPULSE_MAX;
        if (pendingImpulseX < -MOTION_IMPULSE_MAX) pendingImpulseX = -MOTION_IMPULSE_MAX;
        if (pendingImpulseY > MOTION_IMPULSE_MAX) pendingImpulseY = MOTION_IMPULSE_MAX;
        if (pendingImpulseY < -MOTION_IMPULSE_MAX) pendingImpulseY = -MOTION_IMPULSE_MAX;
        lastImpulseMs = now;
      }
    }

    now = millis();
    const uint32_t fluidGapMs = (uint32_t)(now - lastFluidMs);
    if (fluidGapMs >= FLUID_STEP_MS) {
      if (fluidGapMs > maxFluidGapMs) maxFluidGapMs = fluidGapMs;

      uint32_t stepsDue = fluidGapMs / FLUID_STEP_MS;
      if (stepsDue > 4) stepsDue = 4;
      lastFluidMs = now;

      FluidControl stepControl = control;
      stepControl.impulseX = pendingImpulseX;
      stepControl.impulseY = pendingImpulseY;
      pendingImpulseX = 0.0f;
      pendingImpulseY = 0.0f;

      const uint32_t stepStartUs = micros();
      for (uint32_t step = 0; step < stepsDue; ++step) {
        gFluid.step(stepControl, (float)FLUID_STEP_MS * 0.001f);
        stepControl.impulseX = 0.0f;
        stepControl.impulseY = 0.0f;
      }
      uint8_t snapshotSlot = 0;
      if (gSnapshots.beginWrite(snapshotSlot)) {
        gFluid.makeSnapshot(gSnapshots.writeBuffer(snapshotSlot));
        gSnapshots.commitWrite(snapshotSlot);
        ++snapshotPublishes;
      }
      const uint32_t batchUs = (uint32_t)(micros() - stepStartUs);

      fluidSteps += stepsDue;
      ++fluidBatches;
      fluidTimeTotalUs += batchUs;
      if (batchUs > fluidTimeMaxUs) fluidTimeMaxUs = batchUs;
    }

    now = millis();
    if ((uint32_t)(now - lastStatsMs) >= 5000) {
      const uint32_t windowMs = now - lastStatsMs;
      const uint32_t physicsHzX10 = windowMs ? (fluidSteps * 10000UL) / windowMs : 0;
      const uint32_t publishHzX10 = windowMs ? (snapshotPublishes * 10000UL) / windowMs : 0;
      const uint32_t avgStepUs = fluidSteps ? fluidTimeTotalUs / fluidSteps : 0;
      const uint32_t avgBatchUs = fluidBatches ? fluidTimeTotalUs / fluidBatches : 0;
      const ImuTelemetry imuMetrics = takeImuTelemetry();
      const uint32_t imuHzX10 = windowMs ? (imuMetrics.successes * 10000UL) / windowMs : 0;
      const uint32_t avgImuReadUs = imuMetrics.attempts ?
                                    imuMetrics.totalReadUs / imuMetrics.attempts : 0;
      ImuState tilt;
      uint32_t tiltSequence = 0;
      uint32_t tiltSampleMs = 0;
      const bool imuReady = copyLatestImu(tilt, tiltSequence, tiltSampleMs);
      const uint32_t imuAgeMs = tiltSampleMs ? (uint32_t)(millis() - tiltSampleMs) : 0xFFFFFFFFUL;

      Serial.printf("[core0] physics %lu.%01lu Hz | publish %lu.%01lu Hz (avg %lu us/step, batch avg/max %lu/%lu us, gap max %lu ms), IMU %lu.%01lu Hz (ok %lu/%lu, fail %lu, age %lu ms, read avg/max %lu/%lu us)\n",
                    (unsigned long)(physicsHzX10 / 10), (unsigned long)(physicsHzX10 % 10),
                    (unsigned long)(publishHzX10 / 10), (unsigned long)(publishHzX10 % 10),
                    (unsigned long)avgStepUs, (unsigned long)avgBatchUs,
                    (unsigned long)fluidTimeMaxUs, (unsigned long)maxFluidGapMs,
                    (unsigned long)(imuHzX10 / 10), (unsigned long)(imuHzX10 % 10),
                    (unsigned long)imuMetrics.successes, (unsigned long)imuMetrics.attempts,
                    (unsigned long)imuMetrics.failures, (unsigned long)imuAgeMs,
                    (unsigned long)avgImuReadUs, (unsigned long)imuMetrics.maxReadUs);
      if (imuReady && tilt.valid) {
        const float accelMagnitude = sqrtf(tilt.accelG[0] * tilt.accelG[0] +
                                           tilt.accelG[1] * tilt.accelG[1] +
                                           tilt.accelG[2] * tilt.accelG[2]);
        Serial.printf("[tilt] IMU roll %+.2f deg | pitch %+.2f deg | accel %.2f g | gravity target %+.2f deg | bulk flow %.1f cells/s\n",
                      tilt.rollDeg, tilt.pitchDeg, accelMagnitude,
                      gFluid.gravityAngleDeg(), gFluid.bulkFlowSpeed());
      }
      lastStatsMs = now;
      fluidSteps = 0;
      fluidBatches = 0;
      snapshotPublishes = 0;
      fluidTimeTotalUs = 0;
      fluidTimeMaxUs = 0;
      maxFluidGapMs = 0;
    }

    vTaskDelay(1);
  }
}
void audioTaskCode(void *parameter) {
  (void)parameter;
  int32_t samples[AUDIO_SAMPLES * 2];
  for (;;) {
    // Deliberately lower priority than the fluid task; audio is capture-only.
    if (!gMicrophone.readBlock(samples, sizeof(samples))) vTaskDelay(1);
  }
}

static bool touchedNow() {
  const int touchValue = touchRead(HW_TOUCH);
  return touchValue > 0 && touchValue < 80;
}

static void updateTouch() {
  const uint32_t now = millis();
  const bool touched = touchedNow();
  if (touched) {
    if (!gTouchHeld) {
      gTouchHeld = true;
      gTouchTriggered = false;
      gTouchStartedMs = now;
    } else if (!gTouchTriggered && (uint32_t)(now - gTouchStartedMs) >= VIEW_HOLD_MS) {
      gTouchTriggered = true;
      gView = (FluidViewMode)(((uint8_t)gView + 1u) % VIEW_COUNT);
      Serial.printf("[view] %s\n", gView == VIEW_PARTICLES ? "Particles" :
                                         (gView == VIEW_WATER ? "Water" : "Ink"));
    }
  } else {
    gTouchHeld = false;
    gTouchTriggered = false;
  }
}

static void renderCurrentView(const FluidSnapshot &snapshot, int firstCellRow,
                              int cellRowCount) {
  switch (gView) {
    case VIEW_WATER:
      FluidRender::renderWater(snapshot, gFramebuffer, firstCellRow, cellRowCount);
      break;
    case VIEW_INK:
      FluidRender::renderInk(snapshot, gFramebuffer, firstCellRow, cellRowCount);
      break;
    case VIEW_PARTICLES:
    default:
      FluidRender::renderParticles(snapshot, gFramebuffer, firstCellRow, cellRowCount);
      break;
  }
}

static void renderFrame(const FluidSnapshot &snapshot, uint32_t &prepUs,
                        uint32_t &transferUs) {
  transferUs = 0;
  for (int firstCellRow = 0; firstCellRow < FLUID_GRID_H;
       firstCellRow += RENDER_BAND_CELL_ROWS) {
    int cellRowCount = FLUID_GRID_H - firstCellRow;
    if (cellRowCount > RENDER_BAND_CELL_ROWS) cellRowCount = RENDER_BAND_CELL_ROWS;

    uint32_t phaseStartUs = micros();
    renderCurrentView(snapshot, firstCellRow, cellRowCount);
    prepUs += (uint32_t)(micros() - phaseStartUs);

    const int y = firstCellRow * FLUID_CELL_PIXELS;
    const int height = cellRowCount * FLUID_CELL_PIXELS;
    phaseStartUs = micros();
    display->draw16bitRGBBitmap(0, y, gFramebuffer, SCREEN_W, height);
    transferUs += (uint32_t)(micros() - phaseStartUs);
  }
}
