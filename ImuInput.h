#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <FastIMU.h>
#include <math.h>
#include "FluidTypes.h"

class ImuInput {
public:
  bool begin() {
    present_ = false;
    address_ = 0;
    state_ = {};
    filterInitialized_ = false;
    accelReferenceReady_ = false;
    lastFilterUs_ = 0;
    lastFastTimestampUs_ = 0;
    filteredRoll_ = 0.0f;
    filteredPitch_ = 0.0f;
    for (int axis = 0; axis < 3; ++axis) gyroBiasDps_[axis] = 0.0f;

    // Select the acknowledged I2C address; FastIMU then verifies WHO_AM_I as
    // part of its MPU6500 initialization.
    const uint8_t candidates[2] = {0x68, 0x69};
    for (uint8_t i = 0; i < 2; ++i) {
      Wire.beginTransmission(candidates[i]);
      if (Wire.endTransmission(true) == 0) {
        address_ = candidates[i];
        break;
      }
    }
    if (address_ == 0) return false;

    calData calibration = {};
    const int initResult = fastImu_.init(calibration, address_);
    if (initResult != 0) {
      address_ = 0;
      return false;
    }

    // Preserve the existing firmware's sensor scales while using FastIMU's
    // MPU6500 register setup and data-ready handling.
    const bool configured = fastImu_.setAccelRange(2) == 0 &&
                            fastImu_.setGyroRange(250) == 0 &&
                            fastImu_.setGyroODR(200) >= 100;
    if (!configured) {
      address_ = 0;
      return false;
    }

    present_ = true;
    calibrateGyroBias();  // Keep the device still during startup calibration.
    state_.valid = false;
    return true;
  }

  bool update() {
    if (!present_) return false;

    AccelData accel = {};
    GyroData gyro = {};
    if (!readFreshSensorSample(accel, gyro)) return false;

    const float accelAxes[3] = {accel.accelX, accel.accelY, accel.accelZ};
    const float gyroAxes[3] = {gyro.gyroX, gyro.gyroY, gyro.gyroZ};
    for (int axis = 0; axis < 3; ++axis) {
      state_.accelG[axis] = accelAxes[axis];
      state_.accelRaw[axis] = (int16_t)lrintf(accelAxes[axis] * ACCEL_LSB_PER_G);
      state_.gyroRaw[axis] = (int16_t)lrintf(gyroAxes[axis] * GYRO_LSB_PER_DPS);
      float rate = gyroAxes[axis] - gyroBiasDps_[axis];
      if (fabsf(rate) < GYRO_DEADBAND_DPS) rate = 0.0f;
      state_.gyroDps[axis] = rate;
    }
    state_.temperatureC = fastImu_.getTemp();
    state_.valid = true;
    updateComplementaryFilter();
    return true;
  }

  const ImuState &state() const { return state_; }
  bool present() const { return present_; }
  bool accelReferenceReady() const { return accelReferenceReady_; }
  uint8_t address() const { return address_; }

  const char *modelName() const { return "MPU6500 / FastIMU"; }

private:
  static const uint32_t MAX_ACCEPTED_READ_US = 100000UL;
  static const uint8_t CALIBRATION_SAMPLE_TARGET = 80;

  MPU6500 fastImu_;
  bool present_ = false;
  uint8_t address_ = 0;
  ImuState state_ = {};
  float gyroBiasDps_[3] = {0.0f, 0.0f, 0.0f};
  uint32_t lastFastTimestampUs_ = 0;

  bool filterInitialized_ = false;
  bool accelReferenceReady_ = false;
  uint32_t lastFilterUs_ = 0;
  float accelReference_[3] = {1.0f, 0.0f, 0.0f};
  float filteredRoll_ = 0.0f;
  float filteredPitch_ = 0.0f;

  static bool sampleIsSane(const AccelData &accel, const GyroData &gyro) {
    const float values[6] = {
      accel.accelX, accel.accelY, accel.accelZ,
      gyro.gyroX, gyro.gyroY, gyro.gyroZ
    };
    for (uint8_t i = 0; i < 6; ++i) {
      if (!isfinite(values[i])) return false;
    }
    if (fabsf(accel.accelX) > 4.0f || fabsf(accel.accelY) > 4.0f ||
        fabsf(accel.accelZ) > 4.0f || fabsf(gyro.gyroX) > 1000.0f ||
        fabsf(gyro.gyroY) > 1000.0f || fabsf(gyro.gyroZ) > 1000.0f) return false;
    const float magnitudeSq = accel.accelX * accel.accelX +
                              accel.accelY * accel.accelY +
                              accel.accelZ * accel.accelZ;
    return magnitudeSq > 0.0225f && magnitudeSq < 16.0f;
  }

  bool readFreshSensorSample(AccelData &accel, GyroData &gyro) {
    const uint32_t startUs = micros();
    fastImu_.update();
    const uint32_t readUs = (uint32_t)(micros() - startUs);
    fastImu_.getAccel(&accel);
    fastImu_.getGyro(&gyro);

    if (accel.timestamp == 0 || accel.timestamp == lastFastTimestampUs_) {
      if (readUs > MAX_ACCEPTED_READ_US) lastFilterUs_ = micros();
      return false;
    }
    // Consume the timestamp even for a rejected/slow transfer so the same
    // questionable library buffer is never published twice.
    lastFastTimestampUs_ = accel.timestamp;
    if (readUs > MAX_ACCEPTED_READ_US) {
      lastFilterUs_ = micros();
      return false;
    }
    return sampleIsSane(accel, gyro);
  }

  void calibrateGyroBias() {
    uint8_t samples = 0;
    float gyroSums[3] = {0.0f, 0.0f, 0.0f};
    float accelSums[3] = {0.0f, 0.0f, 0.0f};
    const uint32_t startMs = millis();

    while (samples < CALIBRATION_SAMPLE_TARGET &&
           (uint32_t)(millis() - startMs) < 2500UL) {
      AccelData accel = {};
      GyroData gyro = {};
      if (readFreshSensorSample(accel, gyro)) {
        accelSums[0] += accel.accelX;
        accelSums[1] += accel.accelY;
        accelSums[2] += accel.accelZ;
        gyroSums[0] += gyro.gyroX;
        gyroSums[1] += gyro.gyroY;
        gyroSums[2] += gyro.gyroZ;
        ++samples;
      }
      delay(1);
    }

    if (samples == 0) return;

    float referenceMagnitudeSq = 0.0f;
    for (int axis = 0; axis < 3; ++axis) {
      gyroBiasDps_[axis] = gyroSums[axis] / (float)samples;
      accelReference_[axis] = accelSums[axis] / (float)samples;
      referenceMagnitudeSq += accelReference_[axis] * accelReference_[axis];
    }

    const float referenceMagnitude = sqrtf(referenceMagnitudeSq);
    if (referenceMagnitude > 0.5f && referenceMagnitude < 1.5f) {
      for (int axis = 0; axis < 3; ++axis) accelReference_[axis] /= referenceMagnitude;
      accelReferenceReady_ = true;
    }
    lastFilterUs_ = 0;
    filterInitialized_ = false;
  }

  void updateComplementaryFilter() {
    // Installed orientation, looking at the screen: sensor X=up, Y=depth, Z=left.
    float accelMagnitudeSq = 0.0f;
    for (int axis = 0; axis < 3; ++axis)
      accelMagnitudeSq += state_.accelG[axis] * state_.accelG[axis];
    const float accelMagnitude = sqrtf(accelMagnitudeSq);

    if (!accelReferenceReady_ && accelMagnitude > 0.5f) {
      for (int axis = 0; axis < 3; ++axis)
        accelReference_[axis] = state_.accelG[axis] / accelMagnitude;
      accelReferenceReady_ = true;
    }

    float accelRoll = 0.0f;
    float accelPitch = 0.0f;
    float accelTrust = 0.0f;
    if (accelReferenceReady_ && accelMagnitude > 0.1f) {
      const float current[3] = {
        state_.accelG[0] / accelMagnitude,
        state_.accelG[1] / accelMagnitude,
        state_.accelG[2] / accelMagnitude
      };
      const float crossX = accelReference_[1] * current[2] - accelReference_[2] * current[1];
      const float crossY = accelReference_[2] * current[0] - accelReference_[0] * current[2];
      const float crossZ = accelReference_[0] * current[1] - accelReference_[1] * current[0];
      const float crossMagnitude = sqrtf(crossX * crossX + crossY * crossY + crossZ * crossZ);
      float dot = accelReference_[0] * current[0] +
                  accelReference_[1] * current[1] +
                  accelReference_[2] * current[2];
      if (dot > 1.0f) dot = 1.0f;
      if (dot < -1.0f) dot = -1.0f;

      // The shortest gravity-vector rotation is resolved onto the established
      // roll/Z and pitch/Y axes. This makes the calibrated table pose exactly 0°.
      if (crossMagnitude > 0.00001f) {
        const float angleRad = atan2f(crossMagnitude, dot);
        const float axisScale = angleRad / crossMagnitude;
        accelRoll = crossZ * axisScale * RAD_TO_DEG_F;
        accelPitch = -crossY * axisScale * RAD_TO_DEG_F;
        wrap180(accelRoll);
        wrap180(accelPitch);
      }

      // Trust gravity most when its magnitude is near 1 g and the device is
      // not rotating rapidly; during a flick, rely more heavily on the gyro.
      // Keep this weight active even at zero relative angle, so a stationary
      // return to the reference pose can remove accumulated gyro error.
      if (dot > -0.95f) {
        float magnitudeTrust = 1.0f - fabsf(accelMagnitude - 1.0f) / 0.18f;
        if (magnitudeTrust < 0.0f) magnitudeTrust = 0.0f;
        if (magnitudeTrust > 1.0f) magnitudeTrust = 1.0f;
        const float gyroRate = sqrtf(state_.gyroDps[0] * state_.gyroDps[0] +
                                     state_.gyroDps[1] * state_.gyroDps[1] +
                                     state_.gyroDps[2] * state_.gyroDps[2]);
        float motionTrust = (120.0f - gyroRate) / 60.0f;
        if (motionTrust < 0.0f) motionTrust = 0.0f;
        if (motionTrust > 1.0f) motionTrust = 1.0f;
        accelTrust = magnitudeTrust * motionTrust;
      }
    }

    const uint32_t nowUs = micros();
    float dt = 0.0f;
    if (lastFilterUs_ != 0) {
      dt = (uint32_t)(nowUs - lastFilterUs_) / 1000000.0f;
      if (dt > 0.25f) dt = 0.25f;
    }
    lastFilterUs_ = nowUs;

    if (!filterInitialized_) {
      filteredRoll_ = accelRoll;
      filteredPitch_ = accelPitch;
      state_.yawRelativeDeg = 0.0f;
      filterInitialized_ = true;
    } else if (dt > 0.0f) {
      const float alpha = 0.98f;
      const float accelWeight = (1.0f - alpha) * accelTrust;
      const float gyroWeight = 1.0f - accelWeight;
      // Preserve the installed sign convention: roll uses -gyro Z; pitch +gyro Y.
      // Keep the gyro prediction unwrapped. Gravity angles repeat every 360°,
      // so select the equivalent accel angle nearest the continuous prediction.
      const float predictedRoll = filteredRoll_ - state_.gyroDps[2] * dt;
      const float predictedPitch = filteredPitch_ + state_.gyroDps[1] * dt;
      const float continuousAccelRoll = nearestEquivalent(accelRoll, predictedRoll);
      const float continuousAccelPitch = nearestEquivalent(accelPitch, predictedPitch);
      filteredRoll_ = gyroWeight * predictedRoll + accelWeight * continuousAccelRoll;
      filteredPitch_ = gyroWeight * predictedPitch + accelWeight * continuousAccelPitch;

      state_.yawRelativeDeg += state_.gyroDps[0] * dt;
      while (state_.yawRelativeDeg >= 360.0f) state_.yawRelativeDeg -= 360.0f;
      while (state_.yawRelativeDeg < 0.0f) state_.yawRelativeDeg += 360.0f;
    }

    state_.rollDeg = filteredRoll_;
    state_.pitchDeg = filteredPitch_;
  }

  static float nearestEquivalent(float wrappedAngle, float referenceAngle) {
    const float turns = floorf((referenceAngle - wrappedAngle + 180.0f) / 360.0f);
    return wrappedAngle + turns * 360.0f;
  }

  static void wrap180(float &angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
  }
};
