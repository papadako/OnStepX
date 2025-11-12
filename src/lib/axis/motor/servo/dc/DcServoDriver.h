// -----------------------------------------------------------------------------------
// axis servo DC motor driver
#pragma once

#include <Arduino.h>
#include <math.h>            // fabsf, lroundf, fmaf
#include "../../../../../Common.h"

#if defined(SERVO_PE_PRESENT) || defined(SERVO_EE_PRESENT) || defined(SERVO_TMC2130_DC_PRESENT) || defined(SERVO_TMC5160_DC_PRESENT)

#ifndef ANALOG_WRITE_RANGE
  #define ANALOG_WRITE_RANGE 255
#endif
#ifndef SERVO_ANALOG_WRITE_RANGE
  #define SERVO_ANALOG_WRITE_RANGE ANALOG_WRITE_RANGE
#endif

#ifdef SERVO_SIGMA_DELTA_DITHERING
  #include "SigmaDeltaDither.h"
#endif

#include "../ServoDriver.h"

class ServoDcDriver : public ServoDriver {
  public:
    // constructor
    ServoDcDriver(uint8_t axisNumber, const ServoPins *Pins, const ServoSettings *Settings, float pwmMinimum, float pwmMaximum);

    // returns the number of axis parameters
    uint8_t getParameterCount() { return numParameters; }

    // returns the specified axis parameter
    AxisParameter* getParameter(uint8_t number) { if (number > numParameters) return &invalid; else return parameter[number]; }

    // set motor velocity
    // \param velocity as needed to reach the target position, in encoder counts per second
    // \returns velocity in effect, in encoder counts per second
    float setMotorVelocity(float velocity);

    void setTrackingMode(bool state) {
      ServoDriver::setTrackingMode(state);   // update base trackingMode
      #ifdef SERVO_STICTION_KICK
        kickAllowedByMode = state;
      #endif
    }

  protected:
    // convert from encoder counts per second to analogWriteRange units to roughly match velocity
    // we expect a linear scaling for the motors (maybe in the future we can be smarter?)
    // uses cached min/max counts and a precomputed gain
    // only float ops since usually there is no double FPU
    long toAnalogRange(float velocity);

    // motor control update
    virtual void pwmUpdate(long power);

    long analogWriteRange = SERVO_ANALOG_WRITE_RANGE;

    // runtime adjustable settings (percent 0..100)
    AxisParameter pwmMinimum = {NAN, NAN, NAN, 0.0, 100.0, AXP_FLOAT_IMMEDIATE, AXPN_MIN_PWR};
    AxisParameter pwmMaximum = {NAN, NAN, NAN, 0.0, 100.0, AXP_FLOAT_IMMEDIATE, AXPN_MAX_PWR};

    const int numParameters = 3;
    AxisParameter* parameter[4] = {&invalid, &acceleration, &pwmMinimum, &pwmMaximum};

    // --- helper API (split from toAnalogRange) -------------------------------------
    // Reset state when commanded velocity is zero; returns 0
    long handleZeroVelocity();

    // Hysteresis logic around zero; returns 0 to hold at zero or ±1 for latched sign
    int  applyHysteresis(float vAbs, int sign);

    // Nonlinear concave mapping near zero; returns float "counts" before quantization
    float applyNonLinearMapping(float vAbs);

    // Stiction kick/minimum enforcement/clamp/sign; returns final signed integer counts
    long applyStictionKick(long power, int sign, uint32_t now);

  private:
    // Cached scaling (recomputed only when inputs change)
    // Detect changes to pwmMinimum.value, pwmMaximum.value, velocityMax, analogWriteRange.
    float   pwmMinPctCached   = -1.0f;
    float   pwmMaxPctCached   = -1.0f;
    float   velocityMaxCached = 0.0f;
    long    analogMaxCached   = -1;

    int32_t countsMinCached   = 0;      // integer min duty in counts
    int32_t countsMaxCached   = 0;      // integer max duty in counts
    float   velToCountsGain   = 0.0f;   // counts per (encoder count/s)

    void recomputeScalingIfNeeded();

    #ifdef SERVO_HYSTERESIS_ENABLE
      int8_t zeroHoldSign = 0; // 0: at zero; +1 / -1: direction while "moving"
    #endif

    #ifdef SERVO_STICTION_KICK
      // --- breakaway kick state ---
      int32_t  lastPowerCounts = 0;   // last output after clamping (signed)
      int8_t   lastSign        = 0;   // -1, 0, +1 of lastPowerCounts
      uint32_t kickUntilMs     = 0;   // time until which we keep kicking
      int32_t  countsBreakCached = 0; // breakaway minimum in counts (recomputed with scaling)
      bool kickAllowedByMode = false; // set via setTrackingMode()
    #endif

    #ifdef SERVO_SIGMA_DELTA_DITHERING
      SigmaDeltaDither sigmaDelta;
    #endif
};

#endif // driver present
