// -----------------------------------------------------------------------------------
// axis servo DC motor driver (optimized, same public API)
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

#include "../ServoDriver.h"
#include "SigmaDeltaDither.h"

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

  protected:
    // convert from encoder counts per second to analogWriteRange units to roughly match velocity
    // FAST PATH: uses cached min/max counts and a precomputed gain; only float ops since usually there is no double FPU
    long toAnalogRange(float velocity) {
      // Update caches only if inputs changed
      recomputeScalingIfNeeded();

      // Extract sign and work with magnitude (direction handled elsewhere).
      int sign = 1;
      if (velocity < 0.0F) { velocity = -velocity; sign = -1; }

      if (velocity == 0.0F || velocityMaxCached <= 0.0f) {
        // Zero torque: also reset dithering so no stale fraction is carried.
        #if defined(SERVO_SIGMA_DELTA_DITHERING) && SERVO_SIGMA_DELTA_DITHERING != OFF
          sd_pwm_.reset();
        #endif
        return 0; // early out
      }

      // Clamp to velocityMax to avoid over-range math.
      float vAbs = velocity;
      if (vAbs > velocityMaxCached) vAbs = velocityMaxCached;

      // Linear map: counts = countsMin + vAbs * gain
      // fmaf keeps one rounding and is very fast on some FPUS(M7).
      float countsF = fmaf(vAbs, velToCountsGain, (float)countsMinCached);

      // Quantize: sigma–delta (preferred) or round
      long power;
      #if defined(SERVO_SIGMA_DELTA_DITHERING) && SERVO_SIGMA_DELTA_DITHERING != OFF
        power = (long)sd_pwm_.dither_counts(countsF, countsMinCached, countsMaxCached);
      #else
        // Single float→int rounding at the end
        power = (long)lroundf(countsF);
      #endif

      return sign < 0 ? -power : power;
    }

    // motor control update
    virtual void pwmUpdate(long power) { }

    long analogWriteRange = SERVO_ANALOG_WRITE_RANGE;

    // runtime adjustable settings (percent 0..100)
    AxisParameter pwmMinimum = {NAN, NAN, NAN, 0.0, 100.0, AXP_FLOAT_IMMEDIATE, AXPN_MIN_PWR};
    AxisParameter pwmMaximum = {NAN, NAN, NAN, 0.0, 100.0, AXP_FLOAT_IMMEDIATE, AXPN_MAX_PWR};

    const int numParameters = 3;
    AxisParameter* parameter[4] = {&invalid, &acceleration, &pwmMinimum, &pwmMaximum};

  private:
    // -------- Cached scaling (recomputed only when inputs change) ----------
    // detect changes to: pwmMinimum.value, pwmMaximum.value, velocityMax, analogWriteRange.
    float   pwmMinPctCached   = -1.0f;
    float   pwmMaxPctCached   = -1.0f;
    float   velocityMaxCached = 0.0f;
    long    analogMaxCached   = -1;

    int32_t countsMinCached   = 0;      // integer min duty in counts
    int32_t countsMaxCached   = 0;      // integer max duty in counts
    float   velToCountsGain   = 0.0f;   // counts per (encoder count/s)

    // Recompute cache if any dependency changed.
    inline void recomputeScalingIfNeeded() {
      const long analogMaxNow = (analogWriteRange - 1);

      if (pwmMinPctCached   != pwmMinimum.value ||
          pwmMaxPctCached   != pwmMaximum.value ||
          velocityMaxCached != velocityMax ||
          analogMaxCached   != analogMaxNow)
      {
        pwmMinPctCached   = pwmMinimum.value;   // percent 0..100
        pwmMaxPctCached   = pwmMaximum.value;   // percent 0..100
        velocityMaxCached = velocityMax;
        analogMaxCached   = analogMaxNow;

        // Convert % to float counts once, then round once.
        const float minCountsF = (pwmMinPctCached * 0.01f) * (float)analogMaxCached;
        const float maxCountsF = (pwmMaxPctCached * 0.01f) * (float)analogMaxCached;

        countsMinCached = (int32_t)lroundf(minCountsF);
        countsMaxCached = (int32_t)lroundf(maxCountsF);
        if (countsMaxCached < countsMinCached) countsMaxCached = countsMinCached; // safety

        const int span = (int)(countsMaxCached - countsMinCached);
        velToCountsGain = (velocityMaxCached > 0.0f) ? ((float)span / velocityMaxCached) : 0.0f;
      }
    }

    #if defined(SERVO_SIGMA_DELTA_DITHERING) && SERVO_SIGMA_DELTA_DITHERING != OFF
      SigmaDeltaDither sd_pwm_;     // per-axis sigma–delta state
    #endif
};

#endif
