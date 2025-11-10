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

// Enable to apply hysteresis around zero velocity
#ifdef SERVO_HYSTERESIS_ENABLE
  // Thresholds in encoder counts/sec
  #ifndef SERVO_HYST_ENTER_CPS
    #define SERVO_HYST_ENTER_CPS 20.0f   // must exceed this to LEAVE zero (e.g., ~1/5 of sidereal speed of 92 counts / sec)
  #endif
  #ifndef SERVO_HYST_EXIT_CPS
    #define SERVO_HYST_EXIT_CPS 10.0f    // drop below this to RETURN to zero (half of the above)
  #endif
  // Reset hysteresis on direction changes while "moving" (forces pass-through zero)
  #ifndef SERVO_HYST_RESET_ON_DIR_FLIP
    #define SERVO_HYST_RESET_ON_DIR_FLIP 1
  #endif
#endif

// Stiction breakaway kick (ENABLE by defining SERVO_STICTION_KICK)
#ifdef SERVO_STICTION_KICK
  #ifndef SERVO_STICTION_KICK_MS
    // duration of kick after zero->nonzero or direction flip
    // depends on the mechanical time constant and electrical time constant of motors
    #define SERVO_STICTION_KICK_MS 20
  #endif

  #ifndef SERVO_STICTION_KICK_PERCENT_MULTIPLIER
    #define SERVO_STICTION_KICK_PERCENT_MULTIPLIER 3.50f // means 3.5x
  #endif
#endif

// Nonlinear concave mapping near zero (further help kick)
#ifdef SERVO_NONLINEAR_ENABLE
  #ifndef SERVO_NONLINEAR_GAMMA
    #define SERVO_NONLINEAR_GAMMA 0.65f    // concave (0.5–0.8 is a good range)
  #endif
  #ifndef SERVO_NONLINEAR_KNEE_CPS
    #define SERVO_NONLINEAR_KNEE_CPS 25.0f // tune
  #endif
  // Relative knee: if >0, knee = velocityMax * percent; otherwise use absolute CPS
  #ifndef SERVO_NONLINEAR_KNEE_PERCENT
    #define SERVO_NONLINEAR_KNEE_PERCENT 0.1f   // e.g., 0.10f for 10% of vmax; 0 => use absolute CPS
  #endif
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
      #ifdef SERVO_STICTION_KICK
        kickAllowedByMode = state;
      #else
        (void) state;
      #endif
    }

  protected:
    // convert from encoder counts per second to analogWriteRange units to roughly match velocity
    // we expect a linear scaling for the motors (maybe in the future we can be smarter?)
    // uses cached min/max counts and a precomputed gain
    // only float ops since usually there is no double FPU
    long toAnalogRange(float velocity) {
      // Update caches only if inputs changed
      recomputeScalingIfNeeded();

      // Extract sign and work with magnitude
      int sign = (velocity < 0.0f) ? -1 : +1;
      float vAbs = (velocity < 0.0f) ? -velocity : velocity;

      if (vAbs == 0.0f || velocityMaxCached <= 0.0f) {
        return handleZeroVelocity(); // early out (also resets state)
      }

      // Clamp to velocityMax to avoid over-range math.
      if (vAbs > velocityMaxCached) vAbs = velocityMaxCached;

      #ifdef SERVO_HYSTERESIS_ENABLE
        // Hysteresis around zero: may hold at zero or latch direction
        int hSign = applyHysteresis(vAbs, sign);
        if (hSign == 0) {
          // held/snapped to zero
          #ifdef SERVO_SIGMA_DELTA_DITHERING
            sigmaDelta.reset(); // also reset when snapping back to zero
          #endif
          return 0;
        }
        sign = hSign; // use latched sign while moving
      #endif

      // Nonlinear near-zero mapping (concave) or linear above knee
      float countsF = applyNonLinearMapping(vAbs);

      // Quantize to integer counts
      #ifdef SERVO_SIGMA_DELTA_DITHERING
        // Dither the floating counts to an integer so the time-average equals countsF.
        // Use 0..countsMaxCached as the dither bounds (not countsMinCached),
        // then apply the minimum/kick below.
        long power = (long) sigmaDelta.ditherCounts(countsF, 0, countsMaxCached);
      #else
        long power = (long) lroundf(countsF);
      #endif

      #ifdef SERVO_STICTION_KICK
        const uint32_t now = millis();
        return applyStictionKick(power, sign, now);
      #else
        // No kick feature: just apply sustaining minimum and clamp
        if (power > 0 && power < countsMinCached) power = countsMinCached;
        if (power > countsMaxCached) power = countsMaxCached;
        power *= (sign >= 0) ? +1 : -1;
        return power;
      #endif
    }

    // motor control update
    virtual void pwmUpdate(long power) { }

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

        VF("MSG:"); V(axisPrefix); VF("pwmMin="); V(pwmMinPctCached); VLF(" %");
        VF("MSG:"); V(axisPrefix); VF("pwmMax="); V(pwmMaxPctCached); VLF(" %");
        VF("MSG:"); V(axisPrefix); VF("Vmax="); V(velocityMaxCached); VLF(" steps/s");
        VF("MSG:"); V(axisPrefix); VF("pwm units="); V(analogMaxCached); VLF(" pwm Units");

        // Convert % to float counts once, then round once.
        const float minCountsF = (pwmMinPctCached * 0.01f) * (float)analogMaxCached;
        const float maxCountsF = (pwmMaxPctCached * 0.01f) * (float)analogMaxCached;

        countsMinCached = (int32_t)lroundf(minCountsF);
        countsMaxCached = (int32_t)lroundf(maxCountsF);
        if (countsMaxCached < countsMinCached) countsMaxCached = countsMinCached; // safety

        // gain: map velocity 0..Vmax → counts 0..countsMaxCached (no offset here)
        velToCountsGain = (velocityMaxCached > 0.0f) ? ((float)countsMaxCached / velocityMaxCached) : 0.0f;
        VF("MSG:"); V(axisPrefix); VF("velToCountsGain="); V(velToCountsGain); VLF(" (0..Vmax -> 0..pwm units)");

        #ifdef SERVO_STICTION_KICK
          // Breakaway counts = max(min, min * EXTRA_PCT), clamped to max
          float breakF = fmaxf(minCountsF, minCountsF * SERVO_STICTION_KICK_PERCENT_MULTIPLIER);
          int32_t breakCounts = (int32_t)lroundf(breakF);
          if (breakCounts > countsMaxCached) breakCounts = countsMaxCached;
          countsBreakCached = breakCounts;
          VF("MSG:"); V(axisPrefix); VF("breakaway counts="); V(countsBreakCached); VLF("");
        #endif
      }
    }

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

// ========================= Helper Implementations ================================

inline long ServoDcDriver::handleZeroVelocity() {
  #ifdef SERVO_HYSTERESIS_ENABLE
    zeroHoldSign = 0; // ensure immediate exit and clear latch on exact zero
  #endif

  #ifdef SERVO_SIGMA_DELTA_DITHERING
    sigmaDelta.reset(); // reset dithering residue when output is zero
  #endif

  #ifdef SERVO_STICTION_KICK
    // remember we are outputting zero
    lastPowerCounts = 0;
    lastSign = 0;
    kickUntilMs = 0;
  #endif

  return 0;
}

inline int ServoDcDriver::applyHysteresis(float vAbs, int sign) {
  #ifndef SERVO_HYSTERESIS_ENABLE
    (void)vAbs; (void)sign;
    return (sign >= 0) ? +1 : -1;
  #else
    const int reqSign = (sign >= 0) ? +1 : -1;

    if (zeroHoldSign == 0) {
      // currently at zero: must exceed enter threshold to start moving
      if (vAbs < SERVO_HYST_ENTER_CPS) return 0;
      zeroHoldSign = reqSign; // latch direction
      return zeroHoldSign;
    }

    // currently moving
    #if SERVO_HYST_RESET_ON_DIR_FLIP
      // Alternative: maybe we want to reset hysteresis on direction changes?
      if (reqSign != zeroHoldSign) {
        // Direction changed - reset hysteresis?
        zeroHoldSign = 0;
        return 0;
      }
    #else
      // allow direction change if command flips while above thresholds
      if (reqSign != zeroHoldSign) zeroHoldSign = reqSign;
    #endif

    // if we drop below exit threshold, snap back to zero
    if (vAbs < SERVO_HYST_EXIT_CPS) {
      zeroHoldSign = 0;
      return 0;
    }

    // use latched direction while moving
    return (zeroHoldSign < 0) ? -1 : +1;
  #endif
}

inline float ServoDcDriver::applyNonLinearMapping(float vAbs) {
  if (vAbs <= 0.0f) return 0.0f;

  // Default linear result
  const float linear = vAbs * velToCountsGain;

  // --- Nonlinear near-zero mapping (compiled out if disabled) ---
  #ifdef SERVO_NONLINEAR_ENABLE
    const float gamma = SERVO_NONLINEAR_GAMMA;
    if (gamma > 0.0f && gamma < 1.0f) {
      // Compute knee ONLY if we might use it
      float v_knee =
        (SERVO_NONLINEAR_KNEE_PERCENT > 0.0f && velocityMaxCached > 0.0f)
          ? (velocityMaxCached * SERVO_NONLINEAR_KNEE_PERCENT)
          : SERVO_NONLINEAR_KNEE_CPS;

      if (v_knee > 1e-6f && vAbs < v_knee) {
        // t in (0,1); avoid denormals/zero
        float invKnee = 1.0f / v_knee;
        float t = vAbs * invKnee;
        if (t < 1e-6f) t = 1e-6f;

        // scale = t^(gamma-1)
        const float gm1 = gamma - 1.0f;
        float scale = expf(gm1 * logf(t));   // or powf(t, gm1)

        return velToCountsGain * vAbs * scale;
      }
    }
  #endif

  // Linear when nonlinear is disabled, gamma invalid, above knee, or knee invalid
  return linear;
}

inline long ServoDcDriver::applyStictionKick(long power, int sign, uint32_t now) {
  // No kicking
  #ifndef SERVO_STICTION_KICK
    (void)sign; (void)now;
    // Sustain min, clamp, sign
    if (power > 0 && power < countsMinCached) power = countsMinCached;
    if (power > countsMaxCached) power = countsMaxCached;
    power *= (sign >= 0) ? +1 : -1;
    return power;
  #else
    // We are kicking
    const int reqSign = (sign >= 0) ? +1 : -1;

    if (kickAllowedByMode) {
      // Start a stiction kick ?
      // We kick when we are at rest (lastPowerCounts == 0) and receive a nonzero request,
      // or when we flip direction.
      bool leaveZero   = (lastPowerCounts == 0) && (power > 0);
      bool dirFlip     = (lastSign != 0 && reqSign != lastSign);

      if (leaveZero || dirFlip) {
        kickUntilMs = now + SERVO_STICTION_KICK_MS;
        #ifdef SERVO_SIGMA_DELTA_DITHERING
          // reset dithering residue on flips to avoid bias
          if (dirFlip) sigmaDelta.reset();
        #endif
      }

      // If we are within the kick window, enforce the breakaway minimum
      if (kickUntilMs != 0 && (int32_t)(now - kickUntilMs) < 0) {
        if (power > 0 && power < countsBreakCached) power = countsBreakCached;
      } else {
        kickUntilMs = 0; // window expired
        if (power > 0 && power < countsMinCached) power = countsMinCached;
      }
    } else {
      // Not in tracking mode: no kick, just sustaining minimum
      kickUntilMs = 0;
      if (power > 0 && power < countsMinCached) power = countsMinCached;
    }

    // Final clamp and sign/update
    if (power > countsMaxCached) power = countsMaxCached;
    power *= reqSign;
    lastPowerCounts = power;
    lastSign        = (power > 0) ? +1 : (power < 0 ? -1 : 0);
    return power;
  #endif
}

#endif // driver present
