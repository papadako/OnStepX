// -----------------------------------------------------------------------------------
// axis servo DC motor driver

#include "DcServoDriver.h"

#if defined(SERVO_PE_PRESENT) || defined(SERVO_EE_PRESENT) || defined(SERVO_TMC2130_DC_PRESENT) || defined(SERVO_TMC5160_DC_PRESENT)

#include "../../../../gpioEx/GpioEx.h"

ServoDcDriver::ServoDcDriver(uint8_t axisNumber, const ServoPins *Pins, const ServoSettings *Settings, float pwmMinimum, float pwmMaximum)
                            :ServoDriver(axisNumber, Pins, Settings) {
  if (axisNumber < 1 || axisNumber > 9) return;
  this->pwmMinimum.valueDefault = pwmMinimum;
  this->pwmMaximum.valueDefault = pwmMaximum;
  // initialize caches up front
  recomputeScalingIfNeeded();
}

float ServoDcDriver::setMotorVelocity(float velocity)  {
  velocity = ServoDriver::setMotorVelocity(velocity);

  pwmUpdate(labs(toAnalogRange(velocity)));

  return velocity;
}

// convert from encoder counts per second to analogWriteRange units to roughly match velocity
// we expect a linear scaling for the motors (maybe in the future we can be smarter?)
// uses cached min/max counts and a precomputed gain
// only float ops since usually there is no double FPU
long ServoDcDriver::toAnalogRange(float velocity) {
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
void ServoDcDriver::pwmUpdate(long power) {
  // default no-op; platform-specific drivers override this
}

// ========================= Helper Implementations ================================

long ServoDcDriver::handleZeroVelocity() {
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

int ServoDcDriver::applyHysteresis(float vAbs, int sign) {
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

float ServoDcDriver::applyNonLinearMapping(float vAbs) {
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

long ServoDcDriver::applyStictionKick(long power, int sign, uint32_t now) {
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

void ServoDcDriver::recomputeScalingIfNeeded() {
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

#endif
