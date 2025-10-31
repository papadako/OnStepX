// -----------------------------------------------------------------------------------
// exponential moving average (EMA) filter
// first-order low-pass filter with tunable time constant
// α computed from PID period
// reduce encoder noise (quantization + ISR jitter) at sidereal
// no overshoot artifacts
// implements both position and velocity filtering

#include "EMAFilter.h"

#if AXIS1_SERVO_FLTR == EMA || AXIS2_SERVO_FLTR == EMA || AXIS3_SERVO_FLTR == EMA || \
    AXIS4_SERVO_FLTR == EMA || AXIS5_SERVO_FLTR == EMA || AXIS6_SERVO_FLTR == EMA || \
    AXIS7_SERVO_FLTR == EMA || AXIS8_SERVO_FLTR == EMA || AXIS9_SERVO_FLTR == EMA

EMAFilter::EMAFilter(float tauTracking, float tauSlewing, float dt) {
  setTimeConstants(tauTracking, tauSlewing, dt);
}

// helper to compute alpha from tau
float EMAFilter::alphaFromTau(float tau, float dt) {
  if (dt <= 0.0f)   return 0.0f;
  if (tau <= 0.0f)  return 1.0f;
  float a = dt / (tau + dt);
  return (a < 0.0f) ? 0.0f : (a > 1.0f ? 1.0f : a);
}

void EMAFilter::setTimeConstants(float tauTracking, float tauSlewing, float dt) {
  alphaTracking = alphaFromTau(tauTracking, dt);
  alphaSlewing  = alphaFromTau(tauSlewing,  dt);
}

// ---------------------------
// POSITION FILTER PATH
// ---------------------------
// encoderCounts is the measurement, output filtered counts as long.
long EMAFilter::update(long encoderCounts, long motorCounts, bool isTracking) {
  (void)motorCounts;

  const float x = (float)encoderCounts;
  if (!posInit) {
    yPos   = x;
    posInit = true;
    return encoderCounts;
  }

  #if EMA_DISABLE_POS_ON_SLEW
    if (!isTracking) {
      // bypass during slews so we don't add lag
      yPos = x;
      return encoderCounts;
    }
  #endif

  // pick alpha
  const float a = isTracking ? alphaTracking : alphaSlewing;

  // optional deadband around current output -> no tiny dithering
  #if (EMA_POS_DEADBAND_COUNTS > 0)
      float err = x - yPos;
      if (fabsf(err) < (float)EMA_POS_DEADBAND_COUNTS) {
        return (long)lroundf(yPos);
      }

      // optional snap on big jumps to avoid slow catch-up
      const int snapCounts = isTracking ? EMA_SNAP_COUNTS_TRACK : EMA_SNAP_COUNTS_SLEW;
      if (snapCounts > 0 && fabsf(err) >= (float)snapCounts) {
        yPos = x;
        return (long)lroundf(yPos);
      }

      // normal EMA
      yPos += a * err;
  #else
      float err = x - yPos;

      const int snapCounts = isTracking ? EMA_SNAP_COUNTS_TRACK : EMA_SNAP_COUNTS_SLEW;
      if (snapCounts > 0 && fabsf(err) >= (float)snapCounts) {
        yPos = x;
      } else {
        yPos += a * err;
      }
  #endif

  return (long)lroundf(yPos);
}

// ---------------------------
// VELOCITY FILTER PATH
// ---------------------------
// velocityRaw is in your working units (e.g. steps/s or arcsec/s).
// We return the filtered velocity as float.
float EMAFilter::updateVelocity(float velocityRaw, bool isTracking) {
  if (!velInit) {
    yVel = velocityRaw;
    velInit = true;
    return velocityRaw;
  }

  // For velocity we usually WANT smoothing in tracking,
  // but we often want *minimal lag* in slews. You can choose:
  const float a = isTracking ? alphaTracking : alphaSlewing;
  // Option: set a = 1.0f when !isTracking to bypass during slews.

  // Simple EMA for velocity
  yVel += a * (velocityRaw - yVel);

  // optional tiny deadband in velocity to kill ultra-low jitter
  // Note: for velocity this is better done in the PID loop using a threshold,
  // so we leave it out here for clarity. You can add if you want:
  //   if (fabsf(yVel) < 0.03f) yVel = 0.0f;

  return yVel;
}

#endif // AXIS*_SERVO_FLTR == EMA
