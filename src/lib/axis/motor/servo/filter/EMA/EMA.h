// -----------------------------------------------------------------------------------
// exponential moving average (EMA) filter
// first-order low-pass filter with tunable time constant
// α computed from PID period
// reduce encoder noise (quantization + ISR jitter) at sidereal
// no overshoot artifacts
// implements both position and velocity filtering
#pragma once

#include "../FilterBase.h"
#include <math.h>   // lroundf
#include <float.h>

#if AXIS1_SERVO_FLTR == EMA || AXIS2_SERVO_FLTR == EMA || AXIS3_SERVO_FLTR == EMA || \
    AXIS4_SERVO_FLTR == EMA || AXIS5_SERVO_FLTR == EMA || AXIS6_SERVO_FLTR == EMA || \
    AXIS7_SERVO_FLTR == EMA || AXIS8_SERVO_FLTR == EMA || AXIS9_SERVO_FLTR == EMA

#ifndef PID_SAMPLE_TIME_US
  #error "PID_SAMPLE_TIME_US must be defined (PID sample period in microseconds)"
#endif

#ifndef EMA_DT_SEC
  #define EMA_DT_SEC ( (float)PID_SAMPLE_TIME_US * 1e-6f )   // seconds
#endif

// Time constants for tracking vs slewing. These control smoothing strength.
#ifndef EMA_TAU_TRACKING_SEC
  //#define EMA_TAU_TRACKING_SEC (0.015f)   // ~15 ms corner in tracking (for velocity)
  #define EMA_TAU_TRACKING_SEC (0.01f)   // ~10 ms corner in tracking (for positon) α ~ 0.13
#endif
#ifndef EMA_TAU_SLEWING_SEC
  #define EMA_TAU_SLEWING_SEC  (0.005f)   // ~5 ms corner in slewing
#endif

// Optional behavior tweaks for position filtering only:
#ifndef EMA_SNAP_COUNTS_TRACK
  #define EMA_SNAP_COUNTS_TRACK  8    // counts; 0 disables snap
#endif

#ifndef EMA_SNAP_COUNTS_SLEW
  #define EMA_SNAP_COUNTS_SLEW   4    // counts; 0 disables snap
#endif

#ifndef EMA_POS_DEADBAND_COUNTS
  #define EMA_POS_DEADBAND_COUNTS  0  // counts; 0 disables deadband
#endif

// Optional: bypass smoothing during slews for position, because we mostly
// care about smoothing in tracking anyway.
#ifndef EMA_DISABLE_POS_ON_SLEW
  #define EMA_DISABLE_POS_ON_SLEW 1 // set it to true for position
#endif

class EMAFilter : public Filter {
public:
  EMAFilter(float tauTracking = EMA_TAU_TRACKING_SEC,
            float tauSlewing  = EMA_TAU_SLEWING_SEC,
            float dt          = EMA_DT_SEC);

  // helper to compute alpha from tau
  static float alphaFromTau(float tau, float dt);

  void setTimeConstants(float tauTracking, float tauSlewing, float dt);

  // ---------------------------
  // POSITION FILTER PATH
  // ---------------------------
  // encoderCounts is the "measurement"; we output filtered counts as long.
  long update(long encoderCounts, long motorCounts, bool isTracking) override;

  // ---------------------------
  // VELOCITY FILTER PATH
  // ---------------------------
  // velocityRaw is in your working units (e.g. steps/s or arcsec/s).
  // We return the filtered velocity as float.
  float updateVelocity(float velocityRaw, bool isTracking) override;

private:
  // shared alphas
  float alphaTracking = 0.0f;
  float alphaSlewing  = 0.0f;

  // position EMA state
  bool  posInit = false;
  float yPos    = 0.0f;

  // velocity EMA state
  bool  velInit = false;
  float yVel    = 0.0f;
};

#endif // AXIS*_SERVO_FLTR == EMA
