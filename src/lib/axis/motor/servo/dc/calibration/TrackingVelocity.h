// -----------------------------------------------------------------------------------
// calibrate servo tracking velocity
#pragma once

#include <Arduino.h>
#include "../../../../../../Common.h"

#if defined(SERVO_MOTOR_PRESENT) && defined(CALIBRATE_SERVO_DC)

//#include "../ServoDriver.h"
#include "../DcServoDriver.h"

#ifndef CALIBRATE_SERVO_AXIS_SELECT
  #define CALIBRATE_SERVO_AXIS_SELECT 3  // 0 None, 1 for RA, 2 for DEC, 3 for all
#endif

// Configuration constants

#define SERVO_CALIBRATION_START_VELOCITY_PERCENT 0.01f        // Initial percentage for ceiling search (based on max velocity)
#define SERVO_CALIBRATION_STOP_VELOCITY_PERCENT 12.0f         // Maximum allowed percentage of velocity to reach
#define SERVO_CALIBRATION_MOTOR_SETTLE_TIME 2000              // ms to wait after stopping motor
#define SERVO_CALIBRATION_VELOCITY_SETTLE_CHECK_INTERVAL 200  // ms between velocity checks
#define SERVO_CALIBRATION_TIMEOUT 1000000                     // ms before calibration fails

// Tolerances (stability guards)
#define SERVO_CALIBRATION_STICTION_REFINE_ABS 0.01f           // % velocity
#define SERVO_CALIBRATION_STICTION_REFINE_REL 0.10f           // 10% of ceiling
#define SERVO_CALIBRATION_REFINE_MAX_ITERATIONS 200           // Safeguard iteration cap

// Lower bound as % of ceiling when sweeping DOWN to find u_break
#define SERVO_CALIBRATION_VELOCITY_SEARCH_MIN_FACTOR 0.5f
#define SERVO_CALIBRATION_VELOCITY_STABILITY_THRESHOLD 50.0f  // steps/sec^2 for steady state

// --- Staircase step sizes (percent of command range) ---
#define SERVO_CALIBRATION_STAIR_STEP_PERCENT 0.05f   // step UP to find ceiling
#define SERVO_CALIBRATION_UBREAK_STEP_PERCENT 0.01f  // step DOWN from ceiling to find u_break
#define SERVO_CALIBRATION_UHOLD_STEP_PERCENT  0.005f // step DOWN from u_break to find u_hold (after kick)

#define SERVO_CALIBRATION_ERROR_THRESHOLD 5.0f                 // (kept for logs)
#define SERVO_CALIBRATION_IMBALANCE_ERROR_THRESHOLD 2.0f       // Fwd/Rev imbalance warning threshold

#define SERVO_CALIBRATION_VELOCITY_MEASURE_WINDOW_MS 1500      // window for steady measurement
#define SERVO_CALIBRATION_MIN_DETECTABLE_VELOCITY 0.01f        // steps/sec minimum movement threshold
#define SERVO_CALIBRATION_STICTION_SAMPLE_INTERVAL_MS  1000    // short debounce to avoid a single noisy read


// Calibration states (renamed for clarity)
enum CalibrationState {
  CALIBRATION_IDLE,
  CALIBRATION_CEILING,       // staircase UP → ceiling (upper stiction bound)
  CALIBRATION_UBREAK,        // staircase DOWN from ceiling → u_break (old floor)
  CALIBRATION_UHOLD_SEARCH,  // staircase DOWN below u_break (after kick) → u_hold (old tracking)
  CALIBRATION_CHECK_IMBALANCE
};

class ServoCalibrateTrackingVelocity {
public:
  ServoCalibrateTrackingVelocity(uint8_t axisNumber);
  void init();
  void start(float trackingFrequency, long instrumentCoordinateSteps);
  void updateState(long instrumentCoordinateSteps);

  // Results (percent commands)
  float getCeiling(bool forward); // unchanged name
  float getUBreak(bool forward);  // old floor
  float getUHold(bool forward);   // old tracking

  bool experimentMode;
  float experimentVelocity;
  bool enabled;

  // Debug/status
  void printReport();

private:
  // Motor control states
  enum MotorState {
    MOTOR_STOPPED,
    MOTOR_SETTLING,
    MOTOR_ACCELERATING,
    MOTOR_RUNNING_STEADY
  };

  // Core methods
  void handleMotorState();
  void processCeiling();       // staircase UP → ceiling
  void processUBreak();        // staircase DOWN from ceiling → u_break
  void processUHoldSearch();   // staircase DOWN from u_break (after kick) → u_hold
  void processImbalanceCheck();
  void handleCalibrationFailure();

  // Helper methods
  float calculateInstantaneousVelocity();
  void startSettling();
  void startTest(float velocityPercent);
  void setVelocity(float velocityPercent);
  void transitionToUBreak();   // sets up downward sweep for u_break
  void resetCalibrationValues();

  inline float clampPct(float pct) const {
    return constrain(pct,
      -SERVO_CALIBRATION_STOP_VELOCITY_PERCENT,
      +SERVO_CALIBRATION_STOP_VELOCITY_PERCENT);
  }

  // Configuration
  ServoDcDriver* driver = nullptr;

  uint8_t axisNumber;
  char axisPrefix[16]; // For logging

  // State tracking
  CalibrationState calibrationState;
  MotorState motorState;
  bool calibrationDirectionIsForward;

  // Timing and measurements
  unsigned long currentTime;
  unsigned long lastStateChangeTime;
  unsigned long settleStartTime;
  unsigned long calibrationStepStartTime;
  unsigned long lastVelocityTime;
  long currentTicks;
  long calibrationStepStartTicks;
  float lastVelocityMeasurement;

  // capture instant counts for velocity calc, and steady window anchors
  long lastDeltaCounts;
  unsigned long steadySinceMs;
  long steadyStartTicks;

  // Calibration parameters
  float calibrationVelocity;          // signed (%)
  float calibrationMinVelocity;       // abs % (lower bound for u_break sweep)
  float calibrationMaxVelocity;       // abs % (ceiling)
  float targetVelocity;               // desired tracking magnitude (counts/sec), only for logs

  // Results (magnitudes, %)
  float ceilingFwd;   // upper bound % that first sustained motion was observed at (FWD)
  float uBreakFwd;    // old floor (min % that starts moving from rest)
  float uHoldFwd;     // old tracking (min % that maintains motion after kick)
  float ceilingRev;
  float uBreakRev;
  float uHoldRev;

  // Measured velocities (steps/s) & counts captured for report
  float ceilingVelFwd;
  float uBreakVelFwd;
  float uHoldVelFwd;
  long  ceilingCountsFwd;
  long  uBreakCountsFwd;
  long  uHoldCountsFwd;

  float ceilingVelRev;
  float uBreakVelRev;
  float uHoldVelRev;
  long  ceilingCountsRev;
  long  uBreakCountsRev;
  long  uHoldCountsRev;

  // Bookkeeping for best samples (kept for report)
  float bestAvgVel;
  long  bestCounts;

  long lastTicks;
  unsigned long lastCheckTime;

  bool everMovedFwd;
  bool everMovedRev;
  int refineIters;  // guard against infinite loops

  // u_hold staircase bookkeeping
  int   holdIters;
  float lastGoodUHoldAbs;

  // Staircase bookkeeping
  float stairStep;         // abs %
  float uBreakStep;        // abs %
  float uHoldStep;         // abs %
  float lastGoodUBreakAbs; // abs % (best working while sweeping down from ceiling)

  // A tiny queue for kickstarting / settling the motor before a test
  bool hasQueuedTest = false;
  CalibrationState queuedState = CALIBRATION_IDLE;
  float queuedVelocity = 0.0f;

  inline void queueNextTest(CalibrationState st, float velocity) {
    queuedState = st;
    queuedVelocity = velocity;
    hasQueuedTest = true;
  }
};

#endif
