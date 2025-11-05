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
#define SERVO_CALIBRATION_STICTION_REFINE_REL 0.10f           // 10% of base (ceiling/u_break)
#define SERVO_CALIBRATION_REFINE_MAX_ITERATIONS 200           // Safeguard iteration cap

// Lower bound as % of ceiling when sweeping DOWN to find u_break
#define SERVO_CALIBRATION_VELOCITY_SEARCH_MIN_FACTOR 0.5f
#define SERVO_CALIBRATION_VELOCITY_STABILITY_THRESHOLD 50.0f  // steps/sec^2 for steady state

// --- Staircase step sizes (percent of command range) ---
#define SERVO_CALIBRATION_STAIR_STEP_PERCENT 0.05f   // step UP to find ceiling
#define SERVO_CALIBRATION_UBREAK_STEP_PERCENT 0.01f  // step DOWN from ceiling to find u_break
#define SERVO_CALIBRATION_UHOLD_STEP_PERCENT  0.005f // step DOWN from u_break to find u_hold (after kick)

#define SERVO_CALIBRATION_IMBALANCE_ERROR_THRESHOLD 2.0f       // Fwd/Rev imbalance warning threshold
#define SERVO_CALIBRATION_VELOCITY_MEASURE_WINDOW_MS 1500      // window for steady measurement
#define SERVO_CALIBRATION_MIN_DETECTABLE_VELOCITY 0.01f        // steps/sec minimum movement threshold
#define SERVO_CALIBRATION_STICTION_SAMPLE_INTERVAL_MS  1000    // short debounce to avoid a single noisy read


// Calibration states (clarity: ceiling / u_break / u_hold)
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
  float getUBreak(bool forward);  // old floor: min % that starts motion from rest
  float getUHold(bool forward);   // old tracking: min % that maintains motion after a kick

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

  // Small helpers to cut duplication
  enum Dir { FWD = 0, REV = 1 };
  inline Dir   dir()      const { return calibrationDirectionIsForward ? FWD : REV; }
  inline float dirSign()  const { return calibrationDirectionIsForward ? +1.0f : -1.0f; }
  inline void  logDirPrefix() const;
  inline float clampPct(float pct) const {
    return constrain(pct, -SERVO_CALIBRATION_STOP_VELOCITY_PERCENT,
                          +SERVO_CALIBRATION_STOP_VELOCITY_PERCENT);
  }
  inline float tolFor(float base) const {
    return fmaxf(SERVO_CALIBRATION_STICTION_REFINE_ABS,
                 SERVO_CALIBRATION_STICTION_REFINE_REL * base);
  }

  struct Measurement {
    bool  ready;
    float avgVel;   // steps/s
    long  counts;   // encoder counts in window
  };
  Measurement measureWindow(unsigned long now,
                            unsigned long startMs,
                            long startTicks,
                            long curTicks,
                            unsigned long windowMs) const;

  void kickAndQueue(CalibrationState nextState,
                    float kickPctAbs,
                    float testPctAbsSigned);

  // Core methods
  void handleMotorState();
  void processCeiling();       // staircase UP → ceiling
  void processUBreak();        // staircase DOWN from ceiling → u_break
  void processUHoldSearch();   // staircase DOWN from u_break (after kick) → u_hold
  void processImbalanceCheck();
  void handleCalibrationFailure();

  // Motor/measure utilities
  float calculateInstantaneousVelocity();
  void startSettling();
  void startTest(float velocityPercent);
  void setVelocity(float velocityPercent);
  void transitionToUBreak();   // sets up downward sweep for u_break
  void resetCalibrationValues();

  // Configuration
  ServoDcDriver* driver = nullptr; // kept for future use

  uint8_t axisNumber;
  char axisPrefix[32]; // For logging (avoid overflow with "AxisXX ServoCalibration")

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

  // Calibration parameters (shared across phases)
  float calibrationVelocity;          // signed (%)
  float calibrationMinVelocity;       // abs % (lower bound for u_break sweep)
  float calibrationMaxVelocity;       // abs % (ceiling)

  // Results (magnitudes, %)
  float ceiling[2];   // upper bound % that produced sustained motion from rest
  float uBreak[2];    // min % that starts motion from rest (old floor)
  float uHold[2];     // min % that maintains motion after a kick (old tracking)

  // Measured velocities (steps/s) & counts captured for report
  float measCeilVel[2];
  long  measCeilCnt[2];
  float measUBreakVel[2];
  long  measUBreakCnt[2];
  float measUHoldVel[2];
  long  measUHoldCnt[2];

  // Bookkeeping per direction
  bool everMoved[2];
  int  uBreakIters[2];
  int  uHoldIters[2];
  float lastGoodUBreakAbs[2];
  float lastGoodUHoldAbs[2];

  long lastTicks;
  unsigned long lastCheckTime;

  // Staircase step sizes (absolute %)
  float stairStep;
  float uBreakStep;
  float uHoldStep;

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
