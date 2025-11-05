// -----------------------------------------------------------------------------------
// Servo Tracking Velocity Calibration - Overview
//
// This routine performs a 3-phase, bidirectional calibration to determine the
// velocity % needed to reliably track at sidereal velocity with a DC servo motor.
// The process measures the minimum and maximum velocity %  needed to overcome
// static friction (stiction) and identifies the lowest sustained velocity values
// required for continuous motion.
//
// Maximum velocity is u_break. u_hold is the minimum velocity to maintain motion.
//
// Calibration logic (per direction) proceeds in 3 main steps:
//
// 1) Stiction Ceiling (stictionCeiling):
//    - Staircase search (fixed increments starting from
//      SERVO_CALIBRATION_START_VELOCITY_PERCENT) to find a velocity that initiates
//      any movement.
//    - This gives the upper bound for the motor's stiction threshold.
//    - Result is stored in stictionCeilingFwd or stictionCeilingRev.
//
// 2) Stiction Floor (stictionFloor):
//    - **Sweep downward** in staircase steps starting from stictionCeiling down to
//      SERVO_CALIBRATION_VELOCITY_SEARCH_MIN_FACTOR * stictionCeiling.
//    - This identifies the *lowest* velocity percentage that still reliably causes
//      movement (last good step before stall).
//    - Between each test step, the motor is stopped for
//      SERVO_CALIBRATION_MOTOR_SETTLE_TIME to ensure clean restarts.
//    - The final result is stored in stictionFloorFwd or stictionFloorRev.
//    - This velocity will also serve as a kickstarter during the next phase.
//
// 3) Velocity Tracking Calibration (u_hold):
//    - Kick the motor at stictionFloor, then step **down** by
//      SERVO_CALIBRATION_HOLD_STEP_PERCENT each cycle.
//    - The lowest step that still keeps the motor RUNNING_STEADY is the hold value.
//    - The result is trackingVelFwd or trackingVelRev.
//
// After both directions are calibrated, an imbalance check is performed.
//
// The result is a robust and safe calibration of stiction breaking and min velocity
// customized per motor and mechanical setup and telescope load.

#include "TrackingVelocity.h"

#if defined(SERVO_MOTOR_PRESENT) && defined(CALIBRATE_SERVO_DC)

// Make sure we are dealing with just the motors and no algorithms (plain linear mapping to PWM)
#ifdef SERVO_HYSTERESIS_ENABLE
  #undef SERVO_HYSTERESIS_ENABLE
#endif

#ifdef SERVO_STICTION_KICK
  #undef SERVO_STICTION_KICK
#endif

#ifdef SERVO_NONLINEAR_ENABLE
  #undef SERVO_NONLINEAR_ENABLE
#endif

ServoCalibrateTrackingVelocity::ServoCalibrateTrackingVelocity(uint8_t axisNumber) {
  this->axisNumber = axisNumber;
  snprintf(axisPrefix, sizeof(axisPrefix), "Axis%d ServoCalibration", axisNumber);
}

void ServoCalibrateTrackingVelocity::init() {
  experimentVelocity = 0.0f;
  experimentMode = false;
  calibrationState = CALIBRATION_IDLE;
  motorState = MOTOR_STOPPED;

  // Reset calibration values
  resetCalibrationValues();

  lastStateChangeTime = 0;
  calibrationStepStartTime = 0;
  calibrationStepStartTicks = 0;
  lastVelocityMeasurement = 0;
  lastVelocityTime = 0;

  lastTicks = 0;
  lastCheckTime = 0;
  lastDeltaCounts = 0;

  steadySinceMs = 0;
  steadyStartTicks = 0;

  enabled = false;
  everMovedFwd = false;
  everMovedRev = false;

  velIters = 0;
  bestLowVelocitySearchAbs = 0.0f;
  bestVelSearchErr = 1e9f; // sentinel "worst" error
  velSearchBestAvgVel = 0.0f;
  velSearchBestCounts = 0;

  refineIters = 0;

  // NEW: staircase step sizes (absolute %)
  stairStep = fabsf(SERVO_CALIBRATION_STAIR_STEP_PERCENT);
  floorStep = fabsf(SERVO_CALIBRATION_FLOOR_STEP_PERCENT);
  holdStep  = fabsf(SERVO_CALIBRATION_HOLD_STEP_PERCENT);
  lastGoodHoldAbs  = 0.0f;
  lastGoodFloorAbs = 0.0f;

  // setup the queue
  hasQueuedTest = false;
  queuedState = CALIBRATION_IDLE;
  queuedVelocity = 0.0f;

  // stop the motor
  setVelocity(0);
  startSettling();
}

void ServoCalibrateTrackingVelocity::start(float trackingFrequency, long /*instrumentCoordinateSteps*/) {
  if (!(CALIBRATE_SERVO_AXIS_SELECT & (1 << (axisNumber - 1)))) {
    VF("MSG: "); V(axisPrefix); VLF(" Calibration skipped for this axis");
    return;
  }

  VF("MSG: "); V(axisPrefix); VL(" Starting 3-phase bidirectional calibration");

  // Initialize state machine
  enabled = true;
  experimentMode = true;
  calibrationDirectionIsForward = true;
  calibrationState = CALIBRATION_STICTION_CEILING;
  motorState = MOTOR_STOPPED;
  calibrationVelocity = SERVO_CALIBRATION_START_VELOCITY_PERCENT;
  targetVelocity = trackingFrequency;
  lastStateChangeTime = millis();

  // Start by stopping motor and waiting for settle
  setVelocity(0);
  startSettling();
}

void ServoCalibrateTrackingVelocity::updateState(long instrumentCoordinateSteps) {
  if(!enabled || !experimentMode) return;

  currentTime = millis();
  currentTicks = instrumentCoordinateSteps;

  // Handle motor state transitions
  handleMotorState();

  // State timeout check (safety feature)
  if (currentTime - lastStateChangeTime > SERVO_CALIBRATION_TIMEOUT ) {
    VF("WARN: "); V(axisPrefix); VLF(" State timeout, resetting calibration");
    handleCalibrationFailure();
    return;
  }

  // Only process state machine when motor is in steady state or stopped
  if (motorState != MOTOR_RUNNING_STEADY && motorState != MOTOR_STOPPED) return;

  // Main state machine processing
  switch (calibrationState) {
    case CALIBRATION_IDLE:
      break;

    case CALIBRATION_STICTION_CEILING:
      processStictionCeiling();
      break;

    case CALIBRATION_STICTION_FLOOR:
      processStictionFloor();
      break;

    case CALIBRATION_VELOCITY_SEARCH:
      processVelocitySearch();
      break;

    case CALIBRATION_CHECK_IMBALANCE:
      processImbalanceCheck();
      break;

    default:
      break;
  }
}

// Private helper methods //////////////////////////////////////////////////////

void ServoCalibrateTrackingVelocity::handleMotorState() {
  float currentVelocity = 0.0;
  switch (motorState) {
    case MOTOR_SETTLING:
      // check also the velocity
      currentVelocity = calculateInstantaneousVelocity();
      if (currentTime - settleStartTime >= SERVO_CALIBRATION_MOTOR_SETTLE_TIME) {
        motorState = MOTOR_STOPPED;
        V(axisPrefix); VF(": Motor settled to STOPPED_STATE"); VF(" Vel="); V(currentVelocity); VL(" steps/s");

        // NOW safe to start the next test, if any
        if (hasQueuedTest) {
          // if we were kicking the motor
          kickToFloorVelocity = false;
          calibrationState = queuedState;
          hasQueuedTest = false;
          startTest(queuedVelocity);
        }
      }
      lastVelocityMeasurement = currentVelocity;
      lastVelocityTime = currentTime;

      break;

    case MOTOR_ACCELERATING:
      // Check if motor has reached steady state
      if (currentTime - lastVelocityTime >= SERVO_CALIBRATION_VELOCITY_SETTLE_CHECK_INTERVAL) {
        currentVelocity = calculateInstantaneousVelocity();
        float velocityChange = fabsf(currentVelocity - lastVelocityMeasurement);

        if (fabsf(currentVelocity) > SERVO_CALIBRATION_MIN_DETECTABLE_VELOCITY &&
            fabsf(velocityChange) < SERVO_CALIBRATION_VELOCITY_STABILITY_THRESHOLD) {

          motorState = MOTOR_RUNNING_STEADY;
          calibrationStepStartTime = currentTime;
          calibrationStepStartTicks = currentTicks;

          // NEW: start steady window anchors for sustained stiction check
          steadySinceMs = currentTime;
          steadyStartTicks = currentTicks;

          V(axisPrefix);
          if (calibrationDirectionIsForward) { VF(" FWD"); } else { VF(" REV"); }
          VF(": STEADY_STATE  @vel="); V(calibrationVelocity); VF("%");
          VF(" Vel="); V(currentVelocity); VL(" steps/s");

          // NOW safe to start the next test, if any
          if (hasQueuedTest) {
            // if we were kicking the motor
            kickToFloorVelocity = false;
            calibrationState = queuedState;
            hasQueuedTest = false;
            startTest(queuedVelocity);
          }

        } else if (fabsf(currentVelocity) <= SERVO_CALIBRATION_MIN_DETECTABLE_VELOCITY) {
          // The motor never managed to
          motorState = MOTOR_STOPPED;
          calibrationStepStartTime = currentTime;
          calibrationStepStartTicks = currentTicks;

          V(axisPrefix);
          if (calibrationDirectionIsForward) { VF(" FWD"); } else { VF(" REV"); }
          VF(": STOPPED_STATE  @vel="); V(calibrationVelocity); VF("%");
          VF(" Vel="); V(currentVelocity); VL(" steps/s");
        }

        lastVelocityMeasurement = currentVelocity;
        lastVelocityTime = currentTime;
      }
      break;

    case MOTOR_RUNNING_STEADY:
      // Motor is running steady, nothing to do (ceil/floor/velocity code will read window)
      break;

    case MOTOR_STOPPED:
      // Nothing to do here, waiting for state machine to start next test
      break;
  }
}

// compute current speed
float ServoCalibrateTrackingVelocity::calculateInstantaneousVelocity() {
  if (lastCheckTime == 0) {
    lastCheckTime = currentTime;
    lastTicks = currentTicks;
    lastDeltaCounts = 0;
    return 0.0f;
  }

  float elapsed = (currentTime - lastCheckTime) / 1000.0f;
  if (elapsed <= 0) return 0.0f;
  long dCounts = (currentTicks - lastTicks);
  float velocity = dCounts / elapsed; // steps/sec

#ifdef SERVO_CAL_DEBUG
  VF("DBG: "); V(axisPrefix);VF(" vel (%) ="); V(calibrationVelocity);
  VF(" dTicks="); V(dCounts);
  VF(" dt(ms)=");V(currentTime - lastCheckTime);
  VF(" Vel="); V(velocity); VL(" steps/s");
#endif

  lastTicks = currentTicks;
  lastCheckTime = currentTime;
  lastDeltaCounts = dCounts; // cache raw counts used for this instant velocity

  return velocity;
}

// This function checks whether stiction is broken. Staircase: if no sustained motion, increase by fixed step (clamped).
void ServoCalibrateTrackingVelocity::processStictionCeiling() {
  const float sign = calibrationDirectionIsForward ? +1.0f : -1.0f;

  if (motorState == MOTOR_RUNNING_STEADY) {
    unsigned long steadyElapsedMs = currentTime - steadySinceMs;
    if (steadyElapsedMs < SERVO_CALIBRATION_STICTION_SAMPLE_INTERVAL_MS) return;

    float elapsedSec = steadyElapsedMs / 1000.0f;
    long  dCounts    = currentTicks - steadyStartTicks;
    float avgVel     = dCounts / elapsedSec;

    if (fabsf(avgVel) > SERVO_CALIBRATION_MIN_DETECTABLE_VELOCITY) {
      // motion detected and sustained at this velocity
      float absVelocity = fabsf(calibrationVelocity);
      float& stictionMax = calibrationDirectionIsForward ? stictionCeilingFwd : stictionCeilingRev;
      if (calibrationDirectionIsForward) {
        everMovedFwd = true;
        stictionCeilingVelFwd = avgVel;
        stictionCeilingCountsFwd = dCounts;
      } else {
        everMovedRev = true;
        stictionCeilingVelRev = avgVel;
        stictionCeilingCountsRev = dCounts;
      }
      stictionMax = absVelocity;

      V(axisPrefix);
      if (calibrationDirectionIsForward) { VF(" FWD"); } else { VF(" REV"); }
      VF(" break: BROKEN STICTION @ Velocity % ="); V(calibrationVelocity);
      VF(" Vel="); V(avgVel); VL(" steps/s");

      setVelocity(0);
      settleStartTime = currentTime;
      motorState = MOTOR_SETTLING;

      transitionToRefine();
      return;
    }
  }

  // If stopped or steady w/out movement long enough -> step up
  if (motorState == MOTOR_STOPPED || motorState == MOTOR_RUNNING_STEADY) {
    if (currentTime - lastVelocityTime < SERVO_CALIBRATION_STICTION_SAMPLE_INTERVAL_MS) return;

    float nextAbs = fminf(fabsf(calibrationVelocity) + stairStep, SERVO_CALIBRATION_STOP_VELOCITY_PERCENT);
    calibrationVelocity = sign * nextAbs;

    if (nextAbs >= SERVO_CALIBRATION_STOP_VELOCITY_PERCENT) {
      float& stictionMax = calibrationDirectionIsForward ? stictionCeilingFwd : stictionCeilingRev;
      stictionMax = SERVO_CALIBRATION_STOP_VELOCITY_PERCENT;
      bool ever = calibrationDirectionIsForward ? everMovedFwd : everMovedRev;
      if (!ever) {
        V(axisPrefix); VF(calibrationDirectionIsForward ? " FWD" : " REV");
        VLF(" break: COULD NOT BREAK STICTION up to SERVO_CALIBRATION_STOP_VELOCITY_PERCENT – skipping refine");
        setVelocity(0);
        startSettling();
        if (calibrationDirectionIsForward) {
          calibrationDirectionIsForward = false;
          queueNextTest(CALIBRATION_STICTION_CEILING, -SERVO_CALIBRATION_START_VELOCITY_PE
