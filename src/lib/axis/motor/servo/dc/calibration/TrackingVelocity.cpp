// -----------------------------------------------------------------------------------
// Servo Tracking Velocity Calibration - Overview
//
// This routine performs a 3-phase, bidirectional calibration to determine:
//   • ceiling: an upper bound % that produces sustained motion from rest
//   • u_break: minimum % that starts moving from rest (old "floor")
//   • u_hold : minimum % that maintains motion after a kick (old "tracking")
//
// Search strategy (per direction):
// 1) ceiling: staircase UP from a tiny % until sustained motion is detected.
// 2) u_break: start at ceiling and staircase DOWN in fixed steps until stall;
//             the last steady step before stall is u_break.
// 3) u_hold : kick at u_break, then staircase DOWN below it; the last steady
//             step before stall is u_hold.
//
// After both directions are calibrated, an imbalance check is performed.

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

  bestAvgVel = 0.0f;
  bestCounts = 0;

  refineIters = 0;
  holdIters = 0;

  // staircase step sizes (absolute %)
  stairStep      = fabsf(SERVO_CALIBRATION_STAIR_STEP_PERCENT);
  uBreakStep     = fabsf(SERVO_CALIBRATION_UBREAK_STEP_PERCENT);
  uHoldStep      = fabsf(SERVO_CALIBRATION_UHOLD_STEP_PERCENT);
  lastGoodUHoldAbs  = 0.0f;
  lastGoodUBreakAbs = 0.0f;

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
  calibrationState = CALIBRATION_CEILING;
  motorState = MOTOR_STOPPED;
  calibrationVelocity = SERVO_CALIBRATION_START_VELOCITY_PERCENT;
  targetVelocity = trackingFrequency; // only for logs
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

    case CALIBRATION_CEILING:
      processCeiling();
      break;

    case CALIBRATION_UBREAK:
      processUBreak();
      break;

    case CALIBRATION_UHOLD_SEARCH:
      processUHoldSearch();
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

          // start steady window anchors for sustained stiction check
          steadySinceMs = currentTime;
          steadyStartTicks = currentTicks;

          V(axisPrefix);
          if (calibrationDirectionIsForward) { VF(" FWD"); } else { VF(" REV"); }
          VF(": STEADY_STATE  @%="); V(calibrationVelocity);
          VF(" v="); V(currentVelocity); VL(" steps/s");

          if (hasQueuedTest) {
            calibrationState = queuedState;
            hasQueuedTest = false;
            startTest(queuedVelocity);
          }

        } else if (fabsf(currentVelocity) <= SERVO_CALIBRATION_MIN_DETECTABLE_VELOCITY) {
          motorState = MOTOR_STOPPED;
          calibrationStepStartTime = currentTime;
          calibrationStepStartTicks = currentTicks;

          V(axisPrefix);
          if (calibrationDirectionIsForward) { VF(" FWD"); } else { VF(" REV"); }
          VF(": STOPPED_STATE  @%="); V(calibrationVelocity);
          VF(" v="); V(currentVelocity); VL(" steps/s");
        }

        lastVelocityMeasurement = currentVelocity;
        lastVelocityTime = currentTime;
      }
      break;

    case MOTOR_RUNNING_STEADY:
      break;

    case MOTOR_STOPPED:
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
  VF("DBG: "); V(axisPrefix);VF(" %="); V(calibrationVelocity);
  VF(" dTicks="); V(dCounts);
  VF(" dt(ms)=");V(currentTime - lastCheckTime);
  VF(" v="); V(velocity); VL(" steps/s");
#endif

  lastTicks = currentTicks;
  lastCheckTime = currentTime;
  lastDeltaCounts = dCounts; // cache raw counts used for this instant velocity

  return velocity;
}

// Staircase UP: find ceiling (first sustained movement).
void ServoCalibrateTrackingVelocity::processCeiling() {
  const float sign = calibrationDirectionIsForward ? +1.0f : -1.0f;

  if (motorState == MOTOR_RUNNING_STEADY) {
    unsigned long steadyElapsedMs = currentTime - steadySinceMs;
    if (steadyElapsedMs < SERVO_CALIBRATION_STICTION_SAMPLE_INTERVAL_MS) return;

    float elapsedSec = steadyElapsedMs / 1000.0f;
    long  dCounts    = currentTicks - steadyStartTicks;
    float avgVel     = dCounts / elapsedSec;

    if (fabsf(avgVel) > SERVO_CALIBRATION_MIN_DETECTABLE_VELOCITY) {
      float absVelocity = fabsf(calibrationVelocity);
      float& ceilingRef = calibrationDirectionIsForward ? ceilingFwd : ceilingRev;
      if (calibrationDirectionIsForward) {
        everMovedFwd = true;
        ceilingVelFwd = avgVel;
        ceilingCountsFwd = dCounts;
      } else {
        everMovedRev = true;
        ceilingVelRev = avgVel;
        ceilingCountsRev = dCounts;
      }
      ceilingRef = absVelocity;

      V(axisPrefix);
      if (calibrationDirectionIsForward) { VF(" FWD"); } else { VF(" REV"); }
      VF(" ceiling found @ %="); V(calibrationVelocity);
      VF(" (v="); V(avgVel); VL(" steps/s)");

      setVelocity(0);
      settleStartTime = currentTime;
      motorState = MOTOR_SETTLING;

      transitionToUBreak(); // set up downward sweep
      return;
    }
  }

  // step up if not yet moving
  if (motorState == MOTOR_STOPPED || motorState == MOTOR_RUNNING_STEADY) {
    if (currentTime - lastVelocityTime < SERVO_CALIBRATION_STICTION_SAMPLE_INTERVAL_MS) return;

    float nextAbs = fminf(fabsf(calibrationVelocity) + stairStep, SERVO_CALIBRATION_STOP_VELOCITY_PERCENT);
    calibrationVelocity = sign * nextAbs;

    if (nextAbs >= SERVO_CALIBRATION_STOP_VELOCITY_PERCENT) {
      float& ceilingRef = calibrationDirectionIsForward ? ceilingFwd : ceilingRev;
      ceilingRef = SERVO_CALIBRATION_STOP_VELOCITY_PERCENT;
      bool ever = calibrationDirectionIsForward ? everMovedFwd : everMovedRev;
      if (!ever) {
        V(axisPrefix); VF(calibrationDirectionIsForward ? " FWD" : " REV");
        VLF(" ceiling not found up to limit – skipping u_break/u_hold");
        setVelocity(0);
        startSettling();
        if (calibrationDirectionIsForward) {
          calibrationDirectionIsForward = false;
          queueNextTest(CALIBRATION_CEILING, -SERVO_CALIBRATION_START_VELOCITY_PERCENT);
        } else {
          calibrationState = CALIBRATION_CHECK_IMBALANCE;
          processImbalanceCheck();
        }
        return;
      }
      transitionToUBreak();
    } else {
      startTest(calibrationVelocity);
    }
  }
}

void ServoCalibrateTrackingVelocity::transitionToUBreak() {
  // Enter the u_break phase (downward sweep from ceiling)
  calibrationState = CALIBRATION_UBREAK;
  lastStateChangeTime = currentTime;

  const float ceilingAbs = calibrationDirectionIsForward ? ceilingFwd : ceilingRev;

  // Sweep bounds: down from ceiling to a lower bound
  calibrationMaxVelocity = ceilingAbs; // start here
  calibrationMinVelocity = SERVO_CALIBRATION_VELOCITY_SEARCH_MIN_FACTOR * ceilingAbs; // lower bound
  if (calibrationMinVelocity < 0.0f) calibrationMinVelocity = 0.0f;   // safety clamp

  // Start at ceiling
  float startAbs = calibrationMaxVelocity;
  if (startAbs <= 0.0f) startAbs = stairStep; // avoid zero
  calibrationVelocity = (calibrationDirectionIsForward ? +startAbs : -startAbs);

  refineIters = 0;
  lastGoodUBreakAbs = 0.0f;

  startTest(calibrationVelocity);
}

// u_break via staircase (DOWNWARD): start at ceiling and decrement by uBreakStep.
// The u_break is the lowest % that still moves from rest: last steady before stall.
void ServoCalibrateTrackingVelocity::processUBreak() {
  if (++refineIters > SERVO_CALIBRATION_REFINE_MAX_ITERATIONS) {
    VLF("WARN: u_break sweep iteration cap reached; using last good / ceiling");
    float decider = (lastGoodUBreakAbs > 0.0f) ? lastGoodUBreakAbs : calibrationMaxVelocity;
    float& uBreakRef = calibrationDirectionIsForward ? uBreakFwd : uBreakRev;
    uBreakRef = decider;

    // measured velocity & counts from last state
    if (calibrationDirectionIsForward) {
      uBreakVelFwd    = lastVelocityMeasurement;
      uBreakCountsFwd = lastDeltaCounts;
    } else {
      uBreakVelRev    = lastVelocityMeasurement;
      uBreakCountsRev = lastDeltaCounts;
    }


    // proceed to u_hold search
    calibrationState = CALIBRATION_UHOLD_SEARCH;
    holdIters = 0;
    lastGoodUHoldAbs = decider; // conservative
    bestAvgVel = 0.0f;
    bestCounts = 0;

    // queue a first test just below u_break
    const float firstBelow = fmaxf(stairStep, decider - uHoldStep);
    const float signedNext = (calibrationDirectionIsForward ? +firstBelow : -firstBelow);
    queueNextTest(CALIBRATION_UHOLD_SEARCH, signedNext);

    setVelocity(calibrationDirectionIsForward ? +decider : -decider); // kick at u_break
    motorState = MOTOR_ACCELERATING;
    lastStateChangeTime = currentTime;
    lastVelocityTime = currentTime;
    lastVelocityMeasurement = 0;
    return;
  }

  if (motorState != MOTOR_RUNNING_STEADY && motorState != MOTOR_STOPPED) return;

  // If we are steady at current step, record it and step down further.
  if (motorState == MOTOR_RUNNING_STEADY) {
    const unsigned long dt_ms = currentTime - calibrationStepStartTime;
    if (dt_ms < SERVO_CALIBRATION_VELOCITY_MEASURE_WINDOW_MS) return;

    const float elapsedSec = dt_ms / 1000.0f;
    const long  dCounts    = currentTicks - calibrationStepStartTicks;
    const float avgVel     = (float)dCounts / elapsedSec;

    // use avgVel instead of lastVelocityMeasurement
    if (calibrationDirectionIsForward) {
      uBreakVelFwd    = avgVel;
      uBreakCountsFwd = dCounts;
    } else {
      uBreakVelRev    = avgVel;
      uBreakCountsRev = dCounts;
    }

    V(axisPrefix);
    if (calibrationDirectionIsForward) { VF(" FWD"); } else { VF(" REV"); }
    VF(" u_break sweep: MOVE @ "); V(calibrationVelocity); VF("%, v≈");
    V(lastVelocityMeasurement); VL(" steps/s");

    // Compute next downward step
    float nextAbs = fmaxf(calibrationMinVelocity, lastGoodUBreakAbs - uBreakStep);

    // If we cannot step further down, accept lastGoodUBreakAbs as u_break
    if (nextAbs >= lastGoodUBreakAbs - 1e-6f) {
      float& uBreakRef = calibrationDirectionIsForward ? uBreakFwd : uBreakRev;
      uBreakRef = lastGoodUBreakAbs;

      // Move to u_hold search
      calibrationState = CALIBRATION_UHOLD_SEARCH;
      holdIters = 0;
      lastGoodUHoldAbs = lastGoodUBreakAbs;

      // queue a first below-u_break test
      float below = fmaxf(stairStep, lastGoodUBreakAbs - uHoldStep);
      queueNextTest(CALIBRATION_UHOLD_SEARCH,
                    (calibrationDirectionIsForward ? +below : -below));

      // Kick at u_break then test below
      setVelocity(calibrationDirectionIsForward ? +lastGoodUBreakAbs : -lastGoodUBreakAbs);
      motorState = MOTOR_ACCELERATING;
      lastStateChangeTime = currentTime;
      lastVelocityTime = currentTime;
      lastVelocityMeasurement = 0;
      V(axisPrefix); VF(calibrationDirectionIsForward ? " FWD" : " REV");
      VF(" kick: u_break %="); V(lastGoodUBreakAbs); VL("");
      return;
    }

    // Continue the downward sweep
    calibrationVelocity = (calibrationDirectionIsForward ? +nextAbs : -nextAbs);
    startTest(calibrationVelocity);
    return;
  }

  // If we STOPPED, current step is too low -> u_break = last good moving step.
  if (motorState == MOTOR_STOPPED) {
    float uBreakAbs = (lastGoodUBreakAbs > 0.0f) ? lastGoodUBreakAbs : calibrationMaxVelocity; // fallback to ceiling
    float& uBreakRef = calibrationDirectionIsForward ? uBreakFwd : uBreakRev;
    uBreakRef = uBreakAbs;

    V(axisPrefix);
    if (calibrationDirectionIsForward) { VF(" FWD"); } else { VF(" REV"); }
    VF(" u_break sweep: STOP @ "); V(fabsf(calibrationVelocity)); VF("% -> u_break=");
    V(uBreakAbs); VLF("%");

    // Enter u_hold search
    calibrationState = CALIBRATION_UHOLD_SEARCH;
    holdIters = 0;
    lastGoodUHoldAbs = uBreakAbs;

    // queue a first below-u_break test
    float below = fmaxf(stairStep, uBreakAbs - uHoldStep);
    queueNextTest(CALIBRATION_UHOLD_SEARCH,
                  (calibrationDirectionIsForward ? +below : -below));

    // Kick at u_break then test below
    setVelocity(calibrationDirectionIsForward ? +uBreakAbs : -uBreakAbs);
    motorState = MOTOR_ACCELERATING;
    lastStateChangeTime = currentTime;
    lastVelocityTime = currentTime;
    lastVelocityMeasurement = 0;
    return;
  }
}

// u_hold via staircase: kick at u_break; then step down by uHoldStep until stall.
// Last good (steady) is taken as u_hold; also records measured avg.
void ServoCalibrateTrackingVelocity::processUHoldSearch() {
  if (motorState != MOTOR_RUNNING_STEADY && motorState != MOTOR_STOPPED) return;

  const float uBreakAbs = calibrationDirectionIsForward ? uBreakFwd : uBreakRev;
  const float sign      = calibrationDirectionIsForward ? +1.0f : -1.0f;

  // Safety guard
  if (++holdIters > SERVO_CALIBRATION_REFINE_MAX_ITERATIONS) {
    float& uHoldRef = calibrationDirectionIsForward ? uHoldFwd : uHoldRev;
    uHoldRef = (lastGoodUHoldAbs > 0.0f) ? lastGoodUHoldAbs : uBreakAbs;

    if (calibrationDirectionIsForward) {
      uHoldVelFwd = bestAvgVel != 0.0f ? bestAvgVel : uBreakVelFwd;
      uHoldCountsFwd = bestCounts != 0  ? bestCounts : uBreakCountsFwd;
    } else {
      uHoldVelRev = bestAvgVel != 0.0f ? bestAvgVel : uBreakVelRev;
      uHoldCountsRev = bestCounts != 0  ? bestCounts : uBreakCountsRev;
    }
    VF("WARN: "); V(axisPrefix); VF(calibrationDirectionIsForward ? " FWD" : " REV");
    VLF(" u_hold search iteration cap reached; using last good / u_break");
    goto uhold_done;
  }

  if (motorState == MOTOR_RUNNING_STEADY) {
    // Measure a window to ensure we truly hold
    const unsigned long dt_ms = currentTime - calibrationStepStartTime;
    if (dt_ms < SERVO_CALIBRATION_VELOCITY_MEASURE_WINDOW_MS) return;

    const float elapsedSec = dt_ms / 1000.0f;
    const long  dCounts    = currentTicks - calibrationStepStartTicks;
    const float avgVel     = (float)dCounts / elapsedSec;

    // record as best-so-far at this lower command
    bestAvgVel = avgVel;
    bestCounts = dCounts;

    lastGoodUHoldAbs = fabsf(calibrationVelocity);

    // Step one more notch down; if we go below 0, clamp to a tiny min
    float nextAbs = fmaxf(stairStep, lastGoodUHoldAbs - uHoldStep);

    // Practical lower limit relative to u_break -> accept current
    const float tolAbs = SERVO_CALIBRATION_STICTION_REFINE_ABS;
    const float tolRel = SERVO_CALIBRATION_STICTION_REFINE_REL * uBreakAbs;
    const float minAllowed = fmaxf(tolAbs, tolRel);
    if (nextAbs <= minAllowed) {
      float& uHoldRef = calibrationDirectionIsForward ? uHoldFwd : uHoldRev;
      uHoldRef = lastGoodUHoldAbs;

      if (calibrationDirectionIsForward) {
        uHoldVelFwd = avgVel;
        uHoldCountsFwd = dCounts;
      } else {
        uHoldVelRev = avgVel;
        uHoldCountsRev = dCounts;
      }
      VF("MSG: "); V(axisPrefix);
      VF(calibrationDirectionIsForward ? " FWD" : " REV");
      VF(" u_hold found (below u_break): %="); V(uHoldRef); VLF("");
      goto uhold_done;
    }

    // Kick at u_break, then queue test at nextAbs
    queueNextTest(CALIBRATION_UHOLD_SEARCH, sign * nextAbs);
    setVelocity(sign * uBreakAbs); // kick
    motorState = MOTOR_ACCELERATING;
    lastStateChangeTime = currentTime;
    lastVelocityTime = currentTime;
    lastVelocityMeasurement = 0;

    V(axisPrefix); VF(calibrationDirectionIsForward ? " FWD" : " REV");
    VF(" kick: u_break %="); V(uBreakAbs); VF(" -> test %="); V(nextAbs); VLF("");
    return;
  }

  // MOTOR_STOPPED -> the last commanded value was too low; use last good as u_hold
  {
    float& uHoldRef = calibrationDirectionIsForward ? uHoldFwd : uHoldRev;
    uHoldRef = (lastGoodUHoldAbs > 0.0f) ? lastGoodUHoldAbs : uBreakAbs;

    // measured velocity & counts: prefer last best steady sample
    if (calibrationDirectionIsForward) {
      uHoldVelFwd = (bestAvgVel != 0.0f) ? bestAvgVel : uBreakVelFwd;
      uHoldCountsFwd = (bestCounts   != 0  ) ? bestCounts   : uBreakCountsFwd;
    } else {
      uHoldVelRev = (bestAvgVel != 0.0f) ? bestAvgVel : uBreakVelRev;
      uHoldCountsRev = (bestCounts   != 0  ) ? bestCounts   : uBreakCountsRev;
    }

    VF("MSG: "); V(axisPrefix);
    VF(calibrationDirectionIsForward ? " FWD" : " REV");
    VF(" u_hold: stall encountered; using %="); V(uHoldRef); VLF("");
  }

uhold_done:
  setVelocity(0);
  startSettling();
  if (calibrationDirectionIsForward) {
    // Switch to reverse direction starting again from ceiling
    calibrationDirectionIsForward = false;
    calibrationState = CALIBRATION_CEILING;
    // Queue the next test, since we first have to ensure stop
    queueNextTest(CALIBRATION_CEILING, -SERVO_CALIBRATION_START_VELOCITY_PERCENT);
  } else {
    calibrationState = CALIBRATION_CHECK_IMBALANCE;
    processImbalanceCheck();
  }
}

void ServoCalibrateTrackingVelocity::processImbalanceCheck() {
  // Print calibration report
  printReport();

  // Check for significant imbalance between u_hold FWD/REV
  if (uHoldFwd > 0 && uHoldRev > 0) {
    float avgHold = (uHoldFwd + uHoldRev) / 2.0f;
    float imbalance = fabsf(uHoldFwd - uHoldRev) / avgHold * 100.0f;

    if (imbalance > SERVO_CALIBRATION_IMBALANCE_ERROR_THRESHOLD) {
      VF("WARN: "); V(axisPrefix);
      VF(" Significant imbalance: "); V(imbalance); VL("%");
    }
  }

  // Calibration complete
  experimentMode = false;
  calibrationState = CALIBRATION_IDLE;
  setVelocity(0);
  VF("MSG: "); V(axisPrefix); VL(" complete");
}

void ServoCalibrateTrackingVelocity::handleCalibrationFailure() {
  VF("ERR: "); V(axisPrefix); VL("Calibration failed, resetting");

  // Reset to safe state
  experimentMode = false;
  calibrationState = CALIBRATION_IDLE;
  setVelocity(0);

  // Set default values
  resetCalibrationValues();

  hasQueuedTest = false; // cancel any queued action
}

void ServoCalibrateTrackingVelocity::startSettling() {
  settleStartTime = currentTime;
  motorState = MOTOR_SETTLING;
  VF("DBG: "); V(axisPrefix); VF(" Starting settling for ");
  V(SERVO_CALIBRATION_MOTOR_SETTLE_TIME); VL("ms");
}

void ServoCalibrateTrackingVelocity::startTest(float velocity) {
  calibrationVelocity = clampPct(velocity);
  setVelocity(calibrationVelocity);
  lastVelocityTime = currentTime;
  lastVelocityMeasurement = 0;
  motorState = MOTOR_ACCELERATING;
  lastStateChangeTime = currentTime;

  V(axisPrefix);
  if (calibrationDirectionIsForward) {VF(" FWD");} else {VF(" REV");}
  VF(" Start test @ %="); V(calibrationVelocity); VLF("");
}

void ServoCalibrateTrackingVelocity::setVelocity(float velocityPercent) {
  // Clamp percent
  float clamped = clampPct(velocityPercent);
  experimentVelocity = clamped; // telemetry in percent
}

// Getters ////////////////////////////////////////////////////////////////////
float ServoCalibrateTrackingVelocity::getCeiling(bool forward) {
  return forward ? ceilingFwd : ceilingRev;
}
float ServoCalibrateTrackingVelocity::getUBreak(bool forward) {
  return forward ? uBreakFwd : uBreakRev;
}
float ServoCalibrateTrackingVelocity::getUHold(bool forward) {
  return forward ? uHoldFwd : uHoldRev;
}

// --- Compact report helpers
static inline void prDirHeader(const char* dir) {
  VF(dir); VF("  | ");
}
static inline void prPctVelCnt(float pct, float vel, long cnt) {
  VF("% "); V(pct); VF(", v "); V(vel); VF(" steps/s, cnt "); V(cnt);
}

void ServoCalibrateTrackingVelocity::printReport() {
  VF("\n=== Calibration Report: "); V(axisPrefix); VLF(" ===");

  // Header (ceiling / u_break / u_hold)
  VLF("Dir  | ceiling                        | u_break                         | u_hold");
  VLF("-----+--------------------------------+--------------------------------+--------------------------------");

  // FWD row
  prDirHeader("FWD");
  prPctVelCnt(ceilingFwd, ceilingVelFwd, ceilingCountsFwd); VF("  | ");
  prPctVelCnt(uBreakFwd,  uBreakVelFwd,  uBreakCountsFwd);  VF("  | ");
  prPctVelCnt(uHoldFwd,   uHoldVelFwd,   uHoldCountsFwd);   VL("");

  // REV row
  prDirHeader("REV");
  prPctVelCnt(ceilingRev, ceilingVelRev, ceilingCountsRev); VF("  | ");
  prPctVelCnt(uBreakRev,  uBreakVelRev,  uBreakCountsRev);  VF("  | ");
  prPctVelCnt(uHoldRev,   uHoldVelRev,   uHoldCountsRev);   VL("");

  // Imbalance (use magnitudes of %)
  float uBreakImbalance = fabsf(uBreakFwd - uBreakRev);
  float uHoldImbalance  = fabsf(uHoldFwd - uHoldRev);

  VF("Δ u_break: "); V(uBreakImbalance); VF("%, ");
  VF("Δ u_hold: ");  V(uHoldImbalance);  VLF("%");

  if (uHoldFwd >= SERVO_CALIBRATION_STOP_VELOCITY_PERCENT - 0.1f) {
    VLF("WARN: FWD u_hold at calibration limit");
  }
  if (uHoldRev >= SERVO_CALIBRATION_STOP_VELOCITY_PERCENT - 0.1f) {
    VLF("WARN: REV u_hold at calibration limit");
  }

  VF("=== End Report ===\n");
}

void ServoCalibrateTrackingVelocity::resetCalibrationValues(void) {
  ceilingFwd = SERVO_CALIBRATION_STOP_VELOCITY_PERCENT;
  uBreakFwd  = 0.0f;
  uHoldFwd   = 0.0f;
  ceilingRev = SERVO_CALIBRATION_STOP_VELOCITY_PERCENT;
  uBreakRev  = 0.0f;
  uHoldRev   = 0.0f;

  // Measured velocities (steps/s) & counts
  ceilingVelFwd = uBreakVelFwd = uHoldVelFwd = 0.0f;
  ceilingCountsFwd = uBreakCountsFwd = uHoldCountsFwd = 0;

  ceilingVelRev = uBreakVelRev = uHoldVelRev = 0.0f;
  ceilingCountsRev = uBreakCountsRev = uHoldCountsRev = 0;

  lastGoodUHoldAbs  = 0.0f;
  lastGoodUBreakAbs = 0.0f;

  bestAvgVel = 0.0f;
  bestCounts = 0;
}

#endif
