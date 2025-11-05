// -----------------------------------------------------------------------------------
// Servo Tracking Velocity Calibration - Overview
//
// This routine performs a 3-phase, bidirectional calibration to determine:
//   • ceiling: an upper bound % that produces sustained motion from rest
//   • u_break: minimum % that starts moving from rest (old "floor"),
//              with a +step safety margin to guard against noise
//   • u_hold : minimum % that maintains motion after a kick (old "tracking")
//
// Search strategy (per direction):
// 1) ceiling: staircase UP from a tiny % until sustained motion is detected.
// 2) u_break: start at ceiling and staircase DOWN in fixed steps **down to 0%** until stall;
//             the last steady step before stall (+ one uBreakStep) is u_break (capped at ceiling).
// 3) u_hold : kick at u_break, then staircase DOWN below it; the last steady
//             step before stall is u_hold.
//
// After both directions are calibrated, an imbalance check is performed.

#include "TrackingVelocity.h"
#include <math.h>  // fabsf, fmaxf, fminf

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
// stop tracking
#ifdef TRACK_AUTOSTART
  #undef TRACK_AUTOSTART
#endif

// --- Small inline helpers ---------------------------------------------------

inline void ServoCalibrateTrackingVelocity::logDirPrefix() const {
  V(axisPrefix); VF(calibrationDirectionIsForward ? " FWD" : " REV");
}

ServoCalibrateTrackingVelocity::Measurement
ServoCalibrateTrackingVelocity::measureWindow(unsigned long now,
                                              unsigned long startMs,
                                              long startTicks,
                                              long curTicks,
                                              unsigned long windowMs) const {
  Measurement m{false, 0.0f, 0};
  unsigned long dt = now - startMs;
  if (dt < windowMs) return m;
  m.ready  = true;
  m.counts = curTicks - startTicks;
  m.avgVel = (float)m.counts / (dt / 1000.0f);
  return m;
}

void ServoCalibrateTrackingVelocity::kickAndQueue(CalibrationState nextState,
                                                  float kickPctAbs,
                                                  float testPctAbsSigned) {
  queueNextTest(nextState, testPctAbsSigned);
  setVelocity(dirSign() * kickPctAbs); // kick at anchor (ceiling/u_break)
  motorState = MOTOR_ACCELERATING;
  lastStateChangeTime = currentTime;
  lastVelocityTime = currentTime;
  lastVelocityMeasurement = 0;
}

// --- Class ------------------------------------------------------------------

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

  // staircase step sizes (absolute %)
  stairStep  = fabsf(SERVO_CALIBRATION_STAIR_STEP_PERCENT);
  uBreakStep = fabsf(SERVO_CALIBRATION_UBREAK_STEP_PERCENT);
  uHoldStep  = fabsf(SERVO_CALIBRATION_UHOLD_STEP_PERCENT);

  // setup the queue
  hasQueuedTest = false;
  queuedState = CALIBRATION_IDLE;
  queuedVelocity = 0.0f;

  // stop the motor
  setVelocity(0);
  startSettling();
}

void ServoCalibrateTrackingVelocity::start(float /*trackingFrequency*/, long /*instrumentCoordinateSteps*/) {
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

// Private helper methods

void ServoCalibrateTrackingVelocity::handleMotorState() {
  float currentVelocity = 0.0f;
  switch (motorState) {
    case MOTOR_SETTLING:
      // check also the velocity
      currentVelocity = calculateInstantaneousVelocity();
      if (currentTime - settleStartTime >= SERVO_CALIBRATION_MOTOR_SETTLE_TIME) {
        motorState = MOTOR_STOPPED;
        V(axisPrefix); VF(": Motor settled to STOPPED_STATE"); VF(" v="); V(currentVelocity); VL(" steps/s");

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

          logDirPrefix(); VF(": STEADY_STATE  @%="); V(calibrationVelocity);
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

          logDirPrefix(); VF(": STOPPED_STATE  @%="); V(calibrationVelocity);
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
  const float sign = dirSign();

  if (motorState == MOTOR_RUNNING_STEADY) {
    // sustained movement window for "ceiling found"
    unsigned long steadyElapsedMs = currentTime - steadySinceMs;
    if (steadyElapsedMs < SERVO_CALIBRATION_STICTION_SAMPLE_INTERVAL_MS) return;

    float elapsedSec = steadyElapsedMs / 1000.0f;
    long  dCounts    = currentTicks - steadyStartTicks;
    float avgVel     = dCounts / elapsedSec;

    if (fabsf(avgVel) > SERVO_CALIBRATION_MIN_DETECTABLE_VELOCITY) {
      float absVelocity = fabsf(calibrationVelocity);
      ceiling[dir()] = absVelocity;
      measCeilVel[dir()] = avgVel;
      measCeilCnt[dir()] = dCounts;
      everMoved[dir()] = true;

      logDirPrefix(); VF(" ceiling found @ %="); V(calibrationVelocity);
      VF(" (v="); V(avgVel); VL(" steps/s)");

      setVelocity(0);
      settleStartTime = currentTime;
      motorState = MOTOR_SETTLING;

      transitionToUBreak(); // set up downward sweep
      return;
    }
  }

  // step up if not yet moving or after wait
  if (motorState == MOTOR_STOPPED || motorState == MOTOR_RUNNING_STEADY) {
    if (currentTime - lastVelocityTime < SERVO_CALIBRATION_STICTION_SAMPLE_INTERVAL_MS) return;

    float nextAbs = fminf(fabsf(calibrationVelocity) + stairStep,
                          SERVO_CALIBRATION_STOP_VELOCITY_PERCENT);
    calibrationVelocity = sign * nextAbs;

    if (nextAbs >= SERVO_CALIBRATION_STOP_VELOCITY_PERCENT) {
      ceiling[dir()] = SERVO_CALIBRATION_STOP_VELOCITY_PERCENT;
      if (!everMoved[dir()]) {
        logDirPrefix(); VLF(" ceiling not found up to limit – skipping u_break/u_hold");
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

  const float ceilingAbs = ceiling[dir()];

  // Sweep bounds: down from ceiling **all the way to 0%**
  calibrationMaxVelocity = ceilingAbs; // start here
  calibrationMinVelocity = 0.0f;       // lowest allowed is zero

  // Clamp step to a sensible fraction so tiny ceilings don't skip range in one hop
  if (ceilingAbs > 0.0f) {
    uBreakStep = fminf(uBreakStep, 0.25f * ceilingAbs);
  }

  // Start at ceiling
  float startAbs = calibrationMaxVelocity;
  if (startAbs <= 0.0f) startAbs = stairStep; // avoid zero
  calibrationVelocity = dirSign() * startAbs;

  uBreakIters[dir()] = 0;
  lastGoodUBreakAbs[dir()] = 0.0f;

  startTest(calibrationVelocity);
}

// u_break via staircase (DOWNWARD): start at ceiling and decrement by uBreakStep to 0.
// The u_break is the lowest % that still moves from rest, but we add +uBreakStep safety
// (and cap at ceiling) to avoid sitting right on the noise edge.
void ServoCalibrateTrackingVelocity::processUBreak() {
  if (++uBreakIters[dir()] > SERVO_CALIBRATION_REFINE_MAX_ITERATIONS) {
    VLF("WARN: u_break sweep iteration cap reached; using last good / ceiling (+step safety)");
    float lastGood = (lastGoodUBreakAbs[dir()] > 0.0f) ? lastGoodUBreakAbs[dir()] : calibrationMaxVelocity;
    float uBreakSafe = fminf(calibrationMaxVelocity, lastGood + uBreakStep);
    uBreak[dir()] = uBreakSafe;

    // measured velocity & counts (fall back to last instant if needed)
    if (measUBreakVel[dir()] == 0.0f) {
      measUBreakVel[dir()] = lastVelocityMeasurement;
      measUBreakCnt[dir()] = lastDeltaCounts;
    }

    // proceed to u_hold search
    calibrationState = CALIBRATION_UHOLD_SEARCH;
    uHoldIters[dir()] = 0;
    lastGoodUHoldAbs[dir()] = uBreakSafe;

    // queue a first test just below u_break, kick then test
    const float firstBelow = fmaxf(stairStep, uBreakSafe - uHoldStep);
    kickAndQueue(CALIBRATION_UHOLD_SEARCH, uBreakSafe, dirSign() * firstBelow);
    return;
  }

  if (motorState != MOTOR_RUNNING_STEADY && motorState != MOTOR_STOPPED) return;

  if (motorState == MOTOR_RUNNING_STEADY) {
    // windowed average for consistent reporting
    auto m = measureWindow(currentTime, calibrationStepStartTime,
                           calibrationStepStartTicks, currentTicks,
                           SERVO_CALIBRATION_VELOCITY_MEASURE_WINDOW_MS);
    if (!m.ready) return;

    lastGoodUBreakAbs[dir()] = fabsf(calibrationVelocity);
    measUBreakVel[dir()]     = m.avgVel;
    measUBreakCnt[dir()]     = m.counts;

    logDirPrefix(); VF(" u_break sweep: MOVE @ "); V(calibrationVelocity); VF("%, v≈");
    V(m.avgVel); VL(" steps/s");

    // Next downward step (to zero)
    float nextAbs = fmaxf(0.0f, lastGoodUBreakAbs[dir()] - uBreakStep);

    // If we cannot step further down (at 0), accept lastGood (+step safety) as u_break and enter u_hold
    if (nextAbs >= lastGoodUBreakAbs[dir()] - 1e-6f) {
      float uBreakSafe = fminf(calibrationMaxVelocity, lastGoodUBreakAbs[dir()] + uBreakStep);
      uBreak[dir()] = uBreakSafe;

      calibrationState = CALIBRATION_UHOLD_SEARCH;
      uHoldIters[dir()] = 0;
      lastGoodUHoldAbs[dir()] = uBreakSafe;

      float below = fmaxf(0.0f + stairStep, uBreakSafe - uHoldStep);
      kickAndQueue(CALIBRATION_UHOLD_SEARCH, uBreakSafe, dirSign() * below);
      return;
    }

    // Continue the downward sweep
    startTest(dirSign() * nextAbs);
    return;
  }

  // If we STOPPED, current step is too low -> u_break = (last good + step), capped at ceiling.
  if (motorState == MOTOR_STOPPED) {
    float lastGood = (lastGoodUBreakAbs[dir()] > 0.0f) ? lastGoodUBreakAbs[dir()] : calibrationMaxVelocity;
    float uBreakSafe = fminf(calibrationMaxVelocity, lastGood + uBreakStep);
    uBreak[dir()] = uBreakSafe;

    logDirPrefix(); VF(" u_break sweep: STOP @ "); V(fabsf(calibrationVelocity)); VF("% -> u_break=");
    V(uBreakSafe); VLF("% (safe)");

    // Enter u_hold search
    calibrationState = CALIBRATION_UHOLD_SEARCH;
    uHoldIters[dir()] = 0;
    lastGoodUHoldAbs[dir()] = uBreakSafe;

    float below = fmaxf(0.0f + stairStep, uBreakSafe - uHoldStep);
    kickAndQueue(CALIBRATION_UHOLD_SEARCH, uBreakSafe, dirSign() * below);
    return;
  }
}

// u_hold via staircase: kick at u_break; then step down by uHoldStep until stall.
// Last good (steady) is taken as u_hold; also records measured avg.
void ServoCalibrateTrackingVelocity::processUHoldSearch() {
  const float uBreakAbs = uBreak[dir()];  // declare before any early-returns

  // safety cap
  if (++uHoldIters[dir()] > SERVO_CALIBRATION_REFINE_MAX_ITERATIONS) {
    float fallback = (lastGoodUHoldAbs[dir()] > 0.0f) ? lastGoodUHoldAbs[dir()] : uBreakAbs;
    uHold[dir()] = fallback;
    if (measUHoldVel[dir()] == 0.0f) { // prefer any recorded steady sample
      measUHoldVel[dir()] = (measUBreakVel[dir()] != 0.0f) ? measUBreakVel[dir()] : 0.0f;
      measUHoldCnt[dir()] = (measUBreakCnt[dir()] != 0   ) ? measUBreakCnt[dir()] : 0;
    }
    // common tail
    setVelocity(0);
    startSettling();
    if (calibrationDirectionIsForward) {
      calibrationDirectionIsForward = false;
      calibrationState = CALIBRATION_CEILING;
      queueNextTest(CALIBRATION_CEILING, -SERVO_CALIBRATION_START_VELOCITY_PERCENT);
    } else {
      calibrationState = CALIBRATION_CHECK_IMBALANCE;
      processImbalanceCheck();
    }
    return;
  }

  if (motorState != MOTOR_RUNNING_STEADY && motorState != MOTOR_STOPPED) return;

  if (motorState == MOTOR_RUNNING_STEADY) {
    auto m = measureWindow(currentTime, calibrationStepStartTime,
                           calibrationStepStartTicks, currentTicks,
                           SERVO_CALIBRATION_VELOCITY_MEASURE_WINDOW_MS);
    if (!m.ready) return;

    lastGoodUHoldAbs[dir()] = fabsf(calibrationVelocity);
    measUHoldVel[dir()]     = m.avgVel;
    measUHoldCnt[dir()]     = m.counts;

    // Next notch down; practical lower limit relative to u_break -> accept current
    float nextAbs = fmaxf(0.0f + stairStep, lastGoodUHoldAbs[dir()] - uHoldStep);
    const float minAllowed = tolFor(uBreakAbs);
    if (nextAbs <= minAllowed) {
      uHold[dir()] = lastGoodUHoldAbs[dir()];

      // common tail
      setVelocity(0);
      startSettling();
      if (calibrationDirectionIsForward) {
        calibrationDirectionIsForward = false;
        calibrationState = CALIBRATION_CEILING;
        queueNextTest(CALIBRATION_CEILING, -SERVO_CALIBRATION_START_VELOCITY_PERCENT);
      } else {
        calibrationState = CALIBRATION_CHECK_IMBALANCE;
        processImbalanceCheck();
      }
      return;
    }

    // Kick at u_break, then queue test at nextAbs
    kickAndQueue(CALIBRATION_UHOLD_SEARCH, uBreakAbs, dirSign() * nextAbs);
    return;
  }

  // MOTOR_STOPPED -> the last commanded value was too low; use last good as u_hold
  uHold[dir()] = (lastGoodUHoldAbs[dir()] > 0.0f) ? lastGoodUHoldAbs[dir()] : uBreakAbs;
  if (measUHoldVel[dir()] == 0.0f) { // fallback to u_break measured sample
    measUHoldVel[dir()] = measUBreakVel[dir()];
    measUHoldCnt[dir()] = measUBreakCnt[dir()];
  }

  // common tail
  setVelocity(0);
  startSettling();
  if (calibrationDirectionIsForward) {
    calibrationDirectionIsForward = false;
    calibrationState = CALIBRATION_CEILING;
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
  if (uHold[FWD] > 0 && uHold[REV] > 0) {
    float avgHold = (uHold[FWD] + uHold[REV]) / 2.0f;
    float imbalance = fabsf(uHold[FWD] - uHold[REV]) / avgHold * 100.0f;

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
  VF("ERR: "); V(axisPrefix); VL(" Calibration failed, resetting");

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

  logDirPrefix(); VF(" Start test @ %="); V(calibrationVelocity); VL("");
}

void ServoCalibrateTrackingVelocity::setVelocity(float velocityPercent) {
  // Clamp percent
  experimentVelocity = clampPct(velocityPercent); // telemetry in percent
}

// Getters
float ServoCalibrateTrackingVelocity::getCeiling(bool forward) {
  return ceiling[forward ? FWD : REV];
}
float ServoCalibrateTrackingVelocity::getUBreak(bool forward) {
  return uBreak[forward ? FWD : REV];
}
float ServoCalibrateTrackingVelocity::getUHold(bool forward) {
  return uHold[forward ? FWD : REV];
}

// --- Compact report helpers (unchanged layout, new arrays) ------------------
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
  prPctVelCnt(ceiling[FWD], measCeilVel[FWD],  measCeilCnt[FWD]);  VF("  | ");
  prPctVelCnt(uBreak[FWD],  measUBreakVel[FWD],measUBreakCnt[FWD]);VF("  | ");
  prPctVelCnt(uHold[FWD],   measUHoldVel[FWD], measUHoldCnt[FWD]); VL("");

  // REV row
  prDirHeader("REV");
  prPctVelCnt(ceiling[REV], measCeilVel[REV],  measCeilCnt[REV]);  VF("  | ");
  prPctVelCnt(uBreak[REV],  measUBreakVel[REV],measUBreakCnt[REV]);VF("  | ");
  prPctVelCnt(uHold[REV],   measUHoldVel[REV], measUHoldCnt[REV]); VL("");

  // Imbalance (use magnitudes of %)
  VF("Δ u_break: "); V(fabsf(uBreak[FWD] - uBreak[REV])); VF("%, ");
  VF("Δ u_hold: ");  V(fabsf(uHold[FWD]  - uHold[REV]));  VLF("%");

  if (uHold[FWD] >= SERVO_CALIBRATION_STOP_VELOCITY_PERCENT - 0.1f) {
    VLF("WARN: FWD u_hold at calibration limit");
  }
  if (uHold[REV] >= SERVO_CALIBRATION_STOP_VELOCITY_PERCENT - 0.1f) {
    VLF("WARN: REV u_hold at calibration limit");
  }

  VF("=== End Report ===\n");
}


void ServoCalibrateTrackingVelocity::resetCalibrationValues(void) {
  for (int i=0;i<2;i++) {
    ceiling[i] = SERVO_CALIBRATION_STOP_VELOCITY_PERCENT;
    uBreak[i]  = 0.0f;
    uHold[i]   = 0.0f;

    measCeilVel[i] = 0.0f;  measCeilCnt[i]  = 0;
    measUBreakVel[i]= 0.0f; measUBreakCnt[i]= 0;
    measUHoldVel[i] = 0.0f; measUHoldCnt[i] = 0;

    everMoved[i] = false;
    uBreakIters[i] = 0;
    uHoldIters[i]  = 0;
    lastGoodUBreakAbs[i] = 0.0f;
    lastGoodUHoldAbs[i]  = 0.0f;
  }
}

#endif
