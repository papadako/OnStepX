// -----------------------------------------------------------------------------------
// axis servo DC PE motor driver

#include "PE.h"

#ifdef SERVO_PE_PRESENT

#include "../../../../../gpioEx/GpioEx.h"

// Teensy 4.1 Pin-38 PWM emulation disabled for Mesu builds so that we don't mess things up
// I think the Pin-38 PWM emulation should be an option anyway
#if defined(ARDUINO_TEENSY41) && !defined(MESU_MAXPCB4) && defined(AXIS1_STEP_PIN) && AXIS1_STEP_PIN == 38 && defined(SERVO_ANALOG_WRITE_FREQUENCY)
  // this is only for pin 38 of a Teensy4.1
  IntervalTimer itimer4;
  uint16_t _pwm38_period = 0;
  uint8_t _pwm38_toggle = 0;
  float _base_freq_divider = SERVO_ANALOG_WRITE_FREQUENCY/(1.0F/(ANALOG_WRITE_RANGE/1000000.0F));

  void PWM38_HWTIMER() {
    if (_pwm38_period == 0 || _pwm38_period == ANALOG_WRITE_RANGE) {
      itimer4.update(ANALOG_WRITE_RANGE/_base_freq_divider);
      digitalWriteF(38, _pwm38_period == ANALOG_WRITE_RANGE);
    } else {
      if (!_pwm38_toggle) {
        itimer4.update(_pwm38_period/_base_freq_divider);
        digitalWriteF(38, LOW);
      } else {
        itimer4.update((ANALOG_WRITE_RANGE - _pwm38_period)/_base_freq_divider);
        digitalWriteF(38, HIGH);
      }
    }
    _pwm38_toggle = !_pwm38_toggle;
  }
  #define analogWritePin38(x) _pwm38_period = x
#endif

ServoPE::ServoPE(uint8_t axisNumber, const ServoPins *Pins, const ServoSettings *Settings, float pwmMinimum, float pwmMaximum)
                 :ServoDcDriver(axisNumber, Pins, Settings, pwmMinimum, pwmMaximum) {
  if (axisNumber < 1 || axisNumber > 9) return;

  strcpy(axisPrefix, " Axis_ServoPE, ");
  axisPrefix[5] = '0' + axisNumber;
}

bool ServoPE::init(bool reverse) {
  if (!ServoDriver::init(reverse)) return false;

  #if DEBUG == VERBOSE
    VF("MSG:"); V(axisPrefix);
    V("pins dir="); if (Pins->ph1 == OFF) VF("OFF"); else V(Pins->ph1);
    V(", pwm="); if (Pins->ph2 == OFF) VLF("OFF"); else VL(Pins->ph2);
  #endif

  // init default driver control pins
  pinModeEx(Pins->ph1, OUTPUT);
  digitalWriteF(Pins->ph1, Pins->ph1State); // either ph1 or direction, state should default to inactive
  pinModeEx(Pins->ph2, OUTPUT);
  digitalWriteF(Pins->ph2, Pins->ph2State); // either ph2 or step (PWM,) state should default to inactive

  // if this is a T4.1 and we're using a PE driver and ph2 == 38, assume its a MaxPCB4 and make our own PWM on that pin
  #ifdef analogWritePin38
    if (driverModel == SERVO_PE && Pins->ph2 == 38) {
      itimer4.priority(0);
      itimer4.begin(PWM38_HWTIMER, 100);
      VF("MSG: ServoDriver"); V(axisNumber); VLF(", emulating PWM on pin 38");
    }
  #endif

  // set PWM frequency
  #ifdef SERVO_ANALOG_WRITE_FREQUENCY
    VF("MSG:"); V(axisPrefix); VF("setting control pins analog frequency "); VL(SERVO_ANALOG_WRITE_FREQUENCY);
    analogWriteFrequency(Pins->ph2, SERVO_ANALOG_WRITE_FREQUENCY);
  #endif

  // set PWM bits
  #ifdef SERVO_ANALOG_WRITE_RESOLUTION
    VF("MSG:"); V(axisPrefix); VF("setting control pins analog bits "); VL(SERVO_ANALOG_WRITE_RESOLUTION);
    analogWriteResolution(Pins->ph2, SERVO_ANALOG_WRITE_RESOLUTION);
  #endif

  return true;
}

// enable or disable the driver using the enable pin or other method
void ServoPE::enable(bool state) {
  enabled = state;

  VF("MSG:"); V(axisPrefix);
  if (!enabled) {
    int32_t power = 0;

    VF("PE outputs off");
    digitalWriteF(Pins->ph1, Pins->ph1State);
    if (Pins->ph2State == HIGH) power = SERVO_ANALOG_WRITE_RANGE; else power = 0;
    #ifdef analogWritePin38
      if (Pins->ph2 == 38) analogWritePin38(round(power)); else
    #endif
    analogWrite(Pins->ph2, round(power));

    if (enablePin != SHARED) {
      VF(" and powered down using enable pin");
      digitalWriteF(enablePin, !enabledState);
    }
    VLF("");
  } else {
    if (enablePin != SHARED) {
      VLF("powered up using enable pin");
      digitalWriteF(enablePin, enabledState);
    }
  }

  velocityRamp = 0.0F;

  #if defined(SERVO_DIRECTION_CHANGE_DEAD_TIME)
    deadTimeState = DEAD_TIME_IDLE;
  #endif

  ServoDriver::updateStatus();
}

// Clamp helper to keep PWM within the valid range (0..SERVO_ANALOG_WRITE_RANGE)
static inline int clampPower(long p) {
  if (p < 0) return 0;
  if (p > (long)SERVO_ANALOG_WRITE_RANGE) return (int)SERVO_ANALOG_WRITE_RANGE;
  return (int)p;
}

void ServoPE::pwmUpdate(long power) {
  // No output if the driver is disabled
  if (!enabled) {
    #if defined(SERVO_DIRECTION_CHANGE_DEAD_TIME)
      deadTimeState = DEAD_TIME_IDLE;
    #endif
    return;
  }

  // determine requested direction
  const bool forwardReq = (motorDirection == (reversed ? DIR_REVERSE : DIR_FORWARD));
  const bool reverseReq = (motorDirection == (reversed ? DIR_FORWARD : DIR_REVERSE));
  const bool neutralReq = !(forwardReq || reverseReq);

  // clamp power to valid range
  int p = clampPower(power);

  // determine desired PH1 state
  bool desiredPh1 = Pins->ph1State;
  if (reverseReq) {
    desiredPh1 = !Pins->ph1State;
  } else if (neutralReq) {
    desiredPh1 = Pins->ph1State;
  }

  // -------- Dead-time handling (optional) --------
  #if defined(SERVO_DIRECTION_CHANGE_DEAD_TIME)
    // handle dead time state machine (when changing directions)
    if (deadTimeState == DEAD_TIME_ACTIVE) {
      uint32_t elapsed = micros() - deadTimeStart;
      if (elapsed < SERVO_PE_DEAD_TIME_US) {
        // Still in dead time - maintain safe state
        digitalWriteF(Pins->ph1, Pins->ph1State);  // Default direction
        const int idle = (Pins->ph2State == HIGH) ? (int)SERVO_ANALOG_WRITE_RANGE : 0;
        analogWritePh2(Pins->ph2, idle);
        return;
      } else {
        // Dead time completed - apply new direction
        deadTimeState = DEAD_TIME_IDLE;
        digitalWriteF(Pins->ph1, desiredDirection);
      }
    }
  #endif

  // get current PH1 state
  bool currentPh1 = (digitalReadEx(Pins->ph1) == HIGH);
  bool ph1NeedsToggle = (currentPh1 != desiredPh1);

  // safe direction change protocol
  if (ph1NeedsToggle && p > 0) {
    // Ramp down first - set PH2 to inactive
    const int idle = (Pins->ph2State == HIGH) ? (int)SERVO_ANALOG_WRITE_RANGE : 0;
    analogWritePh2(Pins->ph2, idle);
    return;  // Wait for next call with p == 0
  }

  // start direction change when PWM is idle
  if (ph1NeedsToggle && p == 0) {
    #if defined(SERVO_DIRECTION_CHANGE_DEAD_TIME)
      // Enter timed dead time
      deadTimeState = DEAD_TIME_ACTIVE;
      deadTimeStart = micros();
      desiredDirection = desiredPh1;  // Store for after dead time

      // Safe state during dead time
      digitalWriteF(Pins->ph1, Pins->ph1State);  // Default direction
      const int idle = (Pins->ph2State == HIGH) ? (int)SERVO_ANALOG_WRITE_RANGE : 0;
      analogWritePh2(Pins->ph2, idle);
      return;
    #else
      // No timed dead time: flip direction immediately once idle
      digitalWriteF(Pins->ph1, desiredPh1);
      // PH2 already idle; proceed to normal path below
    #endif
  }

  // handle neutral state
  if (neutralReq) {
    digitalWriteF(Pins->ph1, desiredPh1);
    const int idle = (Pins->ph2State == HIGH) ? (int)SERVO_ANALOG_WRITE_RANGE : 0;
    analogWritePh2(Pins->ph2, idle);
    return;
  }

  // normal operation
  digitalWriteF(Pins->ph1, desiredPh1);

  // apply PH2 polarity
  if (Pins->ph2State == HIGH) {
    p = (int)SERVO_ANALOG_WRITE_RANGE - p;
  }

  analogWritePh2(Pins->ph2, p);
}

// update status info. for driver
void ServoPE::updateStatus() {
  if (statusMode == LOW || statusMode == HIGH) {
    status.fault = digitalReadEx(faultPin) == statusMode;
  }
}

#endif
