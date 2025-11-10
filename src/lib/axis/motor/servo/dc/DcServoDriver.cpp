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

  // --- Hysteresis ---
  #ifdef SERVO_HYSTERESIS_ENABLE
    VF("MSG:"); V(axisPrefix);
    VF("Hysteresis: ENABLED  ENTER>="); V((float)SERVO_HYST_ENTER_CPS);
    VF(" cps, EXIT<="); V((float)SERVO_HYST_EXIT_CPS);
    VF(" cps, RESET_ON_DIR_FLIP="); V((int)SERVO_HYST_RESET_ON_DIR_FLIP); VLF("");
  #else
    VF("MSG:"); V(axisPrefix); VLF("Hysteresis: DISABLED");
  #endif

  // --- Stiction kick ---
  #ifdef SERVO_STICTION_KICK
    VF("MSG:"); V(axisPrefix);
    VF("Stiction kick: ENABLED  duration="); V((int)SERVO_STICTION_KICK_MS);
    VF(" ms, multiplier="); V((float)SERVO_STICTION_KICK_PERCENT_MULTIPLIER); VLF("x");
  #else
    VF("MSG:"); V(axisPrefix); VLF("Stiction kick: DISABLED");
  #endif

  // --- Nonlinear mapping near zero ---
  #if SERVO_NONLINEAR_ENABLE
    VF("MSG:"); V(axisPrefix);
    VF("Nonlinear map: ENABLED  gamma="); V((float)SERVO_NONLINEAR_GAMMA);
    #if (SERVO_NONLINEAR_KNEE_PERCENT > 0.0f)
      VF(", knee="); V((float)(SERVO_NONLINEAR_KNEE_PERCENT * 100.0f)); VLF("% of vmax");
    #else
      VF(", knee="); V((float)SERVO_NONLINEAR_KNEE_CPS); VLF(" cps");
    #endif
  #else
    VF("MSG:"); V(axisPrefix); VLF("Nonlinear map: DISABLED");
  #endif

  // --- Sigma-Delta dithering ---
  #ifdef SERVO_SIGMA_DELTA_DITHERING
    VF("MSG:"); V(axisPrefix); VLF("Sigma-Delta dither: ENABLED");
  #else
    VF("MSG:"); V(axisPrefix); VLF("Sigma-Delta dither: DISABLED");
  #endif
}

float ServoDcDriver::setMotorVelocity(float velocity)  {
  velocity = ServoDriver::setMotorVelocity(velocity);

  pwmUpdate(labs(toAnalogRange(velocity)));

  return velocity;
}

#endif
