// -----------------------------------------------------------------------------------
// axis servo DC motor driver
#pragma once

#include <Arduino.h>
#include "../../../../../../Common.h"

#ifdef SERVO_PE_PRESENT

#include "../DcServoDriver.h"

// default for typical hobbyist hbridges
#ifndef SERVO_PE_DEAD_TIME_US
#define SERVO_PE_DEAD_TIME_US 25  // 25 microseconds
#endif

class ServoPE : public ServoDcDriver {
  public:
    // constructor
    ServoPE(uint8_t axisNumber, const ServoPins *Pins, const ServoSettings *Settings, float pwmMinimum, float pwmMaximum);

    // decodes driver model and sets up the pin modes
    bool init(bool reverse);

    // enable or disable the driver using the enable pin or other method
    void enable(bool state);

    // update status info. for driver
    void updateStatus();

    // get the driver name
    const char* name() {
      if (driverModel == SERVO_EE) return "DC Enable/Enable"; else
      if (driverModel == SERVO_PE) return "DC Phase/Enable" ; else
      return "?";
    }

  private:
    // motor control pwm update
    // \param power in SERVO_ANALOG_WRITE_RANGE units
    void pwmUpdate(long power);

      enum DeadTimeState {
        DEAD_TIME_IDLE = 0,
        DEAD_TIME_ACTIVE
      };

    DeadTimeState deadTimeState = DEAD_TIME_IDLE;
    uint32_t deadTimeStart = 0;
    bool desiredDirection = false;

    void analogWritePh2(uint8_t pin, int power) {
      #ifdef analogWritePin38
        if (pin == 38) analogWritePin38(power);
        else analogWrite(pin, power);
      #else
        analogWrite(pin, power);
      #endif
    }
};

#endif
