#ifndef _STEPMOTOR_H_
#define _STEPMOTOR_H_
#include "Arduino.h"
#include <stdint.h>

class StepMotor {
  public:
  // Generator & Destructor
    StepMotor(uint8_t enaPin, uint8_t dirPin, uint8_t pulPin);
    StepMotor(uint8_t enaPin, uint8_t dirPin, uint8_t pulPin, bool isInverse);
    StepMotor(uint8_t enaPin, uint8_t dirPin, uint8_t pulPin, uint16_t Interval);
    StepMotor(uint8_t enaPin, uint8_t dirPin, uint8_t pulPin, uint16_t interval, bool isInverse);
    ~StepMotor() {};

  private:
  // Parameters for step motor
    uint8_t ENA;
    uint8_t DIR;
    uint8_t PUL;
    bool direction;
    bool interrupt;
    uint16_t division;
    uint16_t interval;

  public:
  // User functions
    void begin();
    void setDivision(uint16_t div);
    void setInterval(uint16_t time);
    void setDirection(bool Direction);
    void inverseDirection();
    void stop();
    void resume();
    void writePulse(int32_t pulse);
    void writeDeg(float degree);
    void writeRatio(float ratio);
    void writeDisp(float disp, float dispPerPulse);
};

#endif