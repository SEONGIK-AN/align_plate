#include "./stepMotor.h"

StepMotor :: StepMotor(uint8_t enaPin, uint8_t dirPin, uint8_t pulPin)
: ENA(enaPin), DIR(dirPin), PUL(pulPin), direction(false), interrupt(false), division(1), interval(50) {}

StepMotor :: StepMotor(uint8_t enaPin, uint8_t dirPin, uint8_t pulPin, bool isInverse) 
: ENA(enaPin), DIR(dirPin), PUL(pulPin), direction(isInverse), interrupt(false), division(1), interval(50) {}

StepMotor :: StepMotor(uint8_t enaPin, uint8_t dirPin, uint8_t pulPin, uint16_t Interval) 
: ENA(enaPin), DIR(dirPin), PUL(pulPin), direction(false), interrupt(false), division(1), interval(Interval) {}

StepMotor :: StepMotor(uint8_t enaPin, uint8_t dirPin, uint8_t pulPin, uint16_t Interval, bool isInverse)
: ENA(enaPin), DIR(dirPin), PUL(pulPin), direction(isInverse), interrupt(false), division(1), interval(Interval) {} 

void StepMotor :: begin() {
  pinMode(ENA, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  
  if (direction) {
    digitalWrite(DIR, HIGH);
  } else {
    digitalWrite(DIR, LOW);
  }

  if (interrupt) {
    digitalWrite(ENA, HIGH);
  } else {
    digitalWrite(ENA, LOW);
  }
}

void StepMotor :: setDivision(uint16_t div) {
  switch (div)
  {
    case 1: {division = 1; break;}
    case 2: {division = 2; break;}
    case 4: {division = 4; break;}
    case 8: {division = 8; break;}
    case 16: {division = 16; break;}
    case 32: {division = 32; break;}
    case 64: {division = 64; break;}
    case 128: {division = 128; break;}
    case 256: {division = 256; break;}
    default: {division = 1; break;}
  }
}

void StepMotor :: setInterval(uint16_t time) {
  interval = time;
}

void StepMotor :: setDirection(bool Direction) {
  direction = Direction;
  if (direction) {
    digitalWrite(DIR, HIGH);
  } else {
    digitalWrite(DIR, LOW);
  }
}

void StepMotor :: inverseDirection() {
  direction = !direction;
  if (direction) {
    digitalWrite(DIR, HIGH);
  } else {
    digitalWrite(DIR, LOW);
  }
}

void StepMotor :: stop() {
  interrupt = true;
  digitalWrite(ENA, HIGH);
}

void StepMotor :: resume() {
  interrupt = false;
  digitalWrite(ENA, LOW);
}

void StepMotor :: writePulse(int32_t pulse) {
  if(pulse < 0) inverseDirection();

  for (uint32_t i = 0; i < abs(pulse); i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(interval);
    digitalWrite(PUL, LOW);
    delayMicroseconds(interval);
  }

  if(pulse < 0) inverseDirection();
}

void StepMotor :: writeDeg(float degree) {
  if(degree < 0) inverseDirection();

  uint32_t rep = (uint32_t)round(abs(degree) * (float)division / 1.8 );
  for (uint32_t i = 0; i < rep; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(interval);
    digitalWrite(PUL, LOW);
  }

  if(degree < 0) inverseDirection();
}

void StepMotor :: writeDisp(float disp, float dispPerRev) {
  if(disp < 0) inverseDirection();
  
  uint32_t rep = (uint32_t)round(abs((disp * (float)division) / dispPerRev * 200.0));
  for (uint32_t i = 0; i < rep; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(interval);
    digitalWrite(PUL, LOW);
  }

  if(disp < 0) inverseDirection();
}

void StepMotor :: writeRatio(float ratio) {
  if(ratio < 0) inverseDirection();
  
  int32_t rep = (int32_t)round(ratio * 360 / 1.8 / division);

  for (int32_t i = 0; i < rep; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(interval);
    digitalWrite(PUL, LOW);
  }

  if(ratio < 0) inverseDirection();
}