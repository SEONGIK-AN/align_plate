#include "PID.h"

PID :: PID()
: _Kp(2), _Ki(5), _Kd(1) {}

PID :: PID(double Kp, double Ki, double Kd)
: _Kp(Kp), _Ki(Ki), _Kd(Kd) {}

void PID :: setPID(double Kp, double Ki, double Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

void PID :: setPoint(double setPoint) {
    _setPoint = setPoint;
}

double PID :: getOutput(double input) {
    uint32_t currentTime = millis();
    double error = (_setPoint - input);
    double elapsedTime = (double)(currentTime - _previousTime) / 1000.0;
    _integrationError += error * elapsedTime;
    double output = _Kp * error + _Ki * _integrationError + _Kd * (error - _lastError) / elapsedTime;

    _lastError = error;
    _previousTime = currentTime;

    return output;
}