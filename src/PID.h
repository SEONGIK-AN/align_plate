#ifndef _PID_H_
#define _PID_H_
#include"Arduino.h"
#include<stdint.h>

class PID {
    public:
        PID();
        PID(double Kp, double Ki, double Kd);
        ~PID() {};

    private:
        double _Kp;
        double _Ki;
        double _Kd;
        double _setPoint;
        double _error;
        double _lastError;
        double _integrationError;
        uint32_t _previousTime;

    public:
        void setPID(double Kp, double Ki, double Kd);
        void setPoint(double setPoint);
        double getOutput(double input);
};

#endif