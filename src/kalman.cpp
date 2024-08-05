#include "kalman.h"

Kalman :: Kalman() 
: Q_angle(0.001), Q_bias(0.003), R_measure(0.03) {}

Kalman :: Kalman(float Q_Angle, float Q_Bias, float R_Measure) 
: Q_angle(Q_Angle), Q_bias(Q_Bias), R_measure(R_Measure) {}

void Kalman :: setVal(float Qangle, float Qbias, float Rmeasure) {
    Q_angle = Qangle;
    Q_bias = Qbias;
    R_measure = Rmeasure;
}

float Kalman :: getRate() {return rate;}

float Kalman :: getAngle(float newAngle, float newRate, float dt) {

    // Update predicted state estimate
    rate = newRate - bias; // dot{theta}_{k-1|k-1} - dot{theta}_b 
    angle += dt * rate; // theta_{k|k-1} = theta_{k-1|k-1} + rate * dt 

    // Update predicted (a priori) estimate covariance
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Calculate innovation (or pre-fit residual) covariance
    S = P[0][0] + R_measure;
    if (S == 0) {
        S = 0.001;
    }

    // Update optimal Kalman gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate innovation or measurement pre-fit residual
    y = newAngle - angle;

    //Get updated (a posteriori) state estimate
    angle += K[0] * y;
    bias  += K[1] * y;

    // Get updated (a posteriori) estimate covariance
    float P00 = P[0][0];
    float P01 = P[0][1];
    P[0][0] -= K[0] * P00;
    P[0][1] -= K[0] * P01;
    P[1][0] -= K[1] * P00;
    P[1][1] -= K[1] * P01;

    return angle;
}