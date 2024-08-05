/* Reference
  Kalman Filter:
    https://en.wikipedia.org/wiki/Kalman_filter
  Kalman.h:
    https://blog.naver.com/heennavi1004/10183064672
*/

#ifndef _KALMAN_FILLTER_H_
#define _KALMAN_FILLTER_H_

class Kalman {
    public:
        Kalman();
        Kalman(float Q_Angle, float Q_Bias, float R_Measure);
        ~Kalman() {};
    private:
        float Q_angle;
        float Q_bias;
        float R_measure;

        float angle;
        float bias;
        float rate;
        
        float P[2][2];
        float K[2];
        float y;
        float S;
    public:
        void setVal(float Qangle, float Qbias, float Rmeasure);
        float getRate();
        float getAngle(float newAngle, float newRate, float dt);
};
#endif