/// @ref
/*
  How to use Wire.h file:
    http://adam-meyer.com/arduino/L3G4200D
    
  Address of MPU-6050:
    https://eduino.kr/product/detail.html?product_no=54&cate_no=27
    https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
  
  Quaternion:
    https://blog.naver.com/ysahn2k/221410891895
    https://blog.naver.com/PostView.nhn?isHttpsRedirect=true&blogId=ysahn2k&logNo=221410878981
    https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=jswcomkr&logNo=221551661440
    https://m.blog.naver.com/spinx85/140120555548
*/

#ifndef __MPU6050__
#define __MPU6050__
#define RAD2DEG 57.29577951
#define DEG2RAD 0.017453292
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <math.h>
#include "kalman.h"

class MPU6050 : public TwoWire,  Kalman
{
  public:
    // Generator & Destructor
    MPU6050();
    MPU6050(float kalman_Q_angle, float Q_bias, float R_measure);
    MPU6050(uint16_t accRange);
    MPU6050(uint16_t accRange, uint16_t gyroRange);
    MPU6050(uint16_t accRange, uint16_t gyroRange, float kalman_Q_angle, float Q_bias, float R_measure);
    MPU6050(uint8_t deviceAddress, uint16_t accRange, uint16_t gyroRange);
    ~MPU6050(){};

  private:
    // Address of MPU6050
    uint8_t MPU;
    uint8_t resetAddress;
    uint8_t accRangeAddress;
    uint8_t gyroRangeAddress;
    uint8_t accAddress;
    uint8_t gyroAddress;

    // Parameters for gyro sensor and accelerometer
    uint16_t accRange;
    uint16_t gyroRange;
    uint32_t currentTime, previousTime;
    float alpha;
    float ax, ay, az;
    float eax, eay;
    float gdotx, gdoty, gdotz;
    float gx, gy, gz;
    float egx, egy, egz;
    float ePitch, eRoll, eYaw;
    float aAngleX, aAngleY;
    float gAngleX, gAngleY, gAngleZ;
    float aq[4]; // Quaternion of accelerometer
    float gq[4]; // Quaternion of gyro sensor
    float q[4]; // Quaternion after complementary filter

    // Functions for use in the class
    void readReg(uint8_t deviceAddress, uint8_t address);
    void writeReg(uint8_t deviceAddress, uint8_t address, uint8_t val);
    float getAccSensitivity();
    float getGyroSensitivity();

  public:
    // User functions
    void begin();
    void initialize_Q();
    void setAccRange(uint16_t range);
    void setGyroRange(uint16_t range);
    void setRatio(float ratio);
    void readData(); // If you use the Euler's method, use this function
    void readQuaternion(); // If you use the quaternion method, use this function

    // Get angle values using Euler's method
    void getError();
    float getPitch();
    float getYaw();
    float getRoll();

    // Get angle values using quaternions
    float getPitch_Q();
    float getYaw_Q();
    float getRoll_Q();
    
    // Get angle values using Euler's method and apply the kalman filter
    float getPitch_K();
    float getYaw_K();
    float getRoll_K();

    // Get angle values using quaternions and apply the kalman filter
    float getPitch_QK();
    float getYaw_QK();
    float getRoll_QK();

    float getErrorPitch();
    float getErrorYaw();
    float getErrorRoll();
};

#endif