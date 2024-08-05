#include "MPU6050.h"

// Define generator
MPU6050 :: MPU6050() 
: MPU(0x68), resetAddress(0x6B), accRangeAddress(0x1C), gyroRangeAddress(0x3B), gyroAddress(0x43), 
accRange(2), gyroRange(2000), alpha(0.96) {}

MPU6050 :: MPU6050(float Q_angle, float Q_bias, float R_measure) 
: Kalman(Q_angle, Q_bias, R_measure), MPU(0x68), resetAddress(0x6B), accRangeAddress(0x1C), gyroRangeAddress(0x3B), gyroAddress(0x43), 
accRange(2), gyroRange(2000), alpha(0.96) {}

MPU6050 :: MPU6050(uint16_t accRange)
: MPU(0x68), resetAddress(0x6B), accRangeAddress(0x1C), gyroRangeAddress(0x3B), gyroAddress(0x43), 
accRange(accRange), gyroRange(2000), alpha(0.96) {}

MPU6050 :: MPU6050(uint16_t accRange, uint16_t gyroRange) 
: MPU(0x68), resetAddress(0x6B), accRangeAddress(0x1C), gyroRangeAddress(0x3B), gyroAddress(0x43), 
accRange(accRange), gyroRange(gyroRange), alpha(0.96) {}

MPU6050 :: MPU6050(uint16_t accRange, uint16_t gyroRange, float Q_angle, float Q_bias, float R_measure) 
: Kalman(Q_angle, Q_bias, R_measure), MPU(0x68), resetAddress(0x6B), accRangeAddress(0x1C), gyroRangeAddress(0x3B), gyroAddress(0x43), 
accRange(accRange), gyroRange(gyroRange), alpha(0.96) {}

MPU6050 :: MPU6050(uint8_t deviceAddress, uint16_t accRange, uint16_t gyroRange)
: MPU(deviceAddress), resetAddress(0x6B), accRangeAddress(0x1C), gyroRangeAddress(0x3B), gyroAddress(0x43), 
accRange(accRange), gyroRange(gyroRange), alpha(0.96) {}

// Define internal functions
void MPU6050 :: readReg(uint8_t deviceAddress, uint8_t address) {
    Wire.beginTransmission(deviceAddress); 
    Wire.write(address);
    Wire.endTransmission(false);
}

void MPU6050 :: writeReg(uint8_t deviceAddress, uint8_t address, uint8_t val) {
    Wire.beginTransmission(deviceAddress); 
    Wire.write(address);
    Wire.write(val);
    Wire.endTransmission(true);
}

float MPU6050 :: getAccSensitivity() {
  return (32768.0f / accRange);
}

float MPU6050 :: getGyroSensitivity() {
  return (32768.0f / gyroRange);
}

// Define user functions
void MPU6050 :: begin() {
  Wire.begin();
  writeReg(MPU, resetAddress, 0x00);
  
  if(accRange == 2) {
    writeReg(MPU, accRangeAddress, 0x00);
  } else if(accRange == 4) {
    writeReg(MPU, accRangeAddress, 0x10);
  } else if (accRange == 8) {
    writeReg(MPU, accRangeAddress, 0x20);
  } else if (accRange == 16) {
    writeReg(MPU, accRangeAddress, 0x30);
  } else {
    accRange = 2;
    writeReg(MPU, accRangeAddress, 0x00);
  }

  if(gyroRange == 250) {
    writeReg(MPU, gyroRangeAddress, 0x00);
  } else if(gyroRange == 500) {
    writeReg(MPU, gyroRangeAddress, 0x10);
  } else if (gyroRange == 1000) {
    writeReg(MPU, gyroRangeAddress, 0x20);
  } else if (gyroRange == 2000) {
    writeReg(MPU, gyroRangeAddress, 0x30);
  } else {
    gyroRange = 2000;
    writeReg(MPU, gyroRangeAddress, 0x00);
  }
  // Initial lize the quaternion
  gq[0] = 1.0f;
  gq[1] = 0.0f;
  gq[2] = 0.0f;
  gq[3] = 0.0f;
}

void MPU6050 :: initialize_Q() {
  double errorPitch = 0.0;
  double errorYaw = 0.0;
  double errorRoll = 0.0;

  for (uint16_t i = 0; i < 500; i++) {
    readQuaternion();
    errorPitch += getRoll_Q();
    errorYaw += getYaw_Q();
    errorRoll += getRoll_Q();
  }
  errorPitch /= 500;
  errorYaw /= 500;
  errorRoll /= 500;

  ePitch = (float)errorPitch;
  eYaw = (float)errorYaw;
  eRoll = (float)errorRoll;
}
void MPU6050 :: setAccRange(uint16_t range) {
  accRange = range;
}

void MPU6050 :: setGyroRange(uint16_t range) {
  gyroRange = range;
}

void MPU6050 :: setRatio(float ratio) {
  if(ratio >= 1) alpha = ratio;
}

void MPU6050 :: readData() {
  readReg(MPU, accAddress);
  Wire.requestFrom(MPU, (uint8_t)6);
  ax = (Wire.read() << 8 | Wire.read()) / getAccSensitivity();
  ay = (Wire.read() << 8 | Wire.read()) / getAccSensitivity();
  az = (Wire.read() << 8 | Wire.read()) / getAccSensitivity();

  aAngleX = atan(ay/sqrt(pow(ax, 2) + pow(az, 2))) * RAD2DEG - eax;
  aAngleY = atan(-ax/sqrt(pow(ay, 2) + pow(az, 2))) * RAD2DEG - eay;

  readReg(MPU, gyroAddress);
  previousTime = currentTime;
  currentTime = millis();

  Wire.requestFrom(MPU, (uint8_t)6);
  gdotx = (Wire.read() << 8 | Wire.read()) / getGyroSensitivity();
  gdoty = (Wire.read() << 8 | Wire.read()) / getGyroSensitivity();
  gdotz = (Wire.read() << 8 | Wire.read()) / getGyroSensitivity();

  gAngleX += gdotx * (currentTime - previousTime) / 1000.0f - egx;
  gAngleY += gdoty * (currentTime - previousTime) / 1000.0f - egy;
  gAngleZ += gdotz * (currentTime - previousTime) / 1000.0f - egz;
}

void MPU6050 :: readQuaternion() {
  // Read raw data
  previousTime = currentTime;
  currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0f;
  float norm;
  float qa, qb, qc;

  readReg(MPU, accAddress);
  Wire.requestFrom(MPU, (uint8_t)6);
  ax = (Wire.read() << 8 | Wire.read()) / getAccSensitivity();
  ay = (Wire.read() << 8 | Wire.read()) / getAccSensitivity();
  az = (Wire.read() << 8 | Wire.read()) / getAccSensitivity();

  readReg(MPU, gyroAddress);
  Wire.requestFrom(MPU, (uint8_t)6);
  gdotx = (Wire.read() << 8 | Wire.read()) / getGyroSensitivity();
  gdoty = (Wire.read() << 8 | Wire.read()) / getGyroSensitivity();
  gdotz = (Wire.read() << 8 | Wire.read()) / getGyroSensitivity();
  gx = gdotx * DEG2RAD * dt;
  gy = gdoty * DEG2RAD * dt;
  gz = gdotz * DEG2RAD * dt;

  // Get quaternion from gyro sensor
  norm = sqrt(pow(gq[0], 2) + pow(gq[1], 2) + pow(gq[2], 2) + pow(gq[3], 2));
  qa = gq[0];
  qb = gq[1];
  qc = gq[2];
  gq[0] += (-qb * gx - qc * gy - gq[3] * gz) / norm;
  gq[1] += (qa * gx - qc * gz - gq[3] * gy) / norm;
  gq[2] += (qa * gy - qb * gz - gq[3] * gx) / norm; 
  gq[4] += (qa * gz - qb * gy - qc * gx) / norm;

  // Get quaternion from accelerometer sensor
  norm = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  if (norm == 0.0f) return;
  ax /= norm;
  ay /= norm;
  az /= norm;

  aq[0] = sqrt(az+1.0f) / 2.0f;
  aq[1] = -ay / sqrt(2.0f*(az+1));
  aq[2] = ax / sqrt(2.0f*(az+1));
  aq[3] = 0.0f;

  for (uint8_t i = 0; i < 4; i++)
    q[i] = alpha * gq[i] + (1-alpha) * aq[i];
}

void MPU6050 :: getError() {
  float aax, aay, aaz;
  float ggx, ggy, ggz;

  for (uint8_t i = 0; i < 200; i++) {
    readReg(MPU, accAddress);
    Wire.requestFrom(MPU, (uint8_t)6);
    aax = (Wire.read() << 8 | Wire.read()) / getAccSensitivity();
    aay = (Wire.read() << 8 | Wire.read()) / getAccSensitivity();
    aaz = (Wire.read() << 8 | Wire.read()) / getAccSensitivity();

    eax += atan(aay/sqrt(pow(aax, 2) + pow(aaz, 2))) * RAD2DEG;
    eay += atan(-aax/sqrt(pow(aay, 2) + pow(aaz, 2))) * RAD2DEG;
  }
  previousTime = currentTime;
  currentTime = millis();
  for (uint8_t i = 0; i < 200; i++) {
    readReg(MPU, gyroAddress);

    Wire.requestFrom(MPU, (uint8_t)6);
    ggx = (Wire.read() << 8 | Wire.read()) / getGyroSensitivity();
    ggy = (Wire.read() << 8 | Wire.read()) / getGyroSensitivity();
    ggz = (Wire.read() << 8 | Wire.read()) / getGyroSensitivity();

    egx += ggx * (currentTime - previousTime) / 1000.0f;
    egy += ggy * (currentTime - previousTime) / 1000.0f;
    egz += ggz * (currentTime - previousTime) / 1000.0f;
  }
  eax /= 200.0f;
  eay /= 200.0f;

  egx /= 200.0f;
  egy /= 200.0f;
  egz /= 200.0f;
}

float MPU6050 :: getPitch() {
  return alpha * gAngleY + (1.0 - alpha) * aAngleY;
}

float MPU6050 :: getYaw() {
  return gAngleZ;
}

float MPU6050 :: getRoll() {
  return alpha * gAngleX + (1.0 - alpha) * aAngleX;
}

float MPU6050 :: getPitch_Q() {
  return asinf(2.0f * (q[0]*q[1] - q[2]*q[3])) * RAD2DEG;
}

float MPU6050 :: getYaw_Q() {
  return atan2f(2.0f * (q[1]*q[3] + q[0]*q[2]), (-q[1]*q[1] - q[2]*q[2] + q[3]*q[3] + q[0] * q[0])) * RAD2DEG;
}

float MPU6050 :: getRoll_Q() {
  return atan2f(2.0f * (q[1]*q[2] + q[0]*q[3]), (-q[1]*q[1] + q[2]*q[2] - q[3]*q[3] + q[0] * q[0])) * RAD2DEG;
}

float MPU6050 :: getPitch_K() {
  float angle = getPitch();
  float dt = (currentTime - previousTime) / 1000.0f;
  return getAngle(angle, gdotx, dt);
}

float MPU6050 :: getYaw_K() {
  float angle = getPitch();
  float dt = (currentTime - previousTime) / 1000.0f;
  return getAngle(angle, gdoty, dt);
}

float MPU6050 :: getRoll_K() {
  float angle = getRoll();
  float dt = (currentTime - previousTime) / 1000.0f;
  return getAngle(angle, gdotz, dt);
}

float MPU6050 :: getPitch_QK() {
  float angle = getPitch_Q();
  float dt = (currentTime - previousTime) / 1000.0f;
  return getAngle(angle, gdotx, dt) - ePitch;
}

float MPU6050 :: getYaw_QK() {
  float angle = getPitch_Q();
  float dt = (currentTime - previousTime) / 1000.0f;
  return getAngle(angle, gdoty, dt) - eYaw;
}

float MPU6050 :: getRoll_QK() {
  float angle = getRoll_Q();
  float dt = (currentTime - previousTime) / 1000.0f;
  return getAngle(angle, gdotz, dt) - eRoll;
}

float MPU6050 :: getErrorPitch() {
  return ePitch;
}

float MPU6050 :: getErrorYaw() {
  return eYaw;
}

float MPU6050 :: getErrorRoll() {
  return eRoll;
}