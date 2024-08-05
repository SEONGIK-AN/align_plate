#include "src.h"

// Determine the function to be used
#define USE_KALMAN true
#define USE_QUATERNION false

// Experimental variable setup
#define Q_ANGLE (float)0.06
#define Q_BIAS (float)0.07
#define Q_MEASURE (float)0.05
#define ACC_RANGE 8 // 2 / 4 / 8 / 16
#define GYRO_RANGE 250 // 250 / 500 / 1000 / 2000
#define PULSE_PERIOD_FOR_MOTOR (const uint16_t)2200  // Microseconds
#define READ_PERIOD_FOR_SENSOR (const uint16_t)100   // Milliseconds
#define NUMBER_OF_REPETITIONS_TO_INITIALIZE 100
#define ALPHA (float)0.96  // Variable for setting the complementary filter ratio
#define K_P 0.25
#define K_I 0.001
#define K_D 0.004
#define UNIT_PULSE (uint32_t)2

// Board setting
#define BAUDRATE 115200
#define ENA_PIN (uint8_t)2
#define DIR_PIN (uint8_t)3
#define PUL_PIN (uint8_t)4
#define ENA2_PIN (uint8_t)5
#define DIR2_PIN (uint8_t)6
#define PUL2_PIN (uint8_t)7

// Parameters for the sensor
float roll, pitch, yaw;

// Class declaration
MPU6050 mpu(Q_ANGLE, Q_BIAS, Q_MEASURE);
StepMotor stm(ENA_PIN, DIR_PIN, PUL_PIN, PULSE_PERIOD_FOR_MOTOR);
PID pidPitch(K_P, K_I, K_D);
PID pidRoll(K_P, K_I, K_D);
PID pidYaw(K_P, K_I, K_D);

void setup() {
  Serial.begin(BAUDRATE);
  mpu.begin();
  mpu.setRatio(ALPHA);
  mpu.setAccRange(ACC_RANGE);
  mpu.setGyroRange(GYRO_RANGE);
  delay(3000);
  PIDsetPoint();
  stm.begin();
  Serial.println();
  Serial.println("##### Initialization complete #####");
  Serial.println();
  delay(1500);
}

void loop() {
#if USE_QUATERNION == true
  mpu.readQuaternion();
#endif

#if USE_QUATERNION == false
  mpu.readData();
#endif

// If you want to calculate by Euler'method, but do not want to apply Kalman filter, use this function.
#if USE_QUATERNION == false && USE_KALMAN == false
  pitch = mpu.getPitch();
  yaw = mpu.getYaw();
  roll = mpu.getRoll();
#endif

// If you want to calculate with quaternions and apply Kalman filter, use this function.
#if USE_QUATERNION == false && USE_KALMAN == true
  pitch = mpu.getPitch_K();
  yaw = mpu.getYaw_K();
  roll = mpu.getRoll_K();
#endif

// If you want to calculate with quaternions but do not want to apply Kalman filter, use this function.
#if USE_QUATERNION == true && USE_KALMAN == false
  pitch = mpu.getPitch_Q();
  yaw = mpu.getYaw_Q();
  roll = mpu.getRoll_Q();
#endif

// If you want to calculate with quaternions and apply Kalman filter, use this function.
#if USE_QUATERNION == true && USE_KALMAN == true
  pitch = mpu.getPitch_QK();
  yaw = mpu.getYaw_QK();
  roll = mpu.getRoll_QK();
#endif

  pitch = pidPitch.getOutput((double)pitch);
  yaw = pidYaw.getOutput((double)yaw);
  roll = pidRoll.getOutput((double)roll);
  Serial.print("Roll: ");
  Serial.print(roll);
  if (roll >= 0) {
    stm.writePulse((int32_t) ( - roll * UNIT_PULSE));
    Serial.print(", Input Pulse: ");
    Serial.println((int32_t)( - roll * UNIT_PULSE));
    mpu.readData();
    roll = mpu.getRoll_K();
    roll = pidRoll.getOutput((double)roll);
  } else {
    stm.writePulse((int32_t) ( - roll * UNIT_PULSE) / 2);
    Serial.print(", Input Pulse: ");
    Serial.println((int32_t)( - roll * UNIT_PULSE) / 2);
    roll = mpu.getRoll_K();
    roll = pidRoll.getOutput((double)roll);
  }

  delay(READ_PERIOD_FOR_SENSOR);
}

void PIDsetPoint() {
  double tempPitch = 0;
  double tempYaw = 0;
  double tempRoll = 0;
  double discountRate = 1.005;
  double newDiscountRate = discountRate;

  Serial.println("Correcting the offset...");
  for (uint16_t i = 0; i < NUMBER_OF_REPETITIONS_TO_INITIALIZE; i++) {
    newDiscountRate *= discountRate;
    #if USE_QUATERNION == true
    mpu.readQuaternion();
    #endif

    #if USE_QUATERNION == false
    mpu.readData();
    #endif

    #if USE_QUATERNION == false && USE_KALMAN == false
    tempPitch += mpu.getPitch() * newDiscountRate;
    tempYaw += mpu.getYaw() * newDiscountRate;
    tempRoll += mpu.getRoll() * newDiscountRate;
    #endif

    #if USE_QUATERNION == false && USE_KALMAN == true
    tempPitch += mpu.getPitch_K() * newDiscountRate;
    tempYaw += mpu.getYaw_K() * newDiscountRate;
    tempRoll += mpu.getRoll_K() * newDiscountRate;
    #endif

    #if USE_QUATERNION == true && USE_KALMAN == false
    tempPitch += mpu.getPitch_Q() * newDiscountRate;
    tempYaw += mpu.getYaw_Q() * newDiscountRate;
    tempRoll += mpu.getRoll_Q() * newDiscountRate;
    #endif

    #if USE_QUATERNION == true && USE_KALMAN == true
    tempPitch += mpu.getPitch_QK() * newDiscountRate;
    tempYaw += mpu.getYaw_QK() * newDiscountRate;
    tempRoll += mpu.getRoll_QK() * newDiscountRate;
    #endif
    if ((uint16_t)(100 * i / NUMBER_OF_REPETITIONS_TO_INITIALIZE) % 10 == 0) {
      Serial.print(100 * i / NUMBER_OF_REPETITIONS_TO_INITIALIZE);
      Serial.println("%");
    }
    delay(READ_PERIOD_FOR_SENSOR);
  }

  double initPitch = (tempPitch / NUMBER_OF_REPETITIONS_TO_INITIALIZE);
  double initYaw = (tempYaw / NUMBER_OF_REPETITIONS_TO_INITIALIZE);
  double initRoll = (tempRoll / NUMBER_OF_REPETITIONS_TO_INITIALIZE) - 2.2;

  pidPitch.setPoint(initPitch);
  pidYaw.setPoint(initYaw);
  pidRoll.setPoint(initRoll);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("############ Set Point ############");
  Serial.print("Pitch: ");
  Serial.println(initPitch);
  Serial.print("Yaw: ");
  Serial.println(initYaw);
  Serial.print("Roll: ");
  Serial.println(initYaw);
  Serial.println();
}