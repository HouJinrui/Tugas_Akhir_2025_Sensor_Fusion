#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <EtherCard.h>
#include <Modbus.h>
#include <ModbusIP_ENC28J60.h>

// ------------------ KONFIGURASI AS5600 ------------------
#define AS5600_ADDR 0x36
#define RAW_ANGLE_HIGH 0x0C
#define RAW_ANGLE_LOW  0x0D

// ------------------ MODBUS TCP CONFIG -------------------
const uint8_t ip_addr[] PROGMEM = {192, 168, 1, 102};
const uint8_t mac_addr[] PROGMEM = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEB};

// ------------------ REGISTER INDEX ----------------------
#define MPU_REG 0
#define SHAFT_REG 1
#define FINAL_FUSION_REG 2
#define RESPONSE_TIME_REG 3

// ------------------ OBJEK SENSOR ------------------------
MPU6050 mpu;
ModbusIP mb;

// ------------------ VARIABEL GLOBAL ---------------------
int16_t AccX, AccY, AccZ, GyroX, GyroY, GyroZ;

float prevAngleDegAS5600 = 0.0;
float motorAngleTotal = 0.0;
float prevFusedAngle = 0.0;
uint16_t responseTime = 0;

bool resetRequested = true;
float rollOffset = 0.0;
float shaftOffset = 0.0;

// Konfigurasi Kalman Filter - moved to PROGMEM where possible
const float GYRO_SENSITIVITY = 131.0;
const float GEAR_RATIO = 19.0;
const float ANGLE_THRESHOLD = 1.0;
const float MOVEMENT_THRESHOLD = 15.0;
const float STABILITY_THRESHOLD = 0.3;
const uint8_t STABLE_COUNT_REQUIRED = 8;
const float FUSION_W_KALMAN = 0.15;
const float FUSION_W_SHAFT = 0.85;

// Kalman variables - reduced precision
float Q_angle = 0.001;
float Q_gyro = 0.003;
float R_angle = 0.03;
float x_angle = 0;
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;

// Enhanced response time calculation variables - optimized
enum MovementState : uint8_t { IDLE, TRACKING, TIMING };
MovementState currentState = IDLE;
unsigned long movementStartTime = 0;
float movementStartAngle = 0.0;
uint16_t lastValidResponseTime = 0;
float maxMovementDeviation = 0.0;
uint8_t stableCount = 0;
bool responseTimeCalculated = false;

float gyroOffsetX = 0.0;
unsigned long lastTime = 0;

// ------------------ FUNGSI BACA AS5600 ------------------
uint16_t readAS5600RawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;
  
  uint8_t high = Wire.read();
  uint8_t low = Wire.read();
  return ((high << 8) | low) & 0x0FFF;
}

// ------------ FUNGSI PERHITUNGAN ROLL YANG DIPERBAIKI ------------
float calculateRollAngle(int16_t accX, int16_t accY, int16_t accZ) {
  float ax = (float)accX;
  float ay = (float)accY;
  float az = (float)accZ;
  
  float acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
  
  ax = ax / acc_magnitude;
  ay = ay / acc_magnitude;
  az = az / acc_magnitude;
  
  float denominator = sqrt(ax*ax + az*az);
  
  if (denominator < 0.001) {
    return 0.0;
  }
  
  float rollAngle = atan2(ay, denominator) * (180.0 / PI);
  return rollAngle;
}

// ------------------ RESET ANGLES ------------------------
void resetAngles() {
  mpu.getAcceleration(&AccX, &AccY, &AccZ);
  rollOffset = calculateRollAngle(AccX, AccY, AccZ);
  
  Serial.print(F("Roll offset: "));
  Serial.print(rollOffset, 2);
  Serial.println(F("°"));
  
  uint16_t rawAngleInit = readAS5600RawAngle();
  if (rawAngleInit != 0xFFFF) {
    prevAngleDegAS5600 = (rawAngleInit * 360.0) / 4096.0;
    motorAngleTotal = 0.0;
    shaftOffset = 0.0;
  }
  
  // Reset Kalman Filter
  x_angle = 0;
  x_bias = 0;
  P_00 = 0;
  P_01 = 0;
  P_10 = 0;
  P_11 = 0;
  
  // Reset all tracking variables
  prevFusedAngle = 0.0;
  resetRequested = false;
  currentState = IDLE;
  lastValidResponseTime = 0;
  maxMovementDeviation = 0.0;
  stableCount = 0;
  responseTimeCalculated = false;
  
  Serial.println(F("=== SISTEM RESET ==="));
}

// ------------------ GYRO CALIBRATION --------------------
void calibrateGyro() {
  const int CALIB_SAMPLES = 500;
  long sumX = 0; // Use long for accumulation
  
  Serial.println(F("Kalibrasi gyroscope..."));
  Serial.println(F("Pastikan sensor diam!"));
  
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    mpu.getRotation(&GyroX, &GyroY, &GyroZ);
    sumX += GyroX;
    
    if (i % 100 == 0) {
      Serial.print(F("Progress: "));
      Serial.print((i * 100) / CALIB_SAMPLES);
      Serial.println(F("%"));
    }
    
    delay(3);
  }
  
  gyroOffsetX = (float)sumX / CALIB_SAMPLES;
  Serial.print(F("Gyro offset X: "));
  Serial.println(gyroOffsetX);
  Serial.println(F("Kalibrasi selesai!"));
}

// ------------------ NORMALIZE ANGLE ---------------------
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// ------------------ KALMAN UPDATE -----------------------
float kalmanUpdate(float accAngle, float gyroRate, float dt) {
  // Predict state
  x_angle += dt * (gyroRate - x_bias);
  
  // Predict covariance
  P_00 += dt * (dt * P_11 - P_01 - P_10 + Q_angle);
  P_01 -= dt * P_11;
  P_10 -= dt * P_11;
  P_11 += Q_gyro * dt;

  // Calculate innovation
  float y = accAngle - x_angle;
  float S = P_00 + R_angle;

  // Calculate Kalman gain
  float K_0 = P_00 / S;
  float K_1 = P_10 / S;

  // Update state
  x_angle += K_0 * y;
  x_bias += K_1 * y;

  // Update covariance
  float P_00_temp = P_00;
  P_00 -= K_0 * P_00_temp;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00_temp;
  P_11 -= K_1 * P_01;

  return x_angle;
}

// ------------------ SETUP -------------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println(F("=== INIT SISTEM ==="));

  // Initialize MPU6050
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 gagal!"));
    while(1);
  }
  
  Serial.println(F("MPU6050 OK!"));

  // Test AS5600
  uint16_t testRead = readAS5600RawAngle();
  if (testRead == 0xFFFF) {
    Serial.println(F("WARNING: AS5600 gagal!"));
  } else {
    Serial.println(F("AS5600 OK!"));
  }

  // Calibrate sensors
  calibrateGyro();
  resetAngles();

  // Modbus setup - copy from PROGMEM
  uint8_t ip[4];
  uint8_t mac[6];
  memcpy_P(ip, ip_addr, 4);
  memcpy_P(mac, mac_addr, 6);
  
  mb.config(mac, ip);
  mb.addHreg(MPU_REG);
  mb.addHreg(SHAFT_REG);
  mb.addHreg(FINAL_FUSION_REG);
  mb.addHreg(RESPONSE_TIME_REG);

  Serial.println(F("=== SISTEM SIAP ==="));
  Serial.println(F("Kirim 'r' untuk reset"));
}

// ------------------ MAIN LOOP ---------------------------
void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  if (resetRequested) resetAngles();

  // Read sensor data
  mpu.getMotion6(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);
  
  // Calculate MPU angle
  float accRoll = calculateRollAngle(AccX, AccY, AccZ) - rollOffset;
  float gyroRate = ((float)GyroX - gyroOffsetX) / GYRO_SENSITIVITY;
  float kalmanAngle = kalmanUpdate(accRoll, gyroRate, dt);
  kalmanAngle = normalizeAngle(kalmanAngle);

  // Read shaft angle (AS5600)
  float shaftAngle = 0.0;
  uint16_t rawAngle = readAS5600RawAngle();
  if (rawAngle != 0xFFFF) {
    float angleDegAS5600 = (rawAngle * 360.0) / 4096.0;
    float deltaAngle = angleDegAS5600 - prevAngleDegAS5600;
    
    // Handle 360° wrap-around
    if (deltaAngle > 180) deltaAngle -= 360;
    else if (deltaAngle < -180) deltaAngle += 360;
    
    motorAngleTotal += -deltaAngle;
    prevAngleDegAS5600 = angleDegAS5600;
    shaftAngle = normalizeAngle((motorAngleTotal / GEAR_RATIO) - shaftOffset);
  }

  // Sensor fusion
  float fusedAngle = (FUSION_W_KALMAN * kalmanAngle) + (FUSION_W_SHAFT * shaftAngle);
  fusedAngle = normalizeAngle(fusedAngle);

  // Response Time Calculation
  float angleChange = abs(fusedAngle - prevFusedAngle);
  if (angleChange > 180) angleChange = 360 - angleChange;
  
  switch (currentState) {
    case IDLE:
      if (angleChange >= ANGLE_THRESHOLD) {
        currentState = TRACKING;
        movementStartTime = currentTime;
        movementStartAngle = fusedAngle;
        maxMovementDeviation = 0.0;
        stableCount = 0;
        responseTimeCalculated = false;
        
        Serial.print(F(">>> MOVEMENT START: "));
        Serial.print(fusedAngle, 2);
        Serial.println(F("°"));
      }
      break;
      
    case TRACKING:
      {
        float currentDeviation = abs(fusedAngle - movementStartAngle);
        if (currentDeviation > 180) currentDeviation = 360 - currentDeviation;
        if (currentDeviation > maxMovementDeviation) {
          maxMovementDeviation = currentDeviation;
        }
        
        if (maxMovementDeviation >= MOVEMENT_THRESHOLD) {
          currentState = TIMING;
          Serial.print(F(">>> 15° REACHED: "));
          Serial.print(maxMovementDeviation, 1);
          Serial.println(F("°"));
        }
        
        if (angleChange < STABILITY_THRESHOLD) {
          stableCount++;
          if (stableCount >= STABLE_COUNT_REQUIRED) {
            Serial.print(F(">>> STOPPED <15°: "));
            Serial.print(maxMovementDeviation, 1);
            Serial.println(F("°"));
            currentState = IDLE;
            stableCount = 0;
          }
        } else {
          stableCount = 0;
        }
      }
      break;
      
    case TIMING:
      {
        float currentDeviation = abs(fusedAngle - movementStartAngle);
        if (currentDeviation > 180) currentDeviation = 360 - currentDeviation;
        if (currentDeviation > maxMovementDeviation) {
          maxMovementDeviation = currentDeviation;
        }
        
        if (angleChange < STABILITY_THRESHOLD) {
          stableCount++;
          
          if (stableCount >= STABLE_COUNT_REQUIRED) {
            responseTime = currentTime - movementStartTime;
            lastValidResponseTime = responseTime;
            responseTimeCalculated = true;
            
            Serial.println(F("=================="));
            Serial.println(F(">>> STABLE - RT DONE! <<<"));
            Serial.print(F("Start: "));
            Serial.print(movementStartAngle, 2);
            Serial.println(F("°"));
            Serial.print(F("End: "));
            Serial.print(fusedAngle, 2);
            Serial.println(F("°"));
            Serial.print(F("Total: "));
            Serial.print(maxMovementDeviation, 1);
            Serial.println(F("°"));
            Serial.print(F("RT: "));
            Serial.print(lastValidResponseTime);
            Serial.println(F(" ms"));
            Serial.println(F("=================="));
            
            currentState = IDLE;
            stableCount = 0;
          }
        } else {
          stableCount = 0;
        }
      }
      break;
  }

  prevFusedAngle = fusedAngle;

  // Update Modbus registers
  mb.Hreg(MPU_REG, (int16_t)(kalmanAngle * 100));
  mb.Hreg(SHAFT_REG, (int16_t)(shaftAngle * 100));
  mb.Hreg(FINAL_FUSION_REG, (int16_t)(fusedAngle * 100));
  mb.Hreg(RESPONSE_TIME_REG, lastValidResponseTime);

  // Serial Monitor Display (shortened)
  Serial.print(F("K:"));
  Serial.print(kalmanAngle, 1);
  Serial.print(F("° S:"));
  Serial.print(shaftAngle, 1);
  Serial.print(F("° F:"));
  Serial.print(fusedAngle, 2);
  Serial.print(F("° St:"));
  
  switch(currentState) {
    case IDLE:
      Serial.print(F("IDLE"));
      break;
    case TRACKING:
      Serial.print(F("TRACK("));
      Serial.print(maxMovementDeviation, 1);
      Serial.print(F("°)"));
      break;
    case TIMING:
      Serial.print(F("TIME("));
      Serial.print(maxMovementDeviation, 1);
      Serial.print(F("°,"));
      Serial.print(stableCount);
      Serial.print(F("/"));
      Serial.print(STABLE_COUNT_REQUIRED);
      Serial.print(F(")"));
      break;
  }
  
  Serial.print(F(" RT:"));
  if (responseTimeCalculated) {
    Serial.print(lastValidResponseTime);
    Serial.println(F("ms ***"));
    responseTimeCalculated = false;
  } else {
    Serial.print(lastValidResponseTime);
    Serial.println(F("ms"));
  }

  // Modbus and reset handling
  mb.task();
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      resetRequested = true;
      Serial.println(F(">>> RESET <<<"));
    }
  }

  delay(10);
}