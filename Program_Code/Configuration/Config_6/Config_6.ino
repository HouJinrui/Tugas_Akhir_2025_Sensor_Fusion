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
#define IP_ADDR {192, 168, 2, 101}
#define MAC_ADDR {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}

// ------------------ REGISTER INDEX ----------------------
const int MPU = 0;              // Kalman filter output
const int SHAFT_REG = 1;         // Shaft angle
const int FINAL_FUSION_REG = 2;  // Final fusion value
const int RESPONSE_TIME_REG = 3; // Response time (ms)

// ------------------ OBJEK SENSOR ------------------------
MPU6050 mpu;
ModbusIP mb;

// ------------------ VARIABEL GLOBAL ---------------------
int16_t AccX, AccY, AccZ, GyroX, GyroY, GyroZ;

float prevAngleDegAS5600 = 0.0;
float motorAngleTotal = 0.0;
float prevFusedAngle = 0.0;
float responseTime = 0;

bool resetRequested = true;
bool angleChanged = false;
float rollOffset = 0.0;
float shaftOffset = 0.0;

// Konfigurasi Kalman Filter
const float GYRO_SENSITIVITY = 131.0;
const float GEAR_RATIO = 19.0;
const float ANGLE_THRESHOLD = 1.0;
const float MOVEMENT_THRESHOLD = 15.0; // Minimum movement to trigger response time calculation
const float STABILITY_THRESHOLD = 0.3; // Stability detection threshold
const int STABLE_COUNT_REQUIRED = 8;  // Consecutive stable readings
const float FUSION_W_KALMAN = 0.25;
const float FUSION_W_SHAFT = 0.75;

// Kalman variables
float Q_angle = 0.001;  // Process noise variance
float Q_gyro = 0.003;   // Gyro noise variance
float R_angle = 0.03;   // Measurement noise variance
float x_angle = 0;      // Estimated angle
float x_bias = 0;       // Gyro bias
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0; // Covariance matrix

// Enhanced response time calculation variables
enum MovementState { IDLE, TRACKING, TIMING };
MovementState currentState = IDLE;
unsigned long movementStartTime = 0;
float movementStartAngle = 0.0;
float lastValidResponseTime = 0.0;
float maxMovementDeviation = 0.0;
int stableCount = 0;
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
  return ((high << 8) | low) & 0x0FFF; // 12-bit value
}

// ------------------ RESET ANGLES ------------------------
void resetAngles() {
  // Reset MPU6050 angle
  mpu.getAcceleration(&AccX, &AccY, &AccZ);
  rollOffset = atan2((float)AccY, sqrt(pow((float)AccX, 2) + pow((float)AccZ, 2))) * (180.0 / PI);
  
  // Reset AS5600 angle
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
  angleChanged = false;
  currentState = IDLE;
  lastValidResponseTime = 0.0;
  maxMovementDeviation = 0.0;
  stableCount = 0;
  responseTimeCalculated = false;
  
  Serial.println("=== SISTEM RESET - SUDUT DIKALIBRASI KE 0 ===");
}

// ------------------ GYRO CALIBRATION --------------------
void calibrateGyro() {
  const int CALIB_SAMPLES = 500;
  float sumX = 0;
  
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    mpu.getRotation(&GyroX, &GyroY, &GyroZ);
    sumX += GyroX;
    delay(3);
  }
  
  gyroOffsetX = sumX / CALIB_SAMPLES;
  Serial.print("Gyro offset X: ");
  Serial.println(gyroOffsetX);
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

  // Initialize MPU6050
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }

  // Calibrate sensors
  calibrateGyro();
  resetAngles();

  // Modbus setup
  uint8_t ip[] = IP_ADDR;
  uint8_t mac[] = MAC_ADDR;
  mb.config(mac, ip);
  mb.addHreg(MPU);
  mb.addHreg(SHAFT_REG);
  mb.addHreg(FINAL_FUSION_REG);
  mb.addHreg(RESPONSE_TIME_REG);

  Serial.println("=== SISTEM SIAP - KALMAN + RESPONSE TIME TRACKER ===");
  Serial.println("- Timer dimulai saat ada pergerakan >= 1°");
  Serial.println("- Response time dihitung HANYA jika total pergerakan >= 15°");
  Serial.println("- Timer dihitung dari awal pergerakan hingga stabil");
  Serial.println("- Kirim 'r' untuk reset");
  Serial.println("==========================================");
}

// ------------------ MAIN LOOP ---------------------------
void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  if (resetRequested) resetAngles();

  // Read sensor data
  mpu.getMotion6(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);
  
  // Calculate MPU angle (Kalman Filter)
  float accRoll = atan2((float)AccY, sqrt(pow((float)AccX, 2) + pow((float)AccZ, 2))) * (180.0 / PI) - rollOffset;
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
    
    motorAngleTotal += -deltaAngle; // Invert direction
    prevAngleDegAS5600 = angleDegAS5600;
    shaftAngle = normalizeAngle((motorAngleTotal / GEAR_RATIO) - shaftOffset);
  }

  // Sensor fusion (AS5600 dominant)
  float fusedAngle = (FUSION_W_KALMAN * kalmanAngle) + (FUSION_W_SHAFT * shaftAngle);
  fusedAngle = normalizeAngle(fusedAngle);

  // ---------- Enhanced Response Time Calculation ----------
  float angleChange = abs(fusedAngle - prevFusedAngle);
  if (angleChange > 180) angleChange = 360 - angleChange;  // Handle wrap-around
  
  switch (currentState) {
    case IDLE:
      if (angleChange >= ANGLE_THRESHOLD) {
        // Movement detected - start tracking from beginning
        currentState = TRACKING;
        movementStartTime = currentTime;  // Timer starts here
        movementStartAngle = fusedAngle;
        maxMovementDeviation = 0.0;
        stableCount = 0;
        responseTimeCalculated = false;
        
        Serial.print(">>> PERGERAKAN TERDETEKSI - Timer dimulai dari: ");
        Serial.print(fusedAngle, 2);
        Serial.println("°");
      }
      break;
      
    case TRACKING:
      {
        // Track maximum deviation from start position
        float currentDeviation = abs(fusedAngle - movementStartAngle);
        if (currentDeviation > 180) currentDeviation = 360 - currentDeviation;
        if (currentDeviation > maxMovementDeviation) {
          maxMovementDeviation = currentDeviation;
        }
        
        // Check if we've reached the 15-degree threshold
        if (maxMovementDeviation >= MOVEMENT_THRESHOLD) {
          currentState = TIMING;
          Serial.print(">>> THRESHOLD 15° TERCAPAI - Mulai menghitung response time! (Total: ");
          Serial.print(maxMovementDeviation, 1);
          Serial.println("°)");
        }
        
        // Check if movement stops before reaching 15 degrees
        if (angleChange < STABILITY_THRESHOLD) {
          stableCount++;
          if (stableCount >= STABLE_COUNT_REQUIRED) {
            Serial.print(">>> Pergerakan berhenti sebelum 15° (");
            Serial.print(maxMovementDeviation, 1);
            Serial.println("°) - Kembali ke IDLE");
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
        // Continue tracking maximum deviation
        float currentDeviation = abs(fusedAngle - movementStartAngle);
        if (currentDeviation > 180) currentDeviation = 360 - currentDeviation;
        if (currentDeviation > maxMovementDeviation) {
          maxMovementDeviation = currentDeviation;
        }
        
        // Check for stability
        if (angleChange < STABILITY_THRESHOLD) {
          stableCount++;
          
          if (stableCount >= STABLE_COUNT_REQUIRED) {
            // System is stable - calculate response time from the very beginning
            responseTime = currentTime - movementStartTime;
            lastValidResponseTime = responseTime;
            angleChanged = true;
            responseTimeCalculated = true;
            
            // Log completion
            Serial.println("==========================================");
            Serial.println(">>> SISTEM STABIL - RESPONSE TIME SELESAI! <<<");
            Serial.print("    Sudut awal: ");
            Serial.print(movementStartAngle, 2);
            Serial.println("°");
            Serial.print("    Sudut akhir: ");
            Serial.print(fusedAngle, 2);
            Serial.println("°");
            Serial.print("    Total pergerakan: ");
            Serial.print(maxMovementDeviation, 1);
            Serial.println("°");
            Serial.print("    RESPONSE TIME: ");
            Serial.print(lastValidResponseTime);
            Serial.println(" ms");
            Serial.println("==========================================");
            
            // Reset to IDLE state
            currentState = IDLE;
            stableCount = 0;
          }
        } else {
          // Reset stability counter if significant movement detected
          stableCount = 0;
        }
      }
      break;
  }

  prevFusedAngle = fusedAngle;

  // Update Modbus registers
  mb.Hreg(MPU, (int16_t)(kalmanAngle * 100));
  mb.Hreg(SHAFT_REG, (int16_t)(shaftAngle * 100));
  mb.Hreg(FINAL_FUSION_REG, (int16_t)(fusedAngle * 100));
  mb.Hreg(RESPONSE_TIME_REG, (uint16_t)lastValidResponseTime);

  // ---------- Serial Monitor Display ----------
  Serial.print("Kalman: ");
  Serial.print(kalmanAngle, 1);
  Serial.print("° | Shaft: ");
  Serial.print(shaftAngle, 1);
  Serial.print("° | Fused: ");
  Serial.print(fusedAngle, 2);
  Serial.print("° | State: ");
  
  switch(currentState) {
    case IDLE:
      Serial.print("IDLE");
      break;
    case TRACKING:
      Serial.print("TRACKING(");
      Serial.print(maxMovementDeviation, 1);
      Serial.print("°)");
      break;
    case TIMING:
      Serial.print("TIMING(");
      Serial.print(maxMovementDeviation, 1);
      Serial.print("°,stable:");
      Serial.print(stableCount);
      Serial.print("/");
      Serial.print(STABLE_COUNT_REQUIRED);
      Serial.print(")");
      break;
  }
  
  Serial.print(" | RT: ");
  if (responseTimeCalculated) {
    Serial.print(lastValidResponseTime);
    Serial.println("ms *** NEW ***");
    responseTimeCalculated = false;
  } else {
    Serial.print(lastValidResponseTime);
    Serial.println("ms");
  }

  // Modbus and reset handling
  mb.task();
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      resetRequested = true;
      Serial.println(">>> RESET DIMINTA <<<");
    }
  }

  delay(10);
}