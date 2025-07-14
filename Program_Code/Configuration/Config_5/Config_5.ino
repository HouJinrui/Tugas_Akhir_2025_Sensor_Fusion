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
const int MPU = 0;              // MPU fusion output
const int SHAFT_REG = 1;        // Shaft angle
const int FINAL_FUSION_REG = 2; // Final fusion value
const int RESPONSE_TIME_REG = 3; // Response time (ms)

// ------------------ OBJEK SENSOR ------------------------
MPU6050 mpu;
ModbusIP mb;

// ------------------ VARIABEL GLOBAL ---------------------
int16_t AccX, AccY, AccZ;
float AngleRoll;

float prevAngleDegAS5600 = 0.0;
float motorAngleTotal = 0.0;    // total rotasi motor (derajat)
float prevFusedAngle = 0.0;     // Nilai fusion sebelumnya
float responseTime = 0;         // Waktu respon

bool resetRequested = true;
bool angleChanged = false;
float rollOffset = 0.0;
float shaftOffset = 0.0;

// Weighted average configuration - AS5600 has more influence (60%)
const float MPU_WEIGHT = 0.40;  // Weight for MPU (40%)
const float SHAFT_WEIGHT = 0.60; // Weight for Shaft (60%)
const float ANGLE_THRESHOLD = 1.0; // Movement detection threshold
const float GEAR_RATIO = 19.0;  // Gearbox ratio 1:19
const float MOVEMENT_THRESHOLD = 15.0; // Minimum movement to trigger response time calculation
const float STABILITY_THRESHOLD = 0.3; // Stability detection threshold
const int STABLE_COUNT_REQUIRED = 8;  // Consecutive stable readings

// Enhanced response time calculation variables
enum MovementState { IDLE, TRACKING, TIMING };
MovementState currentState = IDLE;
unsigned long movementStartTime = 0;
float movementStartAngle = 0.0;
float lastValidResponseTime = 0.0;
float maxMovementDeviation = 0.0;
int stableCount = 0;
bool responseTimeCalculated = false;

// ------------------ FUNGSI BACA AS5600 ------------------
uint16_t readAS5600RawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;  // Return error value
  
  uint8_t high = Wire.read();
  uint8_t low = Wire.read();
  return ((high << 8) | low) & 0x0FFF; // 12-bit (0–4095)
}

// ------------------ FUNGSI RESET SUDUT ------------------
void resetAngles() {
  // Reset sudut MPU6050
  mpu.getAcceleration(&AccX, &AccY, &AccZ);
  float fAccX = (float)AccX;
  float fAccY = (float)AccY;
  float fAccZ = (float)AccZ;
  rollOffset = atan2(fAccY, sqrt(fAccX * fAccX + fAccZ * fAccZ)) * (180.0 / PI);
  
  // Reset sudut shaft (AS5600)
  uint16_t rawAngleInit = readAS5600RawAngle();
  if (rawAngleInit != 0xFFFF) {
    prevAngleDegAS5600 = (rawAngleInit * 360.0) / 4096.0;
    motorAngleTotal = 0.0;
    shaftOffset = 0.0;
  }
  
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

// ------------------ FUNGSI NORMALISASI SUDUT --------------------
float normalizeAngle(float angle) {
  // Normalisasi ke rentang -180 hingga +180 derajat
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// ------------------ SETUP -------------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();  // I2C

  // Inisialisasi MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 tidak terhubung!");
    while (1);
  }
  Serial.println("MPU6050 terhubung!");

  // Konfigurasi Modbus
  uint8_t ip[] = IP_ADDR;
  uint8_t mac[] = MAC_ADDR;
  mb.config(mac, ip);
  mb.addHreg(MPU);
  mb.addHreg(SHAFT_REG);
  mb.addHreg(FINAL_FUSION_REG);
  mb.addHreg(RESPONSE_TIME_REG);

  // Reset sudut awal
  resetAngles();

  Serial.println("=== SISTEM SIAP - RESPONSE TIME TRACKER ===");
  Serial.println("- Timer dimulai saat ada pergerakan >= 1°");
  Serial.println("- Response time dihitung HANYA jika total pergerakan >= 15°");
  Serial.println("- Timer dihitung dari awal pergerakan hingga stabil");
  Serial.println("- Kirim 'r' untuk reset");
  Serial.println("==========================================");
}

// ------------------ LOOP --------------------------------
void loop() {
  unsigned long currentTime = millis();
  
  if (resetRequested) {
    resetAngles();
  }

  // ---------- MPU6050: Sudut Roll ----------
  mpu.getAcceleration(&AccX, &AccY, &AccZ);
  float fAccX = (float)AccX;
  float fAccY = (float)AccY;
  float fAccZ = (float)AccZ;

  float rawRoll = atan2(fAccY, sqrt(fAccX * fAccX + fAccZ * fAccZ)) * (180.0 / PI);
  float mpuRoll = normalizeAngle(rawRoll - rollOffset);

  // ---------- AS5600: Sudut Shaft ----------
  float shaftAngle = 0.0;
  uint16_t rawAngle = readAS5600RawAngle();
  
  if (rawAngle != 0xFFFF) {
    float angleDegAS5600 = (rawAngle * 360.0) / 4096.0;
    float deltaAngle = angleDegAS5600 - prevAngleDegAS5600;

    // Koreksi wrap-around 360°
    if (deltaAngle > 180) deltaAngle -= 360;
    if (deltaAngle < -180) deltaAngle += 360;

    // Balik arah agar sesuai MPU6050 (CCW = +)
    deltaAngle = -deltaAngle;

    motorAngleTotal += deltaAngle;
    prevAngleDegAS5600 = angleDegAS5600;

    shaftAngle = normalizeAngle((motorAngleTotal / GEAR_RATIO) - shaftOffset);
  }

  // ---------- WEIGHTED AVERAGE FUSION ----------
  float fusedAngle = (MPU_WEIGHT * mpuRoll) + (SHAFT_WEIGHT * shaftAngle);
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

  // ---------- Simpan ke Modbus ----------
  mb.Hreg(MPU, (int16_t)(mpuRoll * 100));
  mb.Hreg(SHAFT_REG, (int16_t)(shaftAngle * 100));
  mb.Hreg(FINAL_FUSION_REG, (int16_t)(fusedAngle * 100));
  mb.Hreg(RESPONSE_TIME_REG, (uint16_t)lastValidResponseTime);

  // ---------- Serial Monitor Display ----------
  Serial.print("MPU: ");
  Serial.print(mpuRoll, 1);
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

  // ---------- Handle Modbus ----------
  mb.task();

  // ---------- Check for reset command ----------
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      resetRequested = true;
      Serial.println(">>> RESET DIMINTA <<<");
    }
  }

  delay(50);
}