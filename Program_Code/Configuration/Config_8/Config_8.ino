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
const int MPU = 0;             // MPU fusion output
const int SHAFT_REG = 1;       // Shaft angle
const int FINAL_FUSION_REG = 2; // Final fusion value
const int RESPONSE_TIME_REG = 3; // Response time (ms)

// ------------------ OBJEK SENSOR ------------------------
MPU6050 mpu;
ModbusIP mb;

// ------------------ VARIABEL GLOBAL ---------------------
int16_t AccX, AccY, AccZ;
float AngleRoll;

float prevAngleDegAS5600 = 0.0;
float motorAngleTotal = 0.0;     // total rotasi motor (derajat)
float prevFusedAngle = 0.0;      // Nilai fusion sebelumnya
float responseTime = 0;          // Waktu respon

bool resetRequested = true;
bool angleChanged = false; // Flag to indicate if a new response time was calculated
float rollOffset = 0.0;
float shaftOffset = 0.0;

// Weighted average configuration - AS5600 has more influence (60%)
const float MPU_WEIGHT = 0.40;   // Weight for MPU (40%)
const float SHAFT_WEIGHT = 0.60; // Weight for Shaft (60%)
const float ANGLE_CHANGE_THRESHOLD = 1.0; // Minimal angle change to detect potential movement
const float GEAR_RATIO = 19.0;   // Gearbox ratio 1:19
const float MOVEMENT_TRIGGER_THRESHOLD = 15.0; // Minimum movement to trigger response time calculation
const float STABILITY_THRESHOLD = 0.5; // Stability detection threshold (from reference)
const int STABLE_COUNT_REQUIRED = 6;   // Consecutive stable readings (from reference)

// Enhanced response time calculation variables
enum MovementState { IDLE, POTENTIAL_MOVEMENT, TRACKING_RESPONSE };
MovementState currentState = IDLE;
unsigned long movementStartTime = 0;
float movementStartAngle = 0.0;
float lastValidResponseTime = 0.0;
float maxMovementDeviation = 0.0;
int stableCount = 0;

// ------------------ FUNGSI BACA AS5600 ------------------
uint16_t readAS5600RawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;   // Return error value
  
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
  // lastValidResponseTime is NOT reset to 0, per requirement "the time calculated not change to 0 again but still at the last value."
  maxMovementDeviation = 0.0;
  stableCount = 0;
  
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
  Wire.begin();   // I2C

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
  Serial.println("- Mulai hitung saat ada pergerakan >= 1°");
  Serial.println("- Laporkan waktu respon hanya jika pergerakan >= 15°");
  Serial.println("- Berhenti hitung saat sistem stabil");
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
  if (angleChange > 180) angleChange = 360 - angleChange;   // Handle wrap-around for angle change
  
  // Calculate total deviation from the angle when the movement *started* (initially IDLE state)
  float currentDeviationFromStart = abs(fusedAngle - movementStartAngle);
  if (currentDeviationFromStart > 180) currentDeviationFromStart = 360 - currentDeviationFromStart; // Handle wrap-around

  switch (currentState) {
    case IDLE:
      if (angleChange >= ANGLE_CHANGE_THRESHOLD) {
        // Potential movement detected, start tracking from here
        currentState = POTENTIAL_MOVEMENT;
        movementStartTime = currentTime; // Record the initial time of detection
        movementStartAngle = fusedAngle; // Record the angle at the start of potential movement
        maxMovementDeviation = currentDeviationFromStart; // Initialize max deviation
        stableCount = 0;
        Serial.print(">>> POTENTIAL MOVEMENT DETECTED from: ");
        Serial.print(fusedAngle, 2);
        Serial.println("° - Timer initiated for monitoring.");
      }
      break;
      
    case POTENTIAL_MOVEMENT:
      // Update max movement deviation
      if (currentDeviationFromStart > maxMovementDeviation) {
        maxMovementDeviation = currentDeviationFromStart;
      }

      if (maxMovementDeviation >= MOVEMENT_TRIGGER_THRESHOLD) {
        // Movement has exceeded the threshold, start truly tracking response time
        currentState = TRACKING_RESPONSE;
        // Keep movementStartTime as it was from the initial detection
        Serial.println(">>> MOVEMENT THRESHOLD (15°) EXCEEDED - Starting response time calculation!");
      } else {
        // If movement stops before reaching threshold, go back to IDLE
        if (angleChange < STABILITY_THRESHOLD) {
          stableCount++;
          if (stableCount >= STABLE_COUNT_REQUIRED) {
            Serial.print(">>> Movement stopped before reaching 15° (max: ");
            Serial.print(maxMovementDeviation, 1);
            Serial.println("°) - Reset to IDLE.");
            currentState = IDLE;
            stableCount = 0;
            maxMovementDeviation = 0.0; // Reset for next movement
          }
        } else {
          stableCount = 0; // Reset if movement continues to oscillate within small range
        }
      }
      break;

    case TRACKING_RESPONSE:
      // Keep tracking maximum deviation, even if it's already over the threshold
      if (currentDeviationFromStart > maxMovementDeviation) {
        maxMovementDeviation = currentDeviationFromStart;
      }

      // Check if movement has stabilized
      if (angleChange < STABILITY_THRESHOLD) {
        stableCount++;
        
        // Check if we have enough stable readings
        if (stableCount >= STABLE_COUNT_REQUIRED) {
          // Calculate final response time
          responseTime = currentTime - movementStartTime;
          lastValidResponseTime = responseTime;
          angleChanged = true; // Indicate that a new response time has been calculated
          
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
          
          // Reset to IDLE state for the next movement
          currentState = IDLE;
          stableCount = 0;
          maxMovementDeviation = 0.0; // Reset for next movement
        }
      } else {
        // Reset stability counter if significant movement detected
        stableCount = 0;
      }
      break;
  }

  prevFusedAngle = fusedAngle;

  // ---------- Simpan ke Modbus ----------
  mb.Hreg(MPU, (int16_t)(mpuRoll * 100));     // MPU Roll (scaled by 100)
  mb.Hreg(SHAFT_REG, (int16_t)(shaftAngle * 100)); // Shaft Angle (scaled by 100)
  mb.Hreg(FINAL_FUSION_REG, (int16_t)(fusedAngle * 100)); // Fused output (scaled by 100)
  
  // Always keep the last valid response time in register
  mb.Hreg(RESPONSE_TIME_REG, (uint16_t)lastValidResponseTime);

  // ---------- Serial Monitor Display ----------
  Serial.print("MPU: ");
  Serial.print(mpuRoll, 1);
  Serial.print("° | Shaft: ");
  Serial.print(shaftAngle, 1);
  Serial.print("° | Fused: ");
  Serial.print(fusedAngle, 2);
  Serial.print("° | State: ");
  
  switch (currentState) {
    case IDLE:
      Serial.print("IDLE");
      break;
    case POTENTIAL_MOVEMENT:
      Serial.print("POTENTIAL (max dev:");
      Serial.print(maxMovementDeviation, 1);
      Serial.print("°,stable:");
      Serial.print(stableCount);
      Serial.print("/");
      Serial.print(STABLE_COUNT_REQUIRED);
      Serial.print(")");
      break;
    case TRACKING_RESPONSE:
      Serial.print("TRACKING (max dev:");
      Serial.print(maxMovementDeviation, 1);
      Serial.print("°,stable:");
      Serial.print(stableCount);
      Serial.print("/");
      Serial.print(STABLE_COUNT_REQUIRED);
      Serial.print(")");
      break;
  }
  
  Serial.print(" | RT: ");
  if (angleChanged) {
    Serial.print(lastValidResponseTime);
    Serial.println("ms (NEW)");
    angleChanged = false; // Reset the flag after printing
  } else {
    Serial.print(lastValidResponseTime);
    Serial.println("ms (LAST)");
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

  delay(50); // Consistent with reference code timing
}