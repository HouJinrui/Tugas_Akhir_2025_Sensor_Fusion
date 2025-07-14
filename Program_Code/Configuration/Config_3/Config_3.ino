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
const int MF_REG = 0;           // Madgwick filter output
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

// Revised weights - AS5600 has more influence (30% vs previous 8%)
const float ALPHA = 0.30;       // Weight for Madgwick (70%)
const float BETA = 0.70;        // Weight for Shaft (30%)
const float ANGLE_THRESHOLD = 1.0; // Batas perubahan signifikan
const float GEAR_RATIO = 19.0;  // Gearbox ratio 1:19
const float MOVEMENT_THRESHOLD = 15.0; // Minimum movement to trigger timing
const float STABILITY_THRESHOLD = 0.1; // Stability detection threshold
const int STABLE_COUNT_REQUIRED = 10;  // Consecutive stable readings

// New variables for enhanced response time calculation
enum MovementState { IDLE, MOVING, STABILIZING };
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
  
  prevFusedAngle = 0.0;
  resetRequested = false;
  angleChanged = false;
  currentState = IDLE;
  lastValidResponseTime = 0.0;
  maxMovementDeviation = 0.0;
  stableCount = 0;
  Serial.println("Sistem dikalibrasi ulang - sudut direset ke 0");
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
  mb.addHreg(MF_REG);
  mb.addHreg(SHAFT_REG);
  mb.addHreg(FINAL_FUSION_REG);
  mb.addHreg(RESPONSE_TIME_REG);

  // Reset sudut awal
  resetAngles();

  Serial.println("Sistem siap (Config 3: Arch1 + Proven Methods)");
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

  // ---------- SENSOR FUSION (with new weights) ----------
  float fusedAngle = (ALPHA * mpuRoll) + (BETA * shaftAngle);
  fusedAngle = normalizeAngle(fusedAngle);

  // ---------- Enhanced Response Time Calculation ----------
  float angleChange = abs(fusedAngle - prevFusedAngle);
  if (angleChange > 180) angleChange = 360 - angleChange;  // Handle wrap-around
  
  switch (currentState) {
    case IDLE:
      if (angleChange >= ANGLE_THRESHOLD) {
        // Movement detected
        currentState = MOVING;
        movementStartTime = currentTime;
        movementStartAngle = fusedAngle;
        maxMovementDeviation = 0.0;
      }
      break;
      
    case MOVING:
      {
        // Track maximum deviation from start position
        float currentDeviation = abs(fusedAngle - movementStartAngle);
        if (currentDeviation > maxMovementDeviation) {
          maxMovementDeviation = currentDeviation;
        }
        
        // Only start timing if we've passed the movement threshold
        if (maxMovementDeviation >= MOVEMENT_THRESHOLD) {
          // Check if movement has stabilized
          if (angleChange < STABILITY_THRESHOLD) {
            stableCount++;
            
            // Check if we have enough stable readings
            if (stableCount >= STABLE_COUNT_REQUIRED) {
              responseTime = currentTime - movementStartTime;
              lastValidResponseTime = responseTime;
              angleChanged = true;
              currentState = IDLE;
              stableCount = 0;
            }
          } else {
            // Reset stability counter if movement detected
            stableCount = 0;
          }
        }
      }
      break;
  }

  prevFusedAngle = fusedAngle;

  // ---------- Simpan ke Modbus ----------
  mb.Hreg(MF_REG, (int16_t)(mpuRoll * 100));      // MPU Roll
  mb.Hreg(SHAFT_REG, (int16_t)(shaftAngle * 100)); // Shaft Angle
  mb.Hreg(FINAL_FUSION_REG, (int16_t)(fusedAngle * 100)); // Fused output
  
  // Always keep the last valid response time in register
  mb.Hreg(RESPONSE_TIME_REG, (uint16_t)lastValidResponseTime);

  // ---------- Serial Monitor ----------
  Serial.print("MPU Roll: ");
  Serial.print(mpuRoll, 2);
  Serial.print("°\tShaft: ");
  Serial.print(shaftAngle, 2);
  Serial.print("°\tFused: ");
  Serial.print(fusedAngle, 2);
  Serial.print("°\tResp: ");
  
  if (angleChanged) {
    Serial.print(lastValidResponseTime);
    Serial.println("ms");
    angleChanged = false;
  } else {
    Serial.print(lastValidResponseTime);
    Serial.println("ms (Last)");
  }

  // ---------- Handle Modbus ----------
  mb.task();

  // ---------- Check for reset command ----------
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      resetRequested = true;
    }
  }

  delay(10);
}