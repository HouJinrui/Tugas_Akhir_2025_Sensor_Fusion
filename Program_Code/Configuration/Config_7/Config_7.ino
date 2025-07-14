#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h> // Include MadgwickAHRS library
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
const int MPU_REG = 0;           // MPU fusion output (now Madgwick output)
const int SHAFT_REG = 1;         // Shaft angle
const int FINAL_FUSION_REG = 2;  // Final fusion value
const int RESPONSE_TIME_REG = 3; // Response time (ms)

// ------------------ OBJEK SENSOR ------------------------
MPU6050 mpu;
Madgwick filter; // Madgwick filter object
ModbusIP mb;

// ------------------ VARIABEL GLOBAL ---------------------
int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ; // Need Gyro data for Madgwick

float prevAngleDegAS5600 = 0.0;
float motorAngleTotal = 0.0;     // total rotasi motor (derajat)
float prevFusedAngle = 0.0;      // Nilai fusion sebelumnya
float responseTime = 0;          // Waktu respon

bool resetRequested = true;
bool angleChanged = false; // Flag to indicate if response time was just updated
float madgwickOffset = 0.0; // Offset for Madgwick output
float shaftOffset = 0.0;    // Offset for AS5600

// Weights for final fusion (Madgwick output vs AS5600 shaft angle)
// AS5600 (BETA) has more influence as requested
const float ALPHA = 0.30;       // Weight for Madgwick output (30%)
const float BETA = 0.70;        // Weight for Shaft (70%)
const float ANGLE_THRESHOLD = 1.0; // Batas perubahan signifikan for general movement detection
const float GEAR_RATIO = 19.0;  // Gearbox ratio 1:19
const float MOVEMENT_THRESHOLD = 15.0; // Minimum absolute movement (degrees) from start to trigger timing
const float STABILITY_THRESHOLD = 0.1; // Stability detection threshold (degrees)
const int STABLE_COUNT_REQUIRED = 10;  // Consecutive stable readings

// New variables for enhanced response time calculation
enum MovementState { IDLE, MOVING, STABILIZING };
MovementState currentState = IDLE;
unsigned long movementStartTime = 0;
float movementStartAngle = 0.0; // The angle at the start of a new movement cycle
float lastValidResponseTime = 0.0;
float maxMovementDeviation = 0.0; // Maximum absolute deviation from movementStartAngle
int stableCount = 0;
unsigned long lastLoopTime = 0; // To calculate delta time for Madgwick filter

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
  // Reset Madgwick filter
  filter.begin(100); // Initialize Madgwick filter with expected sample rate (e.g., 100 Hz based on 10ms delay)

  // Calibrate MPU6050 initial orientation for Madgwick offset
  mpu.getRotation(&GyroX, &GyroY, &GyroZ);
  mpu.getAcceleration(&AccX, &AccY, &AccZ);
  // Perform a few updates to let Madgwick converge
  for (int i = 0; i < 100; ++i) {
    mpu.getAcceleration(&AccX, &AccY, &AccZ);
    mpu.getRotation(&GyroX, &GyroY, &GyroZ);
    // Convert to m/s^2 and rad/s for Madgwick filter
    filter.updateIMU(
      (float)GyroX / 131.0, (float)GyroY / 131.0, (float)GyroZ / 131.0, // Gyro (rad/s)
      (float)AccX / 16384.0, (float)AccY / 16384.0, (float)AccZ / 16384.0 // Accel (g's, will be converted to m/s^2 by filter)
    );
    delay(10); // Match loop delay
  }
  madgwickOffset = filter.getRoll(); // Get current roll after convergence as offset

  // Reset sudut shaft (AS5600)
  uint16_t rawAngleInit = readAS5600RawAngle();
  if (rawAngleInit != 0xFFFF) {
    prevAngleDegAS5600 = (rawAngleInit * 360.0) / 4096.0;
    motorAngleTotal = 0.0;
    shaftOffset = 0.0; // Reset shaft offset
  }

  prevFusedAngle = 0.0;
  resetRequested = false;
  angleChanged = false;
  currentState = IDLE;
  // Do NOT reset lastValidResponseTime here as per requirement (still at the last value)
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
  mb.addHreg(MPU_REG);
  mb.addHreg(SHAFT_REG);
  mb.addHreg(FINAL_FUSION_REG);
  mb.addHreg(RESPONSE_TIME_REG);

  // Reset sudut awal
  resetAngles();

  lastLoopTime = millis(); // Initialize lastLoopTime
  Serial.println("Sistem siap (Config 7: Arch 2 - 3-Way Independent with Madgwick Filter)");
}

// ------------------ LOOP --------------------------------
void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastLoopTime) / 1000.0f; // Calculate delta time in seconds
  lastLoopTime = currentTime;

  if (resetRequested) {
    resetAngles();
  }

  // ---------- MPU6050: Data acquisition for Madgwick ----------
  mpu.getAcceleration(&AccX, &AccY, &AccZ);
  mpu.getRotation(&GyroX, &GyroY, &GyroZ);

  // Update Madgwick filter
  // Gyro values should be in rad/s, Accel in g's
  filter.updateIMU(
    (float)GyroX / 131.0, // MPU6050 Gyro datasheet sensitivity: 131 LSB/deg/s for +/-250 deg/s range
    (float)GyroY / 131.0,
    (float)GyroZ / 131.0,
    (float)AccX / 16384.0, // MPU6050 Accel datasheet sensitivity: 16384 LSB/g for +/-2g range
    (float)AccY / 16384.0,
    (float)AccZ / 16384.0
  );

  // Get Madgwick's estimated Roll angle in degrees
  float madgwickRoll = filter.getRoll(); // This is in degrees
  madgwickRoll = normalizeAngle(madgwickRoll - madgwickOffset); // Apply offset

  // ---------- AS5600: Sudut Shaft ----------
  float shaftAngle = 0.0;
  uint16_t rawAngle = readAS5600RawAngle();

  if (rawAngle != 0xFFFF) {
    float angleDegAS5600 = (rawAngle * 360.0) / 4096.0;
    float deltaAngle = angleDegAS5600 - prevAngleDegAS5600;

    // Koreksi wrap-around 360°
    if (deltaAngle > 180) deltaAngle -= 360;
    if (deltaAngle < -180) deltaAngle += 360;

    // Balik arah agar sesuai MPU6050 (CCW = + for roll)
    deltaAngle = -deltaAngle;

    motorAngleTotal += deltaAngle;
    prevAngleDegAS5600 = angleDegAS5600;

    shaftAngle = normalizeAngle((motorAngleTotal / GEAR_RATIO) - shaftOffset);
  }

  // ---------- FINAL SENSOR FUSION (Madgwick output + AS5600 shaft angle) ----------
  // Config 7: "3-Way Independent" means Madgwick (from Accel+Gyro) and AS5600 are
  // processed, and then their outputs are fused. AS5600 has higher weight.
  float fusedAngle = (ALPHA * madgwickRoll) + (BETA * shaftAngle);
  fusedAngle = normalizeAngle(fusedAngle);

  // ---------- Enhanced Response Time Calculation ----------
  // Calculate absolute change in angle from the previous fused angle
  float angleChange = abs(fusedAngle - prevFusedAngle);
  if (angleChange > 180) angleChange = 360 - angleChange; // Handle wrap-around for change calculation

  switch (currentState) {
    case IDLE:
      // Check for initial movement
      if (angleChange >= ANGLE_THRESHOLD) {
        // Movement detected, transition to MOVING state
        currentState = MOVING;
        movementStartTime = currentTime;
        movementStartAngle = fusedAngle; // Capture starting angle of this movement phase
        maxMovementDeviation = 0.0;     // Reset max deviation for new movement
        stableCount = 0;                // Reset stability counter
      }
      break;

    case MOVING:
      {
        // Track maximum deviation from the movement's start angle
        float currentDeviationFromStart = abs(fusedAngle - movementStartAngle);
        if (currentDeviationFromStart > 180) currentDeviationFromStart = 360 - currentDeviationFromStart;

        if (currentDeviationFromStart > maxMovementDeviation) {
          maxMovementDeviation = currentDeviationFromStart;
        }

        // Only start timing if we've passed the initial MOVEMENT_THRESHOLD (e.g., 15 degrees)
        if (maxMovementDeviation >= MOVEMENT_THRESHOLD) {
          // Check for stability after movement has passed the threshold
          if (angleChange < STABILITY_THRESHOLD) {
            stableCount++; // Increment stable counter
            // If stable for required number of readings, calculate and store response time
            if (stableCount >= STABLE_COUNT_REQUIRED) {
              responseTime = currentTime - movementStartTime;
              lastValidResponseTime = responseTime; // Store the calculated time
              angleChanged = true;                  // Flag for serial output
              currentState = IDLE;                  // Transition back to IDLE
              stableCount = 0;                      // Reset for next movement
            }
          } else {
            stableCount = 0; // Reset stability counter if significant movement occurs
          }
        }
      }
      break;
  }

  prevFusedAngle = fusedAngle; // Update for next iteration's angleChange calculation

  // ---------- Simpan ke Modbus ----------
  mb.Hreg(MPU_REG, (int16_t)(madgwickRoll * 100));     // Madgwick Roll output
  mb.Hreg(SHAFT_REG, (int16_t)(shaftAngle * 100));    // Shaft Angle
  mb.Hreg(FINAL_FUSION_REG, (int16_t)(fusedAngle * 100)); // Fused output
  // Always keep the last valid response time in register as per requirement
  mb.Hreg(RESPONSE_TIME_REG, (uint16_t)lastValidResponseTime);

  // ---------- Serial Monitor ----------
  Serial.print("Madgwick Roll: ");
  Serial.print(madgwickRoll, 2);
  Serial.print("°\tShaft: ");
  Serial.print(shaftAngle, 2);
  Serial.print("°\tFused: ");
  Serial.print(fusedAngle, 2);
  Serial.print("°\tState: ");
  Serial.print(currentState == IDLE ? "IDLE" : (currentState == MOVING ? "MOVING" : "STABILIZING"));
  Serial.print("\tMaxDev: ");
  Serial.print(maxMovementDeviation, 2);
  Serial.print("°\tResp: ");

  if (angleChanged) {
    Serial.print(lastValidResponseTime);
    Serial.println("ms (New)");
    angleChanged = false; // Reset flag after printing
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

  delay(10); // Maintain roughly 100Hz update rate
}