#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <Modbus.h>
#include <ModbusIP_ENC28J60.h>

// ------------------ KONFIGURASI AS5600 ------------------
#define AS5600_ADDR 0x36
#define RAW_ANGLE_HIGH 0x0C
#define RAW_ANGLE_LOW 0x0D

// ------------------ MODBUS TCP CONFIG -------------------
#define IP_ADDR {192, 168, 2, 101}
#define MAC_ADDR {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}

// ------------------ REGISTER INDEX ----------------------
const int MPU = 0;           // MPU fusion output (Kalman Filtered)
const int SHAFT_REG = 1;         // Shaft angle
const int FINAL_FUSION_REG = 2; // Final fusion value
const int RESPONSE_TIME_REG = 3; // Response time (ms)

// ------------------ OBJEK SENSOR ------------------------
MPU6050 mpu;
ModbusIP mb;

// ------------------ VARIABEL GLOBAL ---------------------
int16_t AccX, AccY, AccZ, GyroX, GyroY, GyroZ;

float prevAngleDegAS5600 = 0.0;
float motorAngleTotal = 0.0;     // total rotasi motor (derajat)

bool resetRequested = true;
float rollOffset = 0.0;
float shaftOffset = 0.0;

// Hierarchical Fusion configuration
// In this architecture, the Kalman Filter output (from MPU) is fused with AS5600
// AS5600 is weighted more (75%)
const float FUSION_W_KF_OUTPUT = 0.25; // Weight for Kalman Filter output (25%)
const float FUSION_W_SHAFT = 0.75;    // Weight for Shaft (AS5600) (75%)
const float ANGLE_THRESHOLD = 0.1; // Batas perubahan kecil untuk mendeteksi konstanta
const float MIN_MOVEMENT_FOR_RESPONSE_TIME = 15.0; // Minimal pergerakan untuk response time (derajat)
const float STABILIZATION_ANGLE_TOLERANCE = 0.1; // Max angle variation for stabilization
const float STABILIZATION_TIME_REQUIRED = 2000; // 2-second stabilization period (ms)

const float GEAR_RATIO = 19.0;   // Gearbox ratio 1:19

// Gyro configuration
const float GYRO_SENSITIVITY = 131.0; // LSB/deg/s for 250dps range
float gyroOffsetX = 0.0;
unsigned long lastTime = 0;

// Response time calculation
enum MovementState { CONSTANT, MOVING, STABILIZING_RESPONSE };
MovementState currentResponseState = CONSTANT;
unsigned long movementStartTime = 0;
unsigned long stabilizationStartTime = 0;
float movementStartAngle = 0.0;
float lastValidResponseTime = 0.0;
bool movementThresholdReached = false;
float lastFinalFusionValues[10]; // Store last few values to check stability
int lastFinalFusionIndex = 0;
const int STABILITY_CHECK_COUNT = 10; // Number of values to check for stability

// Final fusion output
float finalFusion = 0.0;         // Hierarchical fusion output
float shaftAngle = 0.0;          // Global shaft angle
float kf_angle_output = 0.0;     // Kalman filter output from MPU

// ------------------ KALMAN FILTER VARIABLES -------------
float kf_angle = 0.0;            // Kalman filter angle
float kf_bias = 0.0;             // Gyro bias estimate
float P[2][2] = {{1000, 0}, {0, 1000}}; // Covariance matrix
const float Q_angle = 0.001;     // Process noise variance for angle
const float Q_bias = 0.003;      // Process noise variance for bias
const float R_measure = 0.03;    // Measurement noise variance

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
    shaftAngle = 0.0;
  }

  // Reset Kalman Filter
  kf_angle = 0.0;
  kf_bias = 0.0;
  P[0][0] = 1000;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 1000;

  // Reset fusion
  finalFusion = 0.0;
  kf_angle_output = 0.0;

  resetRequested = false;
  currentResponseState = CONSTANT;
  lastValidResponseTime = 0.0;
  movementThresholdReached = false;
  stabilizationStartTime = 0;
  for(int i=0; i<STABILITY_CHECK_COUNT; i++) lastFinalFusionValues[i] = 0.0;


  Serial.println("Sistem dikalibrasi ulang - sudut direset ke 0");
}

// ------------------ KALIBRASI GYRO ----------------------
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

// ------------------ FUNGSI NORMALISASI SUDUT --------------------
float normalizeAngle(float angle) {
  // Normalisasi ke rentang -180 hingga +180 derajat
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// ------------------ KALMAN FILTER UPDATE ------------------------
void updateKalmanFilter(float accAngle, float gyroRate, float dt) {
  // Prediction step
  float rate = gyroRate - kf_bias;
  kf_angle += dt * rate;

  // Update covariance matrix
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Calculate innovation and normalize
  float innovation = accAngle - kf_angle;
  innovation = normalizeAngle(innovation);

  // Calculate Kalman gain
  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Update state
  kf_angle += K[0] * innovation;
  kf_bias += K[1] * innovation;

  // Update covariance matrix
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
}

// Function to check for stability of the final fusion value
bool isStable(float currentAngle) {
  // Add the current angle to the circular buffer
  lastFinalFusionValues[lastFinalFusionIndex] = currentAngle;
  lastFinalFusionIndex = (lastFinalFusionIndex + 1) % STABILITY_CHECK_COUNT;

  // If we haven't filled the buffer yet, it's not stable
  if (lastFinalFusionIndex != (STABILITY_CHECK_COUNT - 1)) { // Check if we have enough data points
    return false;
  }

  // Check if all values in the buffer are within the tolerance
  float minVal = lastFinalFusionValues[0];
  float maxVal = lastFinalFusionValues[0];
  for (int i = 1; i < STABILITY_CHECK_COUNT; i++) {
    if (lastFinalFusionValues[i] < minVal) minVal = lastFinalFusionValues[i];
    if (lastFinalFusionValues[i] > maxVal) maxVal = lastFinalFusionValues[i];
  }
  return (maxVal - minVal) <= STABILIZATION_ANGLE_TOLERANCE;
}


// ------------------ SETUP -------------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();   // I2C

  // Inisialisasi MPU6050
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // Set range 250dps

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 tidak terhubung!");
    while (1);
  }
  Serial.println("MPU6050 terhubung!");

  // Kalibrasi gyro
  calibrateGyro();

  // Konfigurasi Modbus
  uint8_t ip[] = IP_ADDR;
  uint8_t mac[] = MAC_ADDR;
  mb.config(mac, ip);
  mb.addHreg(MPU);
  mb.addHreg(SHAFT_REG);
  mb.addHreg(FINAL_FUSION_REG);
  mb.addHreg(RESPONSE_TIME_REG);

  // Reset sudut awal
  lastTime = millis();
  resetAngles();

  Serial.println("Sistem siap (Config 10: Arch 3 + Kalman Filter)");
}

// ------------------ LOOP --------------------------------
void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // dalam detik
  lastTime = currentTime;

  if (resetRequested) {
    resetAngles();
  }

  // ---------- BACA DATA SENSOR ----------
  mpu.getMotion6(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);

  // ---------- KALMAN FILTER: MPU ROLL (Initial Sensor Fusion) ----------
  float fAccX = (float)AccX;
  float fAccY = (float)AccY;
  float fAccZ = (float)AccZ;
  float accRoll = atan2(fAccY, sqrt(fAccX * fAccX + fAccZ * fAccZ)) * (180.0 / PI);
  accRoll -= rollOffset;
  accRoll = normalizeAngle(accRoll);

  float gyroRate = (GyroX - gyroOffsetX) / GYRO_SENSITIVITY; // deg/s
  updateKalmanFilter(accRoll, gyroRate, dt);
  kf_angle_output = normalizeAngle(kf_angle); // Output of the first sensor fusion (Kalman Filter)

  // ---------- AS5600: SUDUT SHAFT ----------
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

  // ---------- FINAL SENSOR FUSION (Hierarchical Fusion) ----------
  // Combining Kalman Filter output and AS5600 shaft angle
  finalFusion = (FUSION_W_KF_OUTPUT * kf_angle_output) + (FUSION_W_SHAFT * shaftAngle);
  finalFusion = normalizeAngle(finalFusion);

  // ========== RESPONSE TIME CALCULATION ==========
  // Check if current finalFusion value indicates movement
  float currentAngleChange = abs(finalFusion - lastFinalFusionValues[(lastFinalFusionIndex - 1 + STABILITY_CHECK_COUNT) % STABILITY_CHECK_COUNT]);
  if (currentAngleChange > 180) currentAngleChange = 360 - currentAngleChange; // Handle wrap-around

  switch (currentResponseState) {
    case CONSTANT:
      // If movement is detected (more than ANGLE_THRESHOLD)
      if (currentAngleChange >= ANGLE_THRESHOLD) {
        movementStartTime = currentTime;
        movementStartAngle = finalFusion; // Capture the angle where movement started
        movementThresholdReached = false; // Reset movement threshold flag
        currentResponseState = MOVING;
      }
      break;

    case MOVING:
      // Check if the total movement from the start point exceeds MIN_MOVEMENT_FOR_RESPONSE_TIME
      float totalMovementSinceStart = abs(finalFusion - movementStartAngle);
      if (totalMovementSinceStart > 180) totalMovementSinceStart = 360 - totalMovementSinceStart; // Handle wrap-around

      if (totalMovementSinceStart >= MIN_MOVEMENT_FOR_RESPONSE_TIME) {
        movementThresholdReached = true;
      }

      // If the angle has become stable
      if (isStable(finalFusion)) {
        if (movementThresholdReached) {
          lastValidResponseTime = currentTime - movementStartTime; // Calculate response time
          currentResponseState = CONSTANT; // Reset state
        } else {
          // Movement stopped but didn't reach the threshold, so reset without recording time
          currentResponseState = CONSTANT;
        }
      } else {
        // Still moving, keep checking for stability
      }
      break;

    case STABILIZING_RESPONSE:
        // This state is not explicitly used with the new stability check based on a buffer
        // The stability check is integrated directly into the MOVING state.
        break;
  }

  // Store current finalFusion value for stability check in the next loop iteration
  isStable(finalFusion); // Call to update the internal buffer

  // ---------- SIMPAN KE MODBUS ----------
  mb.Hreg(MPU, (int16_t)(kf_angle_output * 100));     // Kalman filter output (first fusion)
  mb.Hreg(SHAFT_REG, (int16_t)(shaftAngle * 100));       // Shaft Angle
  mb.Hreg(FINAL_FUSION_REG, (int16_t)(finalFusion * 100)); // Final hierarchical fusion output
  mb.Hreg(RESPONSE_TIME_REG, (uint16_t)lastValidResponseTime); // Response time

  // ---------- SERIAL MONITOR ----------
  Serial.print("KF (MPU): ");
  Serial.print(kf_angle_output, 1);
  Serial.print("°\tAS5600: ");
  Serial.print(shaftAngle, 1);
  Serial.print("°\tFinal Fusion: ");
  Serial.print(finalFusion, 1);
  Serial.print("°\tResp: ");
  Serial.print(lastValidResponseTime);
  Serial.print("ms\tState: ");

  switch(currentResponseState) {
    case CONSTANT: Serial.print("CONSTANT"); break;
    case MOVING: Serial.print("MOVING"); break;
    case STABILIZING_RESPONSE: Serial.print("STABILIZING_RESPONSE (NOT USED)"); break; // This case is theoretically not reached now
  }
  if (movementThresholdReached) Serial.print(" (THRESHOLD MET)");
  Serial.println();

  // ---------- HANDLE MODBUS ----------
  mb.task();

  // ---------- CHECK RESET COMMAND ----------
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      resetRequested = true;
    }
  }

  delay(10);
}