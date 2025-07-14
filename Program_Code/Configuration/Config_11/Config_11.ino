#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h> // You will need to install this library
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

// ------------------ REGISTER INDEX (UPDATED FOR CONFIG 11) ----------------------
const int MPU_FUSION_REG = 0;      // MPU fusion output (Madgwick)
const int SHAFT_REG = 1;         // Shaft angle
const int FINAL_FUSION_REG = 2;  // Final hierarchical fusion value
const int RESPONSE_TIME_REG = 3; // Response time (ms)

// ------------------ OBJEK SENSOR ------------------------
MPU6050 mpu;
Madgwick filter; // Madgwick filter object
ModbusIP mb;

// ------------------ VARIABEL GLOBAL ---------------------
int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ; // Added for Madgwick filter

float prevAngleDegAS5600 = 0.0;
float motorAngleTotal = 0.0;     // total rotasi motor (derajat)
float prevFinalFusedAngle = 0.0; // Nilai final fusion sebelumnya
float responseTime = 0;          // Waktu respon

bool resetRequested = true;
bool angleChangedForSerial = false; // Flag for serial printing response time
float rollOffsetMPU = 0.0; // Offset for MPU's initial orientation
float shaftOffset = 0.0;

// Weights for the FINAL fusion (AS5600 more influenced)
const float WEIGHT_MADGWICK = 0.30; // Weight for Madgwick output
const float WEIGHT_AS5600 = 0.70;   // Weight for AS5600 (higher influence)

const float GEAR_RATIO = 19.0;      // Gearbox ratio 1:19
const float ANGLE_CHANGE_THRESHOLD_FOR_MOVEMENT_START = 1.0; // Minimum 1 degree change to start considering movement
const float MOVEMENT_THRESHOLD_FOR_TIMING = 15.0; // Minimum total movement (degrees) to trigger timing
const float STABILITY_THRESHOLD = 0.1;           // Stability detection threshold (degrees)
const int STABLE_COUNT_REQUIRED = 10;            // Consecutive stable readings required

// New variables for enhanced response time calculation
enum MovementState { IDLE, PRE_MOVEMENT_DETECTED, MOVING, STABILIZING };
MovementState currentState = IDLE;
unsigned long movementStartTime = 0;
float initialFusionAngleAtMovementStart = 0.0;
float maxMovementDeviation = 0.0;
int stableCount = 0;
float lastValidResponseTime = 0.0;
bool isTiming = false; // Flag to indicate if response time is actively being calculated

// ------------------ FUNGSI BACA AS5600 ------------------
uint16_t readAS5600RawAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(RAW_ANGLE_HIGH);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() < 2) return 0xFFFF; // Return error value

    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    return ((high << 8) | low) & 0x0FFF; // 12-bit (0–4095)
}

// ------------------ FUNGSI RESET SUDUT ------------------
void resetAngles() {
    // Reset Madgwick filter
    filter.begin(50); // Set update rate (Hz) for Madgwick. MPU6050 sample rate should match or be higher.
                      // Delay in loop is 10ms, so 100 Hz. Adjust if necessary.
    
    // Reset sudut shaft (AS5600)
    uint16_t rawAngleInit = readAS5600RawAngle();
    if (rawAngleInit != 0xFFFF) {
        prevAngleDegAS5600 = (rawAngleInit * 360.0) / 4096.0;
        motorAngleTotal = 0.0;
        shaftOffset = 0.0; // Assuming shaftOffset is set to zero initially
    }

    prevFinalFusedAngle = 0.0;
    resetRequested = false;
    angleChangedForSerial = false;
    currentState = IDLE;
    lastValidResponseTime = 0.0;
    maxMovementDeviation = 0.0;
    stableCount = 0;
    isTiming = false;
    Serial.println("System recalibrated - angles reset to 0");
}

// ------------------ FUNGSI NORMALISASI SUDUT --------------------
float normalizeAngle(float angle) {
    // Normalize to range -180 to +180 degrees
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// ------------------ SETUP -------------------------------
void setup() {
    Serial.begin(9600);
    Wire.begin(); // I2C

    // Inisialisasi MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 not connected!");
        while (1);
    }
    Serial.println("MPU6050 connected!");

    // Konfigurasi Modbus
    uint8_t ip[] = IP_ADDR;
    uint8_t mac[] = MAC_ADDR;
    mb.config(mac, ip);
    mb.addHreg(MPU_FUSION_REG);
    mb.addHreg(SHAFT_REG);
    mb.addHreg(FINAL_FUSION_REG);
    mb.addHreg(RESPONSE_TIME_REG);

    // Reset initial angles and Madgwick filter
    resetAngles();

    Serial.println("System ready (Config 11: Arch3 + Madgwick + Hierarchical)");
}

// ------------------ LOOP --------------------------------
void loop() {
    unsigned long currentTime = millis();

    if (resetRequested) {
        resetAngles();
    }

    // ---------- MPU6050: Read Raw Data for Madgwick ----------
    mpu.getAcceleration(&AccX, &AccY, &AccZ);
    mpu.getRotation(&GyroX, &GyroY, &GyroZ);

    // Convert raw values to g and deg/s for Madgwick
    float ax = AccX / 16384.0; // Assuming MPU6050 default range (+/- 2g)
    float ay = AccY / 16384.0;
    float az = AccZ / 16384.0;

    float gx = GyroX / 131.0; // Assuming MPU6050 default range (+/- 250 deg/s)
    float gy = GyroY / 131.0;
    float gz = GyroZ / 131.0;

    // Update Madgwick filter
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // Get roll angle from Madgwick filter (first stage fusion)
    float madgwickRoll = normalizeAngle(filter.getRoll());
    
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

        // Apply gear ratio and normalize. We don't use shaftOffset here if we want absolute shaft value
        shaftAngle = normalizeAngle(motorAngleTotal / GEAR_RATIO);
    }

    // ---------- FINAL HIERARCHICAL SENSOR FUSION ----------
    // Combine Madgwick Roll and AS5600 Shaft Angle with specified weights
    float finalFusedAngle = (WEIGHT_MADGWICK * madgwickRoll) + (WEIGHT_AS5600 * shaftAngle);
    finalFusedAngle = normalizeAngle(finalFusedAngle);

    // ---------- Enhanced Response Time Calculation ----------
    float currentAngleChangeFromPrev = abs(finalFusedAngle - prevFinalFusedAngle);
    if (currentAngleChangeFromPrev > 180) currentAngleChangeFromPrev = 360 - currentAngleChangeFromPrev; // Handle wrap-around

    float currentDeviationFromStart = abs(finalFusedAngle - initialFusionAngleAtMovementStart);
    if (currentDeviationFromStart > 180) currentDeviationFromStart = 360 - currentDeviationFromStart; // Handle wrap-around

    switch (currentState) {
        case IDLE:
            if (currentAngleChangeFromPrev >= ANGLE_CHANGE_THRESHOLD_FOR_MOVEMENT_START) {
                // Initial movement detected, but not necessarily enough to start timing yet
                currentState = PRE_MOVEMENT_DETECTED;
                movementStartTime = currentTime;
                initialFusionAngleAtMovementStart = finalFusedAngle;
                maxMovementDeviation = 0.0; // Reset max deviation
                isTiming = false; // Not actively timing yet
            }
            break;

        case PRE_MOVEMENT_DETECTED:
            if (currentDeviationFromStart >= MOVEMENT_THRESHOLD_FOR_TIMING) {
                // Movement threshold met, start active timing
                currentState = MOVING;
                isTiming = true;
                // movementStartTime is already set from PRE_MOVEMENT_DETECTED
            } else if (currentAngleChangeFromPrev < ANGLE_CHANGE_THRESHOLD_FOR_MOVEMENT_START) {
                // Movement stopped before reaching threshold, go back to IDLE
                currentState = IDLE;
                isTiming = false;
            }
            // Update max movement deviation if still in PRE_MOVEMENT_DETECTED
            if (currentDeviationFromStart > maxMovementDeviation) {
                maxMovementDeviation = currentDeviationFromStart;
            }
            break;

        case MOVING:
            // Update max movement deviation during active movement
            if (currentDeviationFromStart > maxMovementDeviation) {
                maxMovementDeviation = currentDeviationFromStart;
            }

            // Check for stabilization
            if (currentAngleChangeFromPrev < STABILITY_THRESHOLD) {
                stableCount++;
                if (stableCount >= STABLE_COUNT_REQUIRED) {
                    // System stabilized after sufficient movement
                    responseTime = currentTime - movementStartTime;
                    lastValidResponseTime = responseTime;
                    angleChangedForSerial = true; // Indicate new response time for serial print
                    currentState = IDLE;
                    stableCount = 0;
                    isTiming = false;
                }
            } else {
                // Reset stability counter if movement detected again
                stableCount = 0;
            }
            break;
    }

    prevFinalFusedAngle = finalFusedAngle;

    // ---------- Simpan ke Modbus ----------
    mb.Hreg(MPU_FUSION_REG, (int16_t)(madgwickRoll * 100));     // Madgwick output
    mb.Hreg(SHAFT_REG, (int16_t)(shaftAngle * 100));          // Shaft Angle
    mb.Hreg(FINAL_FUSION_REG, (int16_t)(finalFusedAngle * 100)); // Final hierarchical fused output

    // Always keep the last valid response time in register
    mb.Hreg(RESPONSE_TIME_REG, (uint16_t)lastValidResponseTime);

    // ---------- Serial Monitor ----------
    Serial.print("Madgwick Roll: ");
    Serial.print(madgwickRoll, 2);
    Serial.print("°\tShaft: ");
    Serial.print(shaftAngle, 2);
    Serial.print("°\tFinal Fused: ");
    Serial.print(finalFusedAngle, 2);
    Serial.print("°\tResp: ");

    if (angleChangedForSerial) {
        Serial.print(lastValidResponseTime);
        Serial.println("ms (NEW)");
        angleChangedForSerial = false;
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

    delay(10); // Adjust delay based on desired Madgwick update rate
}