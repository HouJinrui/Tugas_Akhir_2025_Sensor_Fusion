#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <EtherCard.h>
#include <Modbus.h>
#include <ModbusIP_ENC28J60.h>

// ------------------ AS5600 CONFIGURATION ------------------
#define AS5600_ADDR 0x36
#define RAW_ANGLE_HIGH 0x0C
#define RAW_ANGLE_LOW  0x0D

// ------------------ MODBUS TCP CONFIGURATION -------------------
#define IP_ADDR {192, 168, 2, 103}
#define MAC_ADDR {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}

// ------------------ MODBUS REGISTER INDEXES ----------------------
// These constants define the Modbus Holding Register addresses for different sensor outputs and calculated values.
const int MPU = 0;              // MPU fusion output (Roll angle from MPU6050)
const int SHAFT_REG = 1;        // Shaft angle (from AS5600 encoder)
const int FINAL_FUSION_REG = 2; // Final fused angle (weighted average of MPU and Shaft)
const int RESPONSE_TIME_REG = 3; // Calculated response time in milliseconds

// ------------------ SENSOR OBJECTS ------------------------
MPU6050 mpu; // MPU6050 sensor object
ModbusIP mb; // ModbusIP object for TCP communication

// ------------------ GLOBAL VARIABLES ---------------------
int16_t AccX, AccY, AccZ; // Raw accelerometer readings from MPU6050
float AngleRoll;          // Unused variable, can be removed

float prevAngleDegAS5600 = 0.0; // Stores the previous raw angle from AS5600 for delta calculation
float motorAngleTotal = 0.0;    // Accumulates total rotation of the motor shaft (in degrees)
float prevFusedAngle = 0.0;     // Stores the previous final fused angle for change detection
float responseTime = 0;         // Current calculated response time (ms)

bool resetRequested = true;     // Flag to trigger a system reset/calibration
bool angleChanged = false;      // Flag to indicate if a new response time has been calculated
float rollOffset = 0.0;         // Offset for MPU6050 roll angle calibration
float shaftOffset = 0.0;        // Offset for AS5600 shaft angle calibration

// Weighted average configuration - AS5600 has more influence (60%)
// These weights determine the contribution of each sensor to the final fused angle.
const float MPU_WEIGHT = 0.40;   // Weight for MPU (40%)
const float SHAFT_WEIGHT = 0.60; // Weight for Shaft (60%) - AS5600 is weighted more heavily as requested

const float ANGLE_THRESHOLD = 1.0; // Minimum angle change (in degrees) to start tracking movement.
                                   // If the angle changes by 1 degree or more, the system considers it a movement start.
const float GEAR_RATIO = 19.0;   // Gearbox ratio (1:19) for AS5600 angle conversion
const float MOVEMENT_THRESHOLD = 15.0; // Minimum total movement (in degrees) required for a response time to be reported.
                                       // If the total movement from the start of the current movement phase is less than 15 degrees,
                                       // the calculated response time will not be sent to Modbus.
const float STABILITY_THRESHOLD = 0.5; // Threshold (in degrees) for detecting stability.
                                       // If the angle change between consecutive readings is less than this, it contributes to stability count.
const int STABLE_COUNT_REQUIRED = 6;   // Number of consecutive stable readings required to declare the system "stable"
                                       // and stop the response time calculation. This helps to filter out minor oscillations.

// Enhanced response time calculation variables
enum MovementState { IDLE, MOVING }; // Defines the two states for movement tracking
MovementState currentState = IDLE;   // Current state of the movement tracker
unsigned long movementStartTime = 0; // Stores the millis() timestamp when movement was first detected
float movementStartAngle = 0.0;      // Stores the fused angle at the start of movement
float lastValidResponseTime = 0.0;   // Stores the last successfully calculated response time (persists in Modbus register)
float maxMovementDeviation = 0.0;    // Tracks the maximum angular deviation from the movement start angle
int stableCount = 0;                 // Counter for consecutive stable readings

// ------------------ AS5600 READ FUNCTION ------------------
// Reads the 12-bit raw angle from the AS5600 magnetic encoder.
uint16_t readAS5600RawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_HIGH); // Request high byte of raw angle
  Wire.endTransmission(false); // Do not release the bus
  Wire.requestFrom(AS5600_ADDR, 2); // Request 2 bytes (high and low)

  if (Wire.available() < 2) return 0xFFFF; // Return error value if not enough bytes are available

  uint8_t high = Wire.read(); // Read high byte
  uint8_t low = Wire.read();  // Read low byte
  return ((high << 8) | low) & 0x0FFF; // Combine bytes and mask to 12-bit (0-4095)
}

// ------------------ ANGLE RESET FUNCTION ------------------
// Resets the sensor offsets and all movement tracking variables, effectively calibrating the system to 0 degrees.
void resetAngles() {
  // Reset MPU6050 angle offset
  mpu.getAcceleration(&AccX, &AccY, &AccZ);
  float fAccX = (float)AccX;
  float fAccY = (float)AccY;
  float fAccZ = (float)AccZ;
  // Calculate initial roll angle from accelerometer data and set it as the offset
  rollOffset = atan2(fAccY, sqrt(fAccX * fAccX + fAccZ * fAccZ)) * (180.0 / PI);

  // Reset shaft angle (AS5600) offset
  uint16_t rawAngleInit = readAS5600RawAngle();
  if (rawAngleInit != 0xFFFF) {
    prevAngleDegAS5600 = (rawAngleInit * 360.0) / 4096.0; // Convert raw 12-bit value to degrees
    motorAngleTotal = 0.0; // Reset total motor rotation
    shaftOffset = 0.0;     // Reset shaft offset (not used in current calculation, but good for consistency)
  }

  // Reset all movement tracking variables to their initial state
  prevFusedAngle = 0.0;
  resetRequested = false;
  angleChanged = false;
  currentState = IDLE;
  lastValidResponseTime = 0.0; // Clear the last reported response time
  maxMovementDeviation = 0.0;
  stableCount = 0;

  Serial.println("=== SYSTEM RESET - ANGLES CALIBRATED TO 0 ===");
}

// ------------------ ANGLE NORMALIZATION FUNCTION --------------------
// Normalizes an angle to the range of -180 to +180 degrees.
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// ------------------ SETUP FUNCTION -------------------------------
// Initializes serial communication, I2C, MPU6050, and Modbus TCP.
void setup() {
  Serial.begin(9600); // Start serial communication for debugging
  Wire.begin();       // Initialize I2C bus for AS5600 and MPU6050

  // Initialize MPU6050 sensor
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected!");
    while (1); // Halt if MPU6050 is not found
  }
  Serial.println("MPU6050 connected!");

  // Configure Modbus TCP
  uint8_t ip[] = IP_ADDR;   // Define IP address
  uint8_t mac[] = MAC_ADDR; // Define MAC address
  mb.config(mac, ip);       // Configure EtherCard and ModbusIP
  mb.addHreg(MPU);          // Add MPU register to Modbus
  mb.addHreg(SHAFT_REG);    // Add Shaft register to Modbus
  mb.addHreg(FINAL_FUSION_REG); // Add Final Fusion register to Modbus
  mb.addHreg(RESPONSE_TIME_REG); // Add Response Time register to Modbus

  // Perform initial angle reset/calibration
  resetAngles();

  // Print system readiness messages to serial monitor
  Serial.println("=== SYSTEM READY - RESPONSE TIME TRACKER ===");
  Serial.println("- Start counting when movement >= 1° detected.");
  Serial.println("- Report response time only if total movement >= 15°.");
  Serial.println("- Stop counting when system is stable.");
  Serial.println("- Send 'r' to reset.");
  Serial.println("==========================================");
}

// ------------------ MAIN LOOP FUNCTION --------------------------------
// Continuously reads sensor data, performs fusion, calculates response time, and updates Modbus registers.
void loop() {
  unsigned long currentTime = millis(); // Get current time in milliseconds

  // Check if a reset has been requested
  if (resetRequested) {
    resetAngles(); // Perform system reset
  }

  // ---------- MPU6050: Roll Angle Calculation ----------
  mpu.getAcceleration(&AccX, &AccY, &AccZ); // Get raw accelerometer data
  float fAccX = (float)AccX;
  float fAccY = (float)AccY;
  float fAccZ = (float)AccZ;

  // Calculate raw roll angle from accelerometer data
  float rawRoll = atan2(fAccY, sqrt(fAccX * fAccX + fAccZ * fAccZ)) * (180.0 / PI);
  // Apply offset and normalize the MPU roll angle
  float mpuRoll = normalizeAngle(rawRoll - rollOffset);

  // ---------- AS5600: Shaft Angle Calculation ----------
  float shaftAngle = 0.0;
  uint16_t rawAngle = readAS5600RawAngle(); // Read raw angle from AS5600

  if (rawAngle != 0xFFFF) { // Check for valid AS5600 reading
    float angleDegAS5600 = (rawAngle * 360.0) / 4096.0; // Convert raw 12-bit value to degrees
    float deltaAngle = angleDegAS5600 - prevAngleDegAS5600; // Calculate change in angle

    // Correct for 360° wrap-around (e.g., 350 -> 10 should be +20, not -340)
    if (deltaAngle > 180) deltaAngle -= 360;
    if (deltaAngle < -180) deltaAngle += 360;

    // Invert direction if needed to match MPU6050 (CCW = +)
    deltaAngle = -deltaAngle;

    motorAngleTotal += deltaAngle; // Accumulate total motor rotation
    prevAngleDegAS5600 = angleDegAS5600; // Update previous angle for next iteration

    // Calculate shaft angle, apply gearbox ratio, and normalize
    shaftAngle = normalizeAngle((motorAngleTotal / GEAR_RATIO) - shaftOffset);
  }

  // ---------- WEIGHTED AVERAGE FUSION (Config 12) ----------
  // Combine MPU roll and AS5600 shaft angle using predefined weights.
  float fusedAngle = (MPU_WEIGHT * mpuRoll) + (SHAFT_WEIGHT * shaftAngle);
  fusedAngle = normalizeAngle(fusedAngle); // Normalize the final fused angle

  // ---------- Enhanced Response Time Calculation ----------
  // Calculate the absolute change in the fused angle from the previous reading.
  float angleChange = abs(fusedAngle - prevFusedAngle);
  if (angleChange > 180) angleChange = 360 - angleChange; // Handle wrap-around for angle change

  // State machine for tracking movement and response time
  switch (currentState) {
    case IDLE:
      // If a significant angle change is detected (>= ANGLE_THRESHOLD), transition to MOVING state.
      if (angleChange >= ANGLE_THRESHOLD) {
        currentState = MOVING;           // Change state to MOVING
        movementStartTime = currentTime; // Record the start time of the movement
        movementStartAngle = fusedAngle; // Record the angle at the start of the movement
        maxMovementDeviation = 0.0;      // Reset max deviation for the new movement
        stableCount = 0;                 // Reset stability counter

        Serial.print(">>> MOVEMENT STARTED from: ");
        Serial.print(fusedAngle, 2);
        Serial.println("° - Timer started!");
      }
      break;

    case MOVING:
      {
        // Track the maximum deviation from the starting angle of the current movement.
        float currentDeviation = abs(fusedAngle - movementStartAngle);
        if (currentDeviation > 180) currentDeviation = 360 - currentDeviation; // Handle wrap-around for deviation
        if (currentDeviation > maxMovementDeviation) {
          maxMovementDeviation = currentDeviation; // Update max deviation if current is greater
        }

        // Only proceed with response time calculation if the total movement has exceeded the MOVEMENT_THRESHOLD.
        if (maxMovementDeviation >= MOVEMENT_THRESHOLD) {
          // Check if the system has stabilized (angle change is below STABILITY_THRESHOLD).
          if (angleChange < STABILITY_THRESHOLD) {
            stableCount++; // Increment stable count if stable
            // If enough consecutive stable readings are accumulated, declare movement complete.
            if (stableCount >= STABLE_COUNT_REQUIRED) {
              responseTime = currentTime - movementStartTime; // Calculate the total response time
              lastValidResponseTime = responseTime;           // Store as the last valid response time
              angleChanged = true;                            // Set flag to indicate new response time is ready

              // Log completion details to serial monitor
              Serial.println("==========================================");
              Serial.println(">>> SYSTEM STABLE - RESPONSE TIME COMPLETE! <<<");
              Serial.print("    Start angle: ");
              Serial.print(movementStartAngle, 2);
              Serial.println("°");
              Serial.print("    End angle: ");
              Serial.print(fusedAngle, 2);
              Serial.println("°");
              Serial.print("    Total movement: ");
              Serial.print(maxMovementDeviation, 1);
              Serial.println("°");
              Serial.print("    RESPONSE TIME: ");
              Serial.print(lastValidResponseTime);
              Serial.println(" ms");
              Serial.println("==========================================");

              currentState = IDLE;   // Reset to IDLE state, ready for next movement
              stableCount = 0;       // Reset stability counter
            }
          } else {
            stableCount = 0; // Reset stability counter if movement is still detected
          }
        } else {
          // If movement is still ongoing but hasn't reached the MOVEMENT_THRESHOLD yet,
          // check if it stops prematurely.
          if (angleChange < STABILITY_THRESHOLD) {
            stableCount++;
            if (stableCount >= STABLE_COUNT_REQUIRED) {
              // Movement stopped before reaching the 15-degree threshold.
              Serial.print(">>> Movement stopped before 15° (");
              Serial.print(maxMovementDeviation, 1);
              Serial.println("°) - Reset to IDLE");
              currentState = IDLE; // Reset to IDLE
              stableCount = 0;     // Reset stability counter
            }
          } else {
            stableCount = 0; // Reset stability counter if movement continues
          }
        }
      }
      break;
  }

  prevFusedAngle = fusedAngle; // Update previous fused angle for next loop iteration

  // ---------- Store to Modbus Registers ----------
  // Scale angle values by 100 to send them as integers via Modbus
  mb.Hreg(MPU, (int16_t)(mpuRoll * 100));         // MPU Roll
  mb.Hreg(SHAFT_REG, (int16_t)(shaftAngle * 100)); // Shaft Angle
  mb.Hreg(FINAL_FUSION_REG, (int16_t)(fusedAngle * 100)); // Fused output

  // Always keep the last valid response time in the register, as requested.
  mb.Hreg(RESPONSE_TIME_REG, (uint16_t)lastValidResponseTime);

  // ---------- Serial Monitor Display ----------
  Serial.print("MPU: ");
  Serial.print(mpuRoll, 1);
  Serial.print("° | Shaft: ");
  Serial.print(shaftAngle, 1);
  Serial.print("° | Fused: ");
  Serial.print(fusedAngle, 2);
  Serial.print("° | State: ");

  if (currentState == IDLE) {
    Serial.print("IDLE");
  } else {
    Serial.print("MOVING(");
    Serial.print(maxMovementDeviation, 1);
    Serial.print("°,stable:");
    Serial.print(stableCount);
    Serial.print("/");
    Serial.print(STABLE_COUNT_REQUIRED);
    Serial.print(")");
  }

  Serial.print(" | RT: ");
  if (angleChanged) {
    Serial.print(lastValidResponseTime);
    Serial.println("ms");
    angleChanged = false; // Reset flag after printing
  } else {
    Serial.print(lastValidResponseTime);
    Serial.println("ms (Last)"); // Indicate that this is the previously calculated time
  }

  // ---------- Handle Modbus Communication ----------
  mb.task(); // Process Modbus requests

  // ---------- Check for reset command from Serial Monitor ----------
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      resetRequested = true; // Set flag to trigger reset in next loop iteration
      Serial.println(">>> RESET REQUESTED <<<");
    }
  }

  delay(50); // Small delay for consistent timing and to avoid overwhelming the serial port
}
