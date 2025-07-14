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
#define IP_ADDR {192, 168, 2, 101}
#define MAC_ADDR {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}

// ------------------ MODBUS REGISTER INDEXES (AS SPECIFIED) ----------------------
const int MPU = 0;              // MPU fusion output (e.g., roll angle from MPU's internal processing)
const int SHAFT_REG = 1;        // Shaft angle from AS5600
const int FINAL_FUSION_REG = 2; // Final hierarchical fusion value
const int RESPONSE_TIME_REG = 3; // Calculated response time (ms)

// ------------------ SENSOR OBJECTS ------------------------
MPU6050 mpu; // MPU6050 sensor object
ModbusIP mb; // Modbus TCP object

// ------------------ GLOBAL VARIABLES ---------------------
int16_t AccX, AccY, AccZ; // Raw accelerometer data
float AngleRoll; // Variable to hold MPU roll angle (though 'mpuRoll' is used for the filtered value)

float prevAngleDegAS5600 = 0.0; // Stores the previous raw AS5600 angle for delta calculation
float motorAngleTotal = 0.0;    // Accumulates total motor rotation in degrees
float prevFusedAngle = 0.0;     // Stores the previous final fused angle for movement detection
// float responseTime = 0;         // This variable was removed as it's no longer needed.

bool resetRequested = true; // Flag to trigger a system reset/recalibration
// bool angleChanged = false; // This flag is redundant with 'responseTimeCalculated' for logging purposes
float rollOffset = 0.0; // Offset for MPU roll angle to calibrate to 0
float shaftOffset = 0.0; // Offset for AS5600 shaft angle to calibrate to 0

// ------------------ FUSION & MOVEMENT THRESHOLDS ----------------------
// Weighted average configuration for Hierarchical Fusion (Config 9)
// AS5600 shaft value is weighted more influenced to the correction system (60%)
const float MPU_WEIGHT = 0.40;  // Weight for MPU output (40%)
const float SHAFT_WEIGHT = 0.60; // Weight for AS5600 shaft angle (60%)
const float ANGLE_THRESHOLD = 1.0; // Minimum angle change (in degrees) to detect movement and start tracking.
                                   // This is the "minimal value for calculated the time if the final fusion value is more than 1 degree."
const float GEAR_RATIO = 19.0;  // Gearbox ratio (1:19) for AS5600 angle calculation
const float MOVEMENT_THRESHOLD = 15.0; // Minimum total movement (in degrees) required to trigger response time calculation.
                                       // "so the respond time only will count the movement if it more than 15 degree."
const float STABILITY_THRESHOLD = 0.3; // Angle change (in degrees) below which the system is considered stable.
                                       // Used to predict "not oscillate again with big gap."
const int STABLE_COUNT_REQUIRED = 8;  // Number of consecutive stable readings required to confirm stability.
                                      // This helps ensure the value is truly constant again.

// ------------------ ENHANCED RESPONSE TIME CALCULATION VARIABLES ----------------------
enum MovementState { IDLE, TRACKING, TIMING }; // States for the response time finite state machine
MovementState currentState = IDLE; // Current state of movement detection
unsigned long movementStartTime = 0; // Timestamp when movement was first detected (constant -> moving)
float movementStartAngle = 0.0; // Fused angle at the start of movement
float lastValidResponseTime = 0.0; // Stores the last calculated response time, retains its value
float maxMovementDeviation = 0.0;  // Tracks the maximum angular deviation from 'movementStartAngle'
int stableCount = 0; // Counter for consecutive stable readings
bool responseTimeCalculated = false; // Flag to indicate if a new response time was just finalized

// ------------------ AS5600 READ FUNCTION ------------------
// Reads the 12-bit raw angle from the AS5600 sensor
uint16_t readAS5600RawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_HIGH); // Request high byte of raw angle
  Wire.endTransmission(false); // Do not release the bus
  Wire.requestFrom(AS5600_ADDR, 2); // Request 2 bytes (high and low)
  if (Wire.available() < 2) return 0xFFFF;  // Return error value if not enough bytes received
  
  uint8_t high = Wire.read(); // Read high byte
  uint8_t low = Wire.read();  // Read low byte
  return ((high << 8) | low) & 0x0FFF; // Combine and mask to 12-bit value (0-4095)
}

// ------------------ ANGLE RESET FUNCTION ------------------
// Resets sensor offsets and response time tracking variables to initial states
void resetAngles() {
  // Reset MPU6050 angle offset
  mpu.getAcceleration(&AccX, &AccY, &AccZ); // Get current accelerometer readings
  float fAccX = (float)AccX;
  float fAccY = (float)AccY;
  float fAccZ = (float)AccZ;
  // Calculate roll angle from accelerometer and set as offset for MPU
  rollOffset = atan2(fAccY, sqrt(fAccX * fAccX + fAccZ * fAccZ)) * (180.0 / PI);
  
  // Reset shaft angle (AS5600) offset
  uint16_t rawAngleInit = readAS5600RawAngle();
  if (rawAngleInit != 0xFFFF) {
    prevAngleDegAS5600 = (rawAngleInit * 360.0) / 4096.0; // Convert raw to degrees
    motorAngleTotal = 0.0; // Reset total motor rotation
    shaftOffset = 0.0; // Reset shaft offset
  }
  
  // Reset all response time tracking variables
  prevFusedAngle = 0.0; // Reset previous fused angle for fresh start
  resetRequested = false;
  // angleChanged = false; // Not needed
  currentState = IDLE;
  lastValidResponseTime = 0.0; // On reset, clear the stored response time
  maxMovementDeviation = 0.0;
  stableCount = 0; // Reset stability counter
  responseTimeCalculated = false; // New calculation will happen

  Serial.println("=== SISTEM RESET - SUDUT DIKALIBRASI KE 0 ===");
}

// ------------------ ANGLE NORMALIZATION FUNCTION --------------------
// Normalizes an angle to the range of -180 to +180 degrees
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// ------------------ SETUP FUNCTION -------------------------------
void setup() {
  Serial.begin(9600); // Initialize serial communication
  Wire.begin();  // Initialize I2C communication for MPU6050 and AS5600

  // Initialize MPU6050 sensor
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 tidak terhubung!"); // MPU6050 not connected error
    while (1); // Halt execution if MPU is not found
  }
  Serial.println("MPU6050 terhubung!"); // MPU6050 connected confirmation

  // Configure Modbus TCP
  uint8_t ip[] = IP_ADDR; // IP address for Modbus
  uint8_t mac[] = MAC_ADDR; // MAC address for Modbus
  mb.config(mac, ip); // Apply Modbus configuration
  // Add holding registers for Modbus communication
  mb.addHreg(MPU);
  mb.addHreg(SHAFT_REG);
  mb.addHreg(FINAL_FUSION_REG);
  mb.addHreg(RESPONSE_TIME_REG);

  // Perform initial angle reset/calibration
  resetAngles();

  Serial.println("=== SISTEM SIAP - RESPONSE TIME TRACKER ===");
  Serial.println("- Timer dimulai saat ada pergerakan >= 1° (ANGLE_THRESHOLD)");
  Serial.println("- Response time dihitung HANYA jika total pergerakan >= 15° (MOVEMENT_THRESHOLD)");
  Serial.println("- Timer dihitung dari awal pergerakan hingga stabil");
  Serial.println("- Kirim 'r' untuk reset");
  Serial.println("==========================================");
}

// ------------------ MAIN LOOP FUNCTION --------------------------------
void loop() {
  unsigned long currentTime = millis(); // Get current system time in milliseconds
  
  if (resetRequested) {
    resetAngles(); // Perform reset if requested
  }

  // ---------- MPU6050: Roll Angle Calculation ----------
  // MPU6050's internal processing (or basic accelerometer-derived roll) serves as one input
  // in the hierarchical fusion.
  mpu.getAcceleration(&AccX, &AccY, &AccZ); // Read raw accelerometer data
  float fAccX = (float)AccX;
  float fAccY = (float)AccY;
  float fAccZ = (float)AccZ;

  float rawRoll = atan2(fAccY, sqrt(fAccX * fAccX + fAccZ * fAccZ)) * (180.0 / PI); // Calculate raw roll from accelerometer
  float mpuRoll = normalizeAngle(rawRoll - rollOffset); // Apply offset and normalize MPU roll

  // ---------- AS5600: Shaft Angle Calculation ----------
  float shaftAngle = 0.0;
  uint16_t rawAngle = readAS5600RawAngle(); // Read raw angle from AS5600
  
  if (rawAngle != 0xFFFF) { // Check for valid AS5600 reading
    float angleDegAS5600 = (rawAngle * 360.0) / 4096.0; // Convert raw 12-bit value to degrees (0-360)
    float deltaAngle = angleDegAS5600 - prevAngleDegAS5600; // Calculate change in angle

    // Correct for 360° wrap-around (e.g., from 350 to 10 degrees should be +20, not -340)
    if (deltaAngle > 180) deltaAngle -= 360;
    if (deltaAngle < -180) deltaAngle += 360;

    // Invert direction if needed to match MPU6050's CCW positive convention
    deltaAngle = -deltaAngle;

    motorAngleTotal += deltaAngle; // Accumulate total rotation
    prevAngleDegAS5600 = angleDegAS5600; // Store current angle for next iteration's delta

    shaftAngle = normalizeAngle((motorAngleTotal / GEAR_RATIO) - shaftOffset); // Apply gear ratio, offset, and normalize
  }

  // ---------- HIERARCHICAL & COMPLEMENTARY FUSION (WEIGHTED AVERAGE) ----------
  // This section implements Config 9's fusion: MPU's output is combined with AS5600's output.
  // AS5600 has higher influence (60%) as requested, acting as the primary correction.
  float fusedAngle = (MPU_WEIGHT * mpuRoll) + (SHAFT_WEIGHT * shaftAngle);
  fusedAngle = normalizeAngle(fusedAngle); // Normalize the final fused angle

  // ---------- ENHANCED RESPONSE TIME CALCULATION LOGIC ----------
  // This state machine manages the response time measurement based on specified criteria.
  float angleChange = abs(fusedAngle - prevFusedAngle); // Absolute change in fused angle from last reading
  if (angleChange > 180) angleChange = 360 - angleChange;  // Handle wrap-around for angle change
  
  switch (currentState) {
    case IDLE:
      // In IDLE, we wait for the initial movement to begin.
      // "minimal value for calculated the time if the final fusion value is more than 1 degree."
      if (angleChange >= ANGLE_THRESHOLD) {
        currentState = TRACKING; // Transition to TRACKING state
        movementStartTime = currentTime;  // Timer starts here (from constant to moving)
        movementStartAngle = fusedAngle;  // Record the starting angle of the movement
        maxMovementDeviation = 0.0;       // Reset max deviation for the new movement
        stableCount = 0;                  // Reset stability counter
        responseTimeCalculated = false;   // Flag that a new calculation is in progress
        
        Serial.print(">>> PERGERAKAN TERDETEKSI - Timer dimulai dari: ");
        Serial.print(fusedAngle, 2);
        Serial.println("°");
      }
      break;
      
    case TRACKING:
      {
        // In TRACKING, we monitor the total deviation and wait to cross the MOVEMENT_THRESHOLD.
        // Track maximum deviation from the starting angle of the current movement
        float currentDeviation = abs(fusedAngle - movementStartAngle);
        if (currentDeviation > 180) currentDeviation = 360 - currentDeviation; // Correct for wrap-around
        if (currentDeviation > maxMovementDeviation) {
          maxMovementDeviation = currentDeviation; // Update max deviation
        }
        
        // "so the respond time only will count the movement if it more than 15 degree."
        if (maxMovementDeviation >= MOVEMENT_THRESHOLD) {
          currentState = TIMING; // Transition to TIMING state
          Serial.print(">>> THRESHOLD 15° TERCAPAI - Mulai menghitung response time! (Total: ");
          Serial.print(maxMovementDeviation, 1);
          Serial.println("°)");
        }
        
        // If movement stops before reaching the 15-degree threshold, reset to IDLE.
        if (angleChange < STABILITY_THRESHOLD) {
          stableCount++;
          if (stableCount >= STABLE_COUNT_REQUIRED) {
            Serial.print(">>> Pergerakan berhenti sebelum 15° (");
            Serial.print(maxMovementDeviation, 1);
            Serial.println("°) - Kembali ke IDLE");
            currentState = IDLE;
            stableCount = 0; // Reset for next movement
          }
        } else {
          stableCount = 0; // Reset if movement continues
        }
      }
      break;
      
    case TIMING:
      {
        // In TIMING, we continue tracking deviation and look for stability to stop the timer.
        // Continue tracking maximum deviation during the timing phase
        float currentDeviation = abs(fusedAngle - movementStartAngle);
        if (currentDeviation > 180) currentDeviation = 360 - currentDeviation;
        if (currentDeviation > maxMovementDeviation) {
          maxMovementDeviation = currentDeviation;
        }
        
        // Check for stability ("constant again" phase, "not oscillate again with big gap")
        // This is based on the change between consecutive readings being very small (STABILITY_THRESHOLD)
        // for a required number of consecutive readings (STABLE_COUNT_REQUIRED).
        if (angleChange < STABILITY_THRESHOLD) {
          stableCount++;
          
          if (stableCount >= STABLE_COUNT_REQUIRED) {
            // System is stable - calculate response time from the very beginning of movement
            // "the respond time is suppose to calculate the final fusion moving time from constant - moving - constant again."
            lastValidResponseTime = currentTime - movementStartTime; // Store the calculated time directly
            responseTimeCalculated = true; // Set flag for logging a new calculation
            
            // Log completion details
            Serial.println("==========================================");
            Serial.println(">>> SISTEM STABIL - RESPONSE TIME SELESAI! <<<");
            Serial.print("    Sudut awal pergerakan: ");
            Serial.print(movementStartAngle, 2);
            Serial.println("°");
            Serial.print("    Sudut akhir stabil: ");
            Serial.print(fusedAngle, 2);
            Serial.println("°");
            Serial.print("    Total pergerakan terdeteksi: ");
            Serial.print(maxMovementDeviation, 1);
            Serial.println("°");
            Serial.print("    RESPONSE TIME: ");
            Serial.print(lastValidResponseTime);
            Serial.println(" ms");
            Serial.println("==========================================");
            
            // Reset to IDLE state for the next movement detection cycle
            currentState = IDLE;
            stableCount = 0; // Reset for next movement
          }
        } else {
          // Reset stability counter if significant movement is detected, preventing false stability detection
          stableCount = 0;
        }
      }
      break;
  }

  prevFusedAngle = fusedAngle; // Update previous fused angle for next iteration's change calculation

  // ---------- SAVE TO MODBUS REGISTERS ----------
  mb.Hreg(MPU, (int16_t)(mpuRoll * 100)); // MPU output (multiplied by 100 for precision)
  mb.Hreg(SHAFT_REG, (int16_t)(shaftAngle * 100)); // AS5600 shaft angle (multiplied by 100 for precision)
  mb.Hreg(FINAL_FUSION_REG, (int16_t)(fusedAngle * 100)); // Final Hierarchical Fusion value (multiplied by 100)
  mb.Hreg(RESPONSE_TIME_REG, (uint16_t)lastValidResponseTime); // Last calculated response time (ms)

  // ---------- SERIAL MONITOR DISPLAY ----------
  Serial.print("MPU: ");
  Serial.print(mpuRoll, 1);
  Serial.print("° | Shaft: ");
  Serial.print(shaftAngle, 1);
  Serial.print("° | Fused: ");
  Serial.print(fusedAngle, 2);
  Serial.print("° | State: ");
  
  // Display current state for debugging
  switch(currentState) {
    case IDLE:
      Serial.print("IDLE");
      break;
    case TRACKING:
      Serial.print("TRACKING (Dev:");
      Serial.print(maxMovementDeviation, 1);
      Serial.print("°)");
      break;
    case TIMING:
      Serial.print("TIMING (Dev:");
      Serial.print(maxMovementDeviation, 1);
      Serial.print("°,Stable:");
      Serial.print(stableCount);
      Serial.print("/");
      Serial.print(STABLE_COUNT_REQUIRED);
      Serial.print(")");
      break;
  }
  
  Serial.print(" | RT: ");
  // "the time calculated not change to 0 again but still at the last value."
  // Only print "NEW CALC" when a new response time has just been determined.
  if (responseTimeCalculated) {
    Serial.print(lastValidResponseTime);
    Serial.println("ms *** NEW CALC ***");
    responseTimeCalculated = false; // Reset flag after displaying
  } else {
    Serial.print(lastValidResponseTime);
    Serial.println("ms"); // Continuously display the last valid response time
  }

  // ---------- HANDLE MODBUS COMMUNICATION ----------
  mb.task(); // Process Modbus requests

  // ---------- CHECK FOR RESET COMMAND ----------
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      resetRequested = true; // Set flag to trigger reset in next loop iteration
      Serial.println(">>> RESET DIMINTA <<<");
    }
  }

  delay(50); // Small delay to prevent overwhelming the serial monitor and allow I2C/Modbus to function smoothly
}
