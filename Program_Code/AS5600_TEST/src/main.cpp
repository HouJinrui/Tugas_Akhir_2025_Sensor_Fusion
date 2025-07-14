#include <Arduino.h>
#include <Wire.h>

// Pin definitions
#define DIR_PIN 3
#define STEP_PIN 4

// Stepper motor settings
const float STEPS_PER_REV = 30200.0;   // Change according to your motor
const float MICROSTEPS = 16.0;       // Change according to TB6600 dip switches
const float STEP_DELAY_US = 1000;    // Controls motor speed (increase to slow down)

// Encoder settings
const float TOLERANCE_DEGREES = 0.5; // Adjust based on required precision
const uint8_t AS5600_ADDRESS = 0x36; // I2C address for AS5600

float target_angle = 0.0;

// Function prototypes
void moveToAngle(float target);
float readAS5600();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  
  Serial.println("Stepper Motor Control System");
  Serial.println("Enter target angle (degrees):");
}

void loop() {
  if (Serial.available()) {
    target_angle = Serial.parseFloat();
    while (Serial.available()) Serial.read();  // Clear buffer
    
    target_angle = fmod(target_angle, 360.0);
    if (target_angle < 0) target_angle += 360.0;

    Serial.print("\nNew target: ");
    Serial.print(target_angle);
    Serial.println("°");

    moveToAngle(target_angle);
  }
}

void moveToAngle(float target) {
  float current_angle = readAS5600();
  current_angle = fmod(current_angle, 360.0);
  
  Serial.println("\nCurrent\tTarget\tSteps\tDirection");
  Serial.println("-------------------------------");

  unsigned long step_count = 0;
  bool target_reached = false;

  while (!target_reached) {
    float diff = target - current_angle;
    diff = fmod(diff + 360.0, 360.0);  // Normalize difference
    
    bool direction = diff <= 180.0;
    digitalWrite(DIR_PIN, direction ? HIGH : LOW);

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
    step_count++;

    float new_angle = readAS5600();
    new_angle = fmod(new_angle, 360.0);
    
    float remaining_diff = fabs(target - new_angle);
    remaining_diff = min(remaining_diff, 360.0 - remaining_diff);

    Serial.print(current_angle);
    Serial.print("°\t");
    Serial.print(target);
    Serial.print("°\t");
    Serial.print(step_count);
    Serial.print("\t");
    Serial.println(direction ? "CW" : "CCW");

    current_angle = new_angle;

    if (remaining_diff <= TOLERANCE_DEGREES) {
      target_reached = true;
      Serial.print("\nTarget reached in ");
      Serial.print(step_count);
      Serial.println(" steps");
      Serial.println("Enter new angle:");
    }
  }
}

float readAS5600() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C);  // Angle register
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDRESS, 2);

  if (Wire.available() >= 2) {
    byte high = Wire.read();
    byte low = Wire.read();
    uint16_t raw = (high << 8) | low;
    return (raw * 360.0) / 4096.0;
  }
  return -1.0;
}