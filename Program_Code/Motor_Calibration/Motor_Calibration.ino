#include <Wire.h>
#include <AS5600.h>

const int dirPin = 4;
const int stepPin = 3;

const int stepsPerRevolution = 200;
const float thresholdDegree = 1.0; // toleransi sekitar 0 derajat

AS5600 encoder;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, HIGH); // Putar searah jarum jam

  Serial.begin(9600);
  Wire.begin();

  // Inisialisasi AS5600
  if (!encoder.begin()) {
    Serial.println("AS5600 tidak terdeteksi!");
    while (1);
  }
}

void loop() {
  // Baca sudut dari AS5600 (0 - 4095)
  uint16_t angleRaw = encoder.readAngle();
  float angleDeg = (angleRaw / 4096.0) * 360.0;

  Serial.print("Angle: ");
  Serial.println(angleDeg);

  // Hentikan motor jika mendekati 0 derajat
  if (angleDeg < thresholdDegree || angleDeg > (360.0 - thresholdDegree)) {
    Serial.println("Sudut 0 derajat tercapai. Motor berhenti.");
    while (true); // Hentikan program
  }

  // Jalankan stepper motor
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(500);
}
