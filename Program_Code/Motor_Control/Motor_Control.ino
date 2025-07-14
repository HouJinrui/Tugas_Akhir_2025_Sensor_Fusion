const int dirPin = 2;
const int stepPin = 3;
const float pulsesPerRev = 800;
const float gearRatio = 1;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Masukkan arah (CW / CCW) dan sudut (x): Contoh: CW 90");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      int spaceIndex = input.indexOf(' ');
      if (spaceIndex != -1) {
        String direction = input.substring(0, spaceIndex);
        float angle = input.substring(spaceIndex + 1).toFloat();

        long steps = (long)((angle / 360.0) * pulsesPerRev * gearRatio);

        // Atur arah
        if (direction.equalsIgnoreCase("CW")) {
          digitalWrite(dirPin, HIGH);
        } else if (direction.equalsIgnoreCase("CCW")) {
          digitalWrite(dirPin, LOW);
        } else {
          Serial.println("Arah tidak valid. Gunakan CW atau CCW.");
          return;
        }

        // Gerakkan stepper
        for (long i = 0; i < steps; i++) {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(500); // sesuaikan jika perlu
          digitalWrite(stepPin, LOW);
          delayMicroseconds(500);
        }

        Serial.print("Selesai gerak ");
        Serial.print(direction);
        Serial.print(" ");
        Serial.print(angle);
        Serial.println(" derajat.");
      } else {
        Serial.println("Format salah. Gunakan format: CW 90");
      }
    }
  }
}
