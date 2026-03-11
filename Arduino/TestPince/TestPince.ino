#include <Servo.h>

// ================================
// TEST PINCE — writeMicroseconds
// Test avec des pulses directes, pas des angles
// ================================

const int SERVO_PIN = 5;

// Pulse widths standard pour un servo :
// 500μs  = butée min
// 1500μs = centre
// 2500μs = butée max
const int PULSE_OUVERT = 1000; // Position ouverte
const int PULSE_FERME = 2000;  // Position fermée
const int DELAI_MS = 2000;

Servo pinceServo;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  pinceServo.attach(SERVO_PIN, 500, 2500); // Min/max pulse width
  Serial.println("=== TEST PINCE (Microseconds) ===");
  Serial.println("Cycle: OUVERT -> 2s -> FERME -> 2s -> ...");
}

void loop() {
  // Ouvrir
  pinceServo.writeMicroseconds(PULSE_OUVERT);
  Serial.print(">> OUVERT (");
  Serial.print(PULSE_OUVERT);
  Serial.println(" us)");
  delay(DELAI_MS);

  // Fermer
  pinceServo.writeMicroseconds(PULSE_FERME);
  Serial.print(">> FERME (");
  Serial.print(PULSE_FERME);
  Serial.println(" us)");
  delay(DELAI_MS);
}
