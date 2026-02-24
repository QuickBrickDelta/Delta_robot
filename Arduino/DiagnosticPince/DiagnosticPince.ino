#include <Servo.h>

// ================================
// DIAGNOSTIC PINCE — Servo standard (PWM)
// Envoie 'o' (ouvrir) / 'f' (fermer) via Serial Monitor
// ou un angle (0-180) directement
// ================================

#define DEBUG_SERIAL Serial

const int SERVO_PIN = 1;  // Pin où le servo est branché

// Angles à tester — MODIFIE ICI si besoin
int ANGLE_OUVERTE = 90;
int ANGLE_FERMEE  = 0;

Servo pinceServo;

bool pinceEstFermee = false;

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) { ; }

  pinceServo.attach(SERVO_PIN);

  DEBUG_SERIAL.println("=== DIAGNOSTIC PINCE (Servo PWM) ===");
  DEBUG_SERIAL.print("Pin: ");
  DEBUG_SERIAL.println(SERVO_PIN);
  DEBUG_SERIAL.println("");
  DEBUG_SERIAL.println("Commandes (Serial Monitor):");
  DEBUG_SERIAL.println("  'o'       -> Ouvrir (angle OUVERTE)");
  DEBUG_SERIAL.println("  'f'       -> Fermer (angle FERMEE)");
  DEBUG_SERIAL.println("  0-180     -> Aller a cet angle directement");
  DEBUG_SERIAL.println("  'p'       -> Lire angle actuel");
  DEBUG_SERIAL.print("\nOUVERTE=");
  DEBUG_SERIAL.print(ANGLE_OUVERTE);
  DEBUG_SERIAL.print(" / FERMEE=");
  DEBUG_SERIAL.println(ANGLE_FERMEE);
  DEBUG_SERIAL.println("---");

  // Ouvrir au départ
  pinceServo.write(ANGLE_OUVERTE);
  DEBUG_SERIAL.print(">> OUVERTE (angle=");
  DEBUG_SERIAL.print(ANGLE_OUVERTE);
  DEBUG_SERIAL.println(")");
}

void loop() {
  if (DEBUG_SERIAL.available() > 0) {
    String input = DEBUG_SERIAL.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    char c = input.charAt(0);

    if (c == 'o' || c == 'O') {
      pinceServo.write(ANGLE_OUVERTE);
      pinceEstFermee = false;
      DEBUG_SERIAL.print(">> OUVERTE (angle=");
      DEBUG_SERIAL.print(ANGLE_OUVERTE);
      DEBUG_SERIAL.println(")");
    }
    else if (c == 'f' || c == 'F') {
      pinceServo.write(ANGLE_FERMEE);
      pinceEstFermee = true;
      DEBUG_SERIAL.print(">> FERMEE (angle=");
      DEBUG_SERIAL.print(ANGLE_FERMEE);
      DEBUG_SERIAL.println(")");
    }
    else if (c == 'p' || c == 'P') {
      DEBUG_SERIAL.print("Angle actuel: ");
      DEBUG_SERIAL.println(pinceServo.read());
    }
    else if (c >= '0' && c <= '9') {
      int angle = input.toInt();
      if (angle >= 0 && angle <= 180) {
        pinceServo.write(angle);
        DEBUG_SERIAL.print(">> Angle = ");
        DEBUG_SERIAL.println(angle);
      } else {
        DEBUG_SERIAL.println("ERR: angle doit etre entre 0 et 180");
      }
    }
    else {
      DEBUG_SERIAL.print("Commande inconnue: ");
      DEBUG_SERIAL.println(input);
    }
  }
}
