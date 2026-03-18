#include <Dynamixel2Arduino.h>
#include <Servo.h>

// ================================
// TEACH POINTS — Routine de calibration
// ================================
// Torque OFF → déplace le robot à la main
// Tape le numéro du point pour enregistrer la position
// Tape 'p' pour afficher tous les points enregistrés
// Tape 'o'/'f' pour ouvrir/fermer la pince
// ================================

#define DEBUG_SERIAL Serial
#define DXL_SERIAL Serial1

const uint8_t ID_M1 = 1;
const uint8_t ID_M2 = 2;
const uint8_t ID_M3 = 3;

// Servo pince
const int SERVO_PIN = 3;
const int PULSE_OUVERT = 1000;
const int PULSE_FERME = 2000;
Servo pinceServo;

// Points à enseigner
const int NUM_POINTS = 7;
const char *pointNames[NUM_POINTS] = {
    "Home (0,0,z_table)", "Pignon 1",   "Pignon 2",  "Pignon 3",
    "Drop Bac 1",         "Drop Bac 2", "Drop Bac 3"};

int32_t savedM1[NUM_POINTS];
int32_t savedM2[NUM_POINTS];
int32_t savedM3[NUM_POINTS];
bool pointSaved[NUM_POINTS];

Dynamixel2Arduino dxl(DXL_SERIAL);

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) {
    ;
  }

  DXL_SERIAL.begin(57600);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  // Torque OFF — le robot est libre, on le déplace à la main
  uint8_t ids[] = {ID_M1, ID_M2, ID_M3};
  for (int i = 0; i < 3; i++) {
    dxl.torqueOff(ids[i]);
  }

  // Servo pince
  pinceServo.attach(SERVO_PIN, 500, 2500);
  pinceServo.writeMicroseconds(PULSE_OUVERT);

  // Init
  for (int i = 0; i < NUM_POINTS; i++) {
    pointSaved[i] = false;
  }

  DEBUG_SERIAL.println("=== TEACH POINTS ===");
  DEBUG_SERIAL.println("Torque OFF — Deplace le robot a la main !");
  DEBUG_SERIAL.println("");
  DEBUG_SERIAL.println("Commandes :");
  DEBUG_SERIAL.println("  1-7  -> Enregistrer le point courant");
  for (int i = 0; i < NUM_POINTS; i++) {
    DEBUG_SERIAL.print("         ");
    DEBUG_SERIAL.print(i + 1);
    DEBUG_SERIAL.print(" = ");
    DEBUG_SERIAL.println(pointNames[i]);
  }
  DEBUG_SERIAL.println("  r    -> Lire position actuelle (live)");
  DEBUG_SERIAL.println("  p    -> Afficher tous les points sauvegardes");
  DEBUG_SERIAL.println("  o/f  -> Ouvrir/Fermer pince");
  DEBUG_SERIAL.println("---");
}

void loop() {
  if (DEBUG_SERIAL.available() > 0) {
    String input = DEBUG_SERIAL.readStringUntil('\n');
    input.trim();
    if (input.length() == 0)
      return;

    char c = input.charAt(0);

    // Enregistrer un point (1-7)
    if (c >= '1' && c <= '7') {
      int idx = c - '1'; // 0-6
      savedM1[idx] = dxl.getPresentPosition(ID_M1);
      savedM2[idx] = dxl.getPresentPosition(ID_M2);
      savedM3[idx] = dxl.getPresentPosition(ID_M3);
      pointSaved[idx] = true;

      DEBUG_SERIAL.print(">> POINT ");
      DEBUG_SERIAL.print(idx + 1);
      DEBUG_SERIAL.print(" [");
      DEBUG_SERIAL.print(pointNames[idx]);
      DEBUG_SERIAL.print("] = M1:");
      DEBUG_SERIAL.print(savedM1[idx]);
      DEBUG_SERIAL.print(" M2:");
      DEBUG_SERIAL.print(savedM2[idx]);
      DEBUG_SERIAL.print(" M3:");
      DEBUG_SERIAL.println(savedM3[idx]);
    }
    // Lire position live
    else if (c == 'r' || c == 'R') {
      int32_t m1 = dxl.getPresentPosition(ID_M1);
      int32_t m2 = dxl.getPresentPosition(ID_M2);
      int32_t m3 = dxl.getPresentPosition(ID_M3);
      DEBUG_SERIAL.print(">> LIVE : M1:");
      DEBUG_SERIAL.print(m1);
      DEBUG_SERIAL.print(" M2:");
      DEBUG_SERIAL.print(m2);
      DEBUG_SERIAL.print(" M3:");
      DEBUG_SERIAL.println(m3);
    }
    // Afficher tous les points
    else if (c == 'p' || c == 'P') {
      DEBUG_SERIAL.println("\n=== POINTS ENREGISTRES ===");
      for (int i = 0; i < NUM_POINTS; i++) {
        DEBUG_SERIAL.print("  ");
        DEBUG_SERIAL.print(i + 1);
        DEBUG_SERIAL.print(". ");
        DEBUG_SERIAL.print(pointNames[i]);
        if (pointSaved[i]) {
          DEBUG_SERIAL.print(" -> M1:");
          DEBUG_SERIAL.print(savedM1[i]);
          DEBUG_SERIAL.print(" M2:");
          DEBUG_SERIAL.print(savedM2[i]);
          DEBUG_SERIAL.print(" M3:");
          DEBUG_SERIAL.println(savedM3[i]);
        } else {
          DEBUG_SERIAL.println(" -> (pas enregistre)");
        }
      }
      DEBUG_SERIAL.println("========================\n");
    }
    // Pince
    else if (c == 'o' || c == 'O') {
      pinceServo.writeMicroseconds(PULSE_OUVERT);
      DEBUG_SERIAL.println(">> PINCE OUVERTE");
    } else if (c == 'f' || c == 'F') {
      pinceServo.writeMicroseconds(PULSE_FERME);
      DEBUG_SERIAL.println(">> PINCE FERMEE");
    } else {
      DEBUG_SERIAL.print("Commande inconnue: ");
      DEBUG_SERIAL.println(input);
    }
  }
}
