#include <Dynamixel2Arduino.h>
#include <Servo.h>

// ================================
// TEST MOTEURS + PINCE
// Envoie des positions en ticks via Serial Monitor
// Format: M1,M2,M3  (ex: 1300,3400,2350)
// Commandes: 'o'=ouvrir, 'f'=fermer, 'r'=lire position
// ================================

#define DEBUG_SERIAL Serial
#define DXL_SERIAL Serial1

const uint8_t ID_M1 = 1;
const uint8_t ID_M2 = 2;
const uint8_t ID_M3 = 3;

const uint32_t PROFILE_VELOCITY = 80;
const uint32_t PROFILE_ACCELERATION = 30;

// Servo pince
const int SERVO_PIN = 3;
const int PULSE_OUVERT = 1000;
const int PULSE_FERME = 2000;

Dynamixel2Arduino dxl(DXL_SERIAL);
Servo pinceServo;

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) {
    ;
  }

  DXL_SERIAL.begin(57600);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  uint8_t ids[] = {ID_M1, ID_M2, ID_M3};
  for (int i = 0; i < 3; i++) {
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_EXTENDED_POSITION);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, ids[i],
                              PROFILE_VELOCITY);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, ids[i],
                              PROFILE_ACCELERATION);
    dxl.torqueOn(ids[i]);
  }

  pinceServo.attach(SERVO_PIN, 500, 2500);
  pinceServo.writeMicroseconds(PULSE_OUVERT);

  // Lire position initiale
  int32_t m1 = dxl.getPresentPosition(ID_M1);
  int32_t m2 = dxl.getPresentPosition(ID_M2);
  int32_t m3 = dxl.getPresentPosition(ID_M3);

  DEBUG_SERIAL.println("=== TEST MOTEURS + PINCE ===");
  DEBUG_SERIAL.println("Commandes :");
  DEBUG_SERIAL.println("  M1,M2,M3  -> Aller a cette position (ticks)");
  DEBUG_SERIAL.println("  r         -> Lire position actuelle");
  DEBUG_SERIAL.println("  o / f     -> Ouvrir / Fermer pince");
  DEBUG_SERIAL.println("  t         -> Torque OFF (libre)");
  DEBUG_SERIAL.println("  e         -> Torque ON (verrouille)");
  DEBUG_SERIAL.print("Position actuelle: ");
  DEBUG_SERIAL.print(m1);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(m2);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(m3);
  DEBUG_SERIAL.println("---");
}

void loop() {
  if (DEBUG_SERIAL.available() > 0) {
    String input = DEBUG_SERIAL.readStringUntil('\n');
    input.trim();
    if (input.length() == 0)
      return;

    char c = input.charAt(0);

    // Lire position
    if (c == 'r' || c == 'R') {
      int32_t m1 = dxl.getPresentPosition(ID_M1);
      int32_t m2 = dxl.getPresentPosition(ID_M2);
      int32_t m3 = dxl.getPresentPosition(ID_M3);
      DEBUG_SERIAL.print(">> POS: ");
      DEBUG_SERIAL.print(m1);
      DEBUG_SERIAL.print(",");
      DEBUG_SERIAL.print(m2);
      DEBUG_SERIAL.print(",");
      DEBUG_SERIAL.println(m3);
    }
    // Pince
    else if (c == 'o' || c == 'O') {
      pinceServo.writeMicroseconds(PULSE_OUVERT);
      DEBUG_SERIAL.println(">> PINCE OUVERTE");
    } else if (c == 'f' || c == 'F') {
      pinceServo.writeMicroseconds(PULSE_FERME);
      DEBUG_SERIAL.println(">> PINCE FERMEE");
    }
    // Torque OFF
    else if (c == 't' || c == 'T') {
      dxl.torqueOff(ID_M1);
      dxl.torqueOff(ID_M2);
      dxl.torqueOff(ID_M3);
      DEBUG_SERIAL.println(">> TORQUE OFF (libre)");
    }
    // Torque ON
    else if (c == 'e' || c == 'E') {
      dxl.torqueOn(ID_M1);
      dxl.torqueOn(ID_M2);
      dxl.torqueOn(ID_M3);
      DEBUG_SERIAL.println(">> TORQUE ON");
    }
    // Position: M1,M2,M3
    else if (c >= '0' && c <= '9') {
      int comma1 = input.indexOf(',');
      int comma2 = input.indexOf(',', comma1 + 1);
      if (comma1 > 0 && comma2 > comma1) {
        int32_t t1 = input.substring(0, comma1).toInt();
        int32_t t2 = input.substring(comma1 + 1, comma2).toInt();
        int32_t t3 = input.substring(comma2 + 1).toInt();

        dxl.setGoalPosition(ID_M1, t1);
        dxl.setGoalPosition(ID_M2, t2);
        dxl.setGoalPosition(ID_M3, t3);

        DEBUG_SERIAL.print(">> GO: ");
        DEBUG_SERIAL.print(t1);
        DEBUG_SERIAL.print(",");
        DEBUG_SERIAL.print(t2);
        DEBUG_SERIAL.print(",");
        DEBUG_SERIAL.println(t3);
      } else {
        DEBUG_SERIAL.println("ERR: format M1,M2,M3 (ex: 1300,3400,2350)");
      }
    } else {
      DEBUG_SERIAL.print("Commande inconnue: ");
      DEBUG_SERIAL.println(input);
    }
  }
}
