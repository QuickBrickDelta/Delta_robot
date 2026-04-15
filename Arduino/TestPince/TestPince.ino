#include <Dynamixel2Arduino.h>
#include <Servo.h>

// ================================
// TEST MOTEURS + PINCE (Version Stable)
// o / p -> Pince (Pin 3)
// g / f -> Poignet (Pin 5)
// r     -> Lire positions
// t / e -> Moteurs libres / verrouillés
// ================================

#define DEBUG_SERIAL Serial
#define DXL_SERIAL Serial1

const uint8_t ID_M1 = 1;
const uint8_t ID_M2 = 2;
const uint8_t ID_M3 = 3;

// Tes nouvelles limites de pince
int pulsePinceOuvert = 1650;
int pulsePinceFerme = 1850;

// Tes limites de poignet
int pulseWristOuvert = 550;
int pulseWristFerme = 1700;

Dynamixel2Arduino dxl(DXL_SERIAL);
Servo pinceServo;
Servo wristServo;

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) { ; }

  DXL_SERIAL.begin(57600);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  uint8_t ids[] = {ID_M1, ID_M2, ID_M3};
  for (int i = 0; i < 3; i++) {
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_EXTENDED_POSITION);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, ids[i], 80);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, ids[i], 30);
    dxl.torqueOn(ids[i]);
  }

  pinceServo.attach(3, 500, 2500);
  pinceServo.writeMicroseconds(pulsePinceOuvert);

  wristServo.attach(5, 500, 2500);
  wristServo.writeMicroseconds(pulseWristOuvert);

  DEBUG_SERIAL.println("=== TEST MATERIEL OK ===");
  DEBUG_SERIAL.println("Touches : o, p (pince) | g, f (poignet) | r (lire)");
}

void loop() {
  if (DEBUG_SERIAL.available() > 0) {
    String input = DEBUG_SERIAL.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    char c = input.charAt(0);

    // 1. Commandes spéciales avec paramètres (ex: W:45 ou P:1600)
    if (input.startsWith("W:") || input.startsWith("w:")) {
      float angle = input.substring(2).toFloat();
      int pulse = map(angle, -90, 90, 500, 1700);
      wristServo.writeMicroseconds(pulse);
      DEBUG_SERIAL.print(">> POIGNET ANGLE: "); DEBUG_SERIAL.print(angle);
      DEBUG_SERIAL.print(" -> PWM: "); DEBUG_SERIAL.println(pulse);
    }
    else if (input.startsWith("P:") || input.startsWith("p:")) {
      int pulse = input.substring(2).toInt();
      pinceServo.writeMicroseconds(pulse);
      DEBUG_SERIAL.print(">> PINCE PWM DIRECT: "); DEBUG_SERIAL.println(pulse);
    }
    
    // 2. Commandes à une seule lettre (Sécurisées par length == 1)
    else if (input.length() == 1) {
      if (c == 'r' || c == 'R') {
        DEBUG_SERIAL.print("M1:"); DEBUG_SERIAL.print(dxl.getPresentPosition(ID_M1));
        DEBUG_SERIAL.print(" M2:"); DEBUG_SERIAL.print(dxl.getPresentPosition(ID_M2));
        DEBUG_SERIAL.print(" M3:"); DEBUG_SERIAL.println(dxl.getPresentPosition(ID_M3));
      }
      else if (c == 'o' || c == 'O') {
        pinceServo.writeMicroseconds(pulsePinceOuvert);
        DEBUG_SERIAL.print(">> PINCE OUVERTE: "); DEBUG_SERIAL.println(pulsePinceOuvert);
      }
      else if (c == 'p' || c == 'P') {
        pinceServo.writeMicroseconds(pulsePinceFerme);
        DEBUG_SERIAL.print(">> PINCE FERMEE: "); DEBUG_SERIAL.println(pulsePinceFerme);
      }
      else if (c == 'g' || c == 'G') {
        wristServo.writeMicroseconds(pulseWristOuvert);
        DEBUG_SERIAL.print(">> POIGNET POSITION G: "); DEBUG_SERIAL.println(pulseWristOuvert);
      }
      else if (c == 'f' || c == 'F') {
        wristServo.writeMicroseconds(pulseWristFerme);
        DEBUG_SERIAL.print(">> POIGNET POSITION F: "); DEBUG_SERIAL.println(pulseWristFerme);
      }
      else if (c == 't' || c == 'T') {
        dxl.torqueOff(ID_M1); dxl.torqueOff(ID_M2); dxl.torqueOff(ID_M3);
        DEBUG_SERIAL.println(">> MOTEURS LIBRES");
      }
      else if (c == 'e' || c == 'E') {
        dxl.torqueOn(ID_M1); dxl.torqueOn(ID_M2); dxl.torqueOn(ID_M3);
        DEBUG_SERIAL.println(">> MOTEURS VERROUILLES");
      }
      else {
        DEBUG_SERIAL.print("Inconnu: "); DEBUG_SERIAL.println(c);
      }
    }
    
    // 3. Format M1,M2,M3
    else if (input.indexOf(',') > 0) {
      int c1 = input.indexOf(','); int c2 = input.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > c1) {
        dxl.setGoalPosition(ID_M1, input.substring(0, c1).toInt());
        dxl.setGoalPosition(ID_M2, input.substring(c1 + 1, c2).toInt());
        dxl.setGoalPosition(ID_M3, input.substring(c2 + 1).toInt());
        DEBUG_SERIAL.println(">> OK MOUVEMENT BRAS");
      }
    }
  }
}
