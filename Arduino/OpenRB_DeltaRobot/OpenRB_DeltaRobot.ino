#include <Dynamixel2Arduino.h>
#include <Servo.h>

// ================================
// Configuration
// ================================
#define DEBUG_SERIAL Serial
#define DXL_SERIAL   Serial1

const uint8_t ID_M1    = 1;
const uint8_t ID_M2    = 2;
const uint8_t ID_M3    = 3;

// ================================
// CALIBRATION — Auto au démarrage
// ================================
// Au boot, le robot doit être en position z_table (0, 0, -40)
// L'Arduino lit les positions actuelles comme offsets.
// THETA_CALIB = angle des moteurs à la position z_table
int32_t REPOS_M1 = 0;
int32_t REPOS_M2 = 0;
int32_t REPOS_M3 = 0;

// Angle moteurs à la position de calibration z_table (0, 0, -40)
const float THETA_CALIB = 0.7268f; // rad (~41.6°)

// Ticks par radian (4095 ticks / tour complet)
const float TICKS_PER_RAD = 4095.0f / (2.0f * PI);

// ================================
// Vitesse et pince (servo PWM)
// ================================
const uint32_t PROFILE_VELOCITY = 50;  // 30=lent, 100=modéré
const int SERVO_PINCE_PIN = 1;   // Pin PWM du servo pince
const int PINCE_OUVERTE   = 45;  // Angle ouvert (degrés)
const int PINCE_FERMEE    = 90;  // Angle fermé (degrés)

Dynamixel2Arduino dxl(DXL_SERIAL);
Servo pinceServo;

// ================================
// Variables
// ================================
const size_t LINE_BUF_SIZE = 64;
char   lineBuf[LINE_BUF_SIZE];
size_t linePos = 0;

float lastTheta1 = 0.0f;
float lastTheta2 = 0.0f;
float lastTheta3 = 0.0f;
bool  lastPince  = false;
bool  prevPinceState = false;  // Pour détecter les changements
bool  newCommand = false;
uint32_t cmdCount = 0;

// ================================
// Conversion angle -> ticks
// ================================
// tick = REPOS - (theta - THETA_CALIB) * TICKS_PER_RAD
// Au boot (theta == THETA_CALIB) → delta = 0 → position = REPOS ✓

int32_t thetaToTicks(float theta_rad, int32_t repos_offset) {
  return repos_offset - (int32_t)((theta_rad - THETA_CALIB) * TICKS_PER_RAD);
}

// ================================
// Setup
// ================================
void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) { ; }

  DXL_SERIAL.begin(57600);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  // 1) Lire les positions actuelles AVANT d'activer le torque
  //    → Le robot doit être en position z_table (0, 0, -40)
  REPOS_M1 = dxl.getPresentPosition(ID_M1);
  REPOS_M2 = dxl.getPresentPosition(ID_M2);
  REPOS_M3 = dxl.getPresentPosition(ID_M3);

  DEBUG_SERIAL.println("=== OpenRB Delta Robot ===");
  DEBUG_SERIAL.println(">> AUTO-CALIBRATION <<");
  DEBUG_SERIAL.print("Repos lu: M1=");
  DEBUG_SERIAL.print(REPOS_M1);
  DEBUG_SERIAL.print(" M2=");
  DEBUG_SERIAL.print(REPOS_M2);
  DEBUG_SERIAL.print(" M3=");
  DEBUG_SERIAL.println(REPOS_M3);

  // 2) Configurer les moteurs Dynamixel (3 bras)
  uint8_t ids[] = {ID_M1, ID_M2, ID_M3};
  for (int i = 0; i < 3; i++) {
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_EXTENDED_POSITION);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, ids[i], PROFILE_VELOCITY);
    dxl.torqueOn(ids[i]);
  }

  // 3) Configurer le servo pince (PWM)
  pinceServo.attach(SERVO_PINCE_PIN);
  pinceServo.write(PINCE_OUVERTE);

  DEBUG_SERIAL.print("Velocity=");
  DEBUG_SERIAL.println(PROFILE_VELOCITY);
  DEBUG_SERIAL.println("Pret. Place le robot a z_table (0,0,-40) avant de demarrer !");
}

// ================================
// Loop — pas de delay pour max vitesse
// ================================
void loop() {
  readSerialLines();

  if (newCommand) {
    applyLastCommand();
    newCommand = false;
  }
}

// ================================
// Lecture série
// ================================
void readSerialLines() {
  while (DEBUG_SERIAL.available() > 0) {
    char c = DEBUG_SERIAL.read();
    if (c == '\r') continue;

    if (c == '\n') {
      lineBuf[linePos] = '\0';
      parseCommandLine(lineBuf);
      linePos = 0;
    } else {
      if (linePos < LINE_BUF_SIZE - 1) {
        lineBuf[linePos++] = c;
      } else {
        linePos = 0;
      }
    }
  }
}

// ================================
// Parsing CSV
// ================================
void parseCommandLine(const char* line) {
  char buf[LINE_BUF_SIZE];
  strncpy(buf, line, LINE_BUF_SIZE);
  buf[LINE_BUF_SIZE - 1] = '\0';

  float vals[4];
  int count = 0;

  char* token = strtok(buf, ",");
  while (token != NULL && count < 4) {
    vals[count] = atof(token);
    count++;
    token = strtok(NULL, ",");
  }

  if (count < 4) {
    DEBUG_SERIAL.print("ERR: ");
    DEBUG_SERIAL.println(line);
    return;
  }

  lastTheta1 = vals[0];
  lastTheta2 = vals[1];
  lastTheta3 = vals[2];
  bool newPince = (vals[3] >= 0.5f);
  lastPince = newPince;
  prevPinceState = newPince;

  newCommand = true;
  cmdCount++;

  // Debug léger : 1 message sur 30
  if (cmdCount % 30 == 0) {
    DEBUG_SERIAL.print("#");
    DEBUG_SERIAL.print(cmdCount);
    DEBUG_SERIAL.print(" [");
    DEBUG_SERIAL.print(thetaToTicks(lastTheta1, REPOS_M1));
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(thetaToTicks(lastTheta2, REPOS_M2));
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(thetaToTicks(lastTheta3, REPOS_M3));
    DEBUG_SERIAL.print("] P=");
    DEBUG_SERIAL.println(lastPince ? "FERME" : "OUVERT");
  }
}

void applyLastCommand() {
  // 1) Moteurs Dynamixel
  dxl.setGoalPosition(ID_M1, thetaToTicks(lastTheta1, REPOS_M1));
  dxl.setGoalPosition(ID_M2, thetaToTicks(lastTheta2, REPOS_M2));
  dxl.setGoalPosition(ID_M3, thetaToTicks(lastTheta3, REPOS_M3));

  // 2) Servo pince — APRES les Dynamixel pour ne pas être perturbé
  //    Rafraîchir à chaque frame pour maintenir le signal PWM
  pinceServo.write(lastPince ? PINCE_FERMEE : PINCE_OUVERTE);
}
