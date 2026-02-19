#include <Dynamixel2Arduino.h>

// ================================
// Configuration
// ================================
#define DEBUG_SERIAL Serial
#define DXL_SERIAL   Serial1

const uint8_t ID_M1    = 1;
const uint8_t ID_M2    = 2;
const uint8_t ID_M3    = 3;
const uint8_t ID_PINCE = 4;

// ================================
// CALIBRATION — Auto au démarrage
// ================================
// Au boot, le robot doit être en position REPOS (bras tout droit vers le bas)
// L'Arduino lit les positions actuelles comme offsets.
// theta=0 -> bras vertical vers le bas
// theta=PI/2 -> bras horizontal
int32_t REPOS_M1 = 0;
int32_t REPOS_M2 = 0;
int32_t REPOS_M3 = 0;

// Ticks par radian (4095 ticks / tour complet)
const float TICKS_PER_RAD = 4095.0f / (2.0f * PI);

// ================================
// Vitesse et pince
// ================================
const uint32_t PROFILE_VELOCITY = 50;  // 30=lent, 100=modéré
const int32_t PINCE_OUVERTE = 1500;
const int32_t PINCE_FERMEE  = 3000;

Dynamixel2Arduino dxl(DXL_SERIAL);

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
bool  newCommand = false;
uint32_t cmdCount = 0;

// ================================
// Conversion angle -> ticks
// ================================
// tick = REPOS - theta * TICKS_PER_RAD

int32_t thetaToTicks(float theta_rad, int32_t repos_offset) {
  return repos_offset - (int32_t)(theta_rad * TICKS_PER_RAD);
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
  //    → Le robot doit être en position repos (bras en bas)
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

  // 2) Configurer les moteurs
  uint8_t ids[] = {ID_M1, ID_M2, ID_M3, ID_PINCE};
  for (int i = 0; i < 4; i++) {
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_EXTENDED_POSITION);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, ids[i], PROFILE_VELOCITY);
    dxl.torqueOn(ids[i]);
  }

  DEBUG_SERIAL.print("Velocity=");
  DEBUG_SERIAL.println(PROFILE_VELOCITY);
  DEBUG_SERIAL.println("Pret. Place le robot bras en bas avant de demarrer !");
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
  lastPince  = (vals[3] >= 0.5f);
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
    DEBUG_SERIAL.println("]");
  }
}

// ================================
// Application de la commande
// ================================
void applyLastCommand() {
  dxl.setGoalPosition(ID_M1, thetaToTicks(lastTheta1, REPOS_M1));
  dxl.setGoalPosition(ID_M2, thetaToTicks(lastTheta2, REPOS_M2));
  dxl.setGoalPosition(ID_M3, thetaToTicks(lastTheta3, REPOS_M3));

  if (lastPince) {
    dxl.setGoalPosition(ID_PINCE, PINCE_FERMEE);
  } else {
    dxl.setGoalPosition(ID_PINCE, PINCE_OUVERTE);
  }
}
