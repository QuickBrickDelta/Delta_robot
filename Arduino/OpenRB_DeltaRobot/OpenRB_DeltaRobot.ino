#include <Dynamixel2Arduino.h>

// ================================
// Configuration matérielle
// ================================

// Adapter ces définitions à ta carte OpenRB si besoin.
// - DEBUG_SERIAL : lien USB-C vers le PC (là où arrivent les lignes Python)
// - DXL_SERIAL   : bus Dynamixel (ports 3-pin)
#define DEBUG_SERIAL Serial
#define DXL_SERIAL   Serial1

// IDs des moteurs (à adapter à TON robot)
const uint8_t ID_M1    = 1;
const uint8_t ID_M2    = 2;
const uint8_t ID_M3    = 3;
const uint8_t ID_PINCE = 4;   // ou 0/255 si la pince n'est pas un Dynamixel

// Exemple pour Dynamixel 12 bits (4095 ticks par tour).
// Adapte à ton modèle (voir datasheet).
const float TICKS_PER_REV = 4095.0f;

// ================================
// Vitesse des moteurs (Profile Velocity)
// ================================
// 0 = vitesse max (DANGEREUX pour les premiers tests !)
// 30  = très lent  (idéal pour débuter)
// 100 = modéré
// 300 = rapide
const uint32_t PROFILE_VELOCITY = 30;

Dynamixel2Arduino dxl(DXL_SERIAL);

// ================================
// Buffer & état courant
// ================================

const size_t LINE_BUF_SIZE = 64;
char   lineBuf[LINE_BUF_SIZE];
size_t linePos = 0;

// Dernière commande reçue
float lastTheta1 = 0.0f;
float lastTheta2 = 0.0f;
float lastTheta3 = 0.0f;
bool  lastPince  = false;
bool  newCommand = false;

// ================================
// Initialisation
// ================================

void setup() {
  // USB vers PC (reçoit les lignes envoyées par PieToArduino.py)
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) {
    ; // attendre l'ouverture du port
  }

  // Bus Dynamixel
  DXL_SERIAL.begin(57600);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  // Met tous les moteurs en mode Position
  dxl.torqueOff(ID_M1);
  dxl.torqueOff(ID_M2);
  dxl.torqueOff(ID_M3);
  dxl.torqueOff(ID_PINCE);

  dxl.setOperatingMode(ID_M1, OP_POSITION);
  dxl.setOperatingMode(ID_M2, OP_POSITION);
  dxl.setOperatingMode(ID_M3, OP_POSITION);
  dxl.setOperatingMode(ID_PINCE, OP_POSITION); // si la pince est aussi un Dynamixel

  // Limiter la vitesse de déplacement (Profile Velocity)
  // Adresse 112 = Profile Velocity dans la table de contrôle Dynamixel
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, ID_M1, PROFILE_VELOCITY);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, ID_M2, PROFILE_VELOCITY);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, ID_M3, PROFILE_VELOCITY);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, ID_PINCE, PROFILE_VELOCITY);

  dxl.torqueOn(ID_M1);
  dxl.torqueOn(ID_M2);
  dxl.torqueOn(ID_M3);
  dxl.torqueOn(ID_PINCE);

  DEBUG_SERIAL.print("OpenRB Delta Robot ready | Profile Velocity = ");
  DEBUG_SERIAL.println(PROFILE_VELOCITY);
}

// ================================
// Boucle principale
// ================================

void loop() {
  // 1) Lire les lignes arrivant depuis le PC (USB)
  readSerialLines();

  // 2) Appliquer uniquement si une nouvelle commande a été reçue
  if (newCommand) {
    applyLastCommand();
    newCommand = false;
  }

  // Fréquence de contrôle ~200 Hz
  delay(5);
}

// ================================
// Lecture série & parsing
// ================================

// Accumule les caractères reçus sur DEBUG_SERIAL
// et appelle parseCommandLine() à chaque fin de ligne '\n'.
void readSerialLines() {
  while (DEBUG_SERIAL.available() > 0) {
    char c = DEBUG_SERIAL.read();

    if (c == '\r') {
      // On ignore les retours chariot éventuels
      continue;
    }

    if (c == '\n') {
      // Fin de ligne : traiter le buffer courant
      lineBuf[linePos] = '\0';
      parseCommandLine(lineBuf);
      linePos = 0;  // reset pour la prochaine ligne
    } else {
      if (linePos < LINE_BUF_SIZE - 1) {
        lineBuf[linePos++] = c;
      } else {
        // Ligne trop longue : on reset pour éviter les débordements
        linePos = 0;
      }
    }
  }
}

// Parse une ligne du type : "theta1,theta2,theta3,pince"
// où les thetas sont envoyés par Python en RADIANS,
// et pince vaut 0 (ouvert) ou 1 (fermé).
void parseCommandLine(const char* line) {
  char buf[LINE_BUF_SIZE];
  strncpy(buf, line, LINE_BUF_SIZE);
  buf[LINE_BUF_SIZE - 1] = '\0';

  float vals[4];
  int   count = 0;

  char* token = strtok(buf, ",");
  while (token != NULL && count < 4) {
    vals[count] = atof(token);
    count++;
    token = strtok(NULL, ",");
  }

  if (count < 4) {
    DEBUG_SERIAL.print("Ligne invalide (attendu 4 valeurs) : ");
    DEBUG_SERIAL.println(line);
    return;
  }

  // Angles en radians reçus depuis Python
  lastTheta1 = vals[0];
  lastTheta2 = vals[1];
  lastTheta3 = vals[2];
  lastPince  = (vals[3] >= 0.5f);
  newCommand = true;

  // Debug
  DEBUG_SERIAL.print("Cmd: ");
  DEBUG_SERIAL.print(lastTheta1); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.print(lastTheta2); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.print(lastTheta3); DEBUG_SERIAL.print(", pince=");
  DEBUG_SERIAL.println(lastPince ? "1" : "0");
}

// ================================
// Application de la commande
// ================================

void applyLastCommand() {
  // Conversion radians -> degrés
  float theta1_deg = lastTheta1 * 180.0f / PI;
  float theta2_deg = lastTheta2 * 180.0f / PI;
  float theta3_deg = lastTheta3 * 180.0f / PI;

  // Conversion degrés -> ticks (0..TICKS_PER_REV)
  int32_t pos1 = (int32_t)(theta1_deg / 360.0f * TICKS_PER_REV);
  int32_t pos2 = (int32_t)(theta2_deg / 360.0f * TICKS_PER_REV);
  int32_t pos3 = (int32_t)(theta3_deg / 360.0f * TICKS_PER_REV);

  dxl.setGoalPosition(ID_M1, pos1);
  dxl.setGoalPosition(ID_M2, pos2);
  dxl.setGoalPosition(ID_M3, pos3);

  // Pince : exemple simple avec deux positions (adapter aux limites réelles)
  if (lastPince) {
    dxl.setGoalPosition(ID_PINCE, 3000);  // Fermé
  } else {
    dxl.setGoalPosition(ID_PINCE, 1500);  // Ouvert
  }
}

