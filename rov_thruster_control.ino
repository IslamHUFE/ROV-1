/*
  ESP32 ROV Thruster Control 
*/

//  Pin  definitions

#define THRUSTER_UP_LEFT     25
#define THRUSTER_UP_RIGHT    26

#define THRUSTER_FRONT_LEFT  27
#define THRUSTER_FRONT_RIGHT 14
#define THRUSTER_BACK_LEFT   12
#define THRUSTER_BACK_RIGHT  13

//  PWM Config

#define PWM_FREQ 50
#define PWM_RESOLUTION 16

#define CH_UP_LEFT     0
#define CH_UP_RIGHT    1
#define CH_FL          2
#define CH_FR          3
#define CH_BL          4
#define CH_BR          5

//  ESC SIGNAL VALUES 
#define ESC_MIN   1000
#define ESC_STOP  1500
#define ESC_MAX   2000

// sEtup 

void setupThruster(int pin, int channel) {
  ledcSetup(channel, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pin, channel);
}

// Convert microseconds to duty cycle
uint32_t usToDuty(int microseconds) {
  return (microseconds * 65535) / 20000;
}

//  make signal safe
int safeSignal(int signal) {
  return constrain(signal, ESC_MIN, ESC_MAX);
}

// Write ESC signal safely
void writeESC(int channel, int microseconds) {
  ledcWrite(channel, usToDuty(safeSignal(microseconds)));
}

//  BASIC CONTROL 

void stopAll() {
  writeESC(CH_UP_LEFT, ESC_STOP);
  writeESC(CH_UP_RIGHT, ESC_STOP);

  writeESC(CH_FL, ESC_STOP);
  writeESC(CH_FR, ESC_STOP);
  writeESC(CH_BL, ESC_STOP);
  writeESC(CH_BR, ESC_STOP);
}

// ** MOVEMENT **

// Vertical
void moveUp(int power) {
  int signal = ESC_STOP + power;
  writeESC(CH_UP_LEFT, signal);
  writeESC(CH_UP_RIGHT, signal);
}

void moveDown(int power) {
  int signal = ESC_STOP - power;
  writeESC(CH_UP_LEFT, signal);
  writeESC(CH_UP_RIGHT, signal);
}

// Forward / Backward
void moveForward(int power) {
  int signal = ESC_STOP + power;

  writeESC(CH_FL, signal);
  writeESC(CH_FR, signal);
  writeESC(CH_BL, signal);
  writeESC(CH_BR, signal);
}

void moveBackward(int power) {
  int signal = ESC_STOP - power;

  writeESC(CH_FL, signal);
  writeESC(CH_FR, signal);
  writeESC(CH_BL, signal);
  writeESC(CH_BR, signal);
}

// Turning (Differential)
void turnLeft(int power) {
  writeESC(CH_FL, ESC_STOP - power);
  writeESC(CH_BL, ESC_STOP - power);

  writeESC(CH_FR, ESC_STOP + power);
  writeESC(CH_BR, ESC_STOP + power);
}

void turnRight(int power) {
  writeESC(CH_FL, ESC_STOP + power);
  writeESC(CH_BL, ESC_STOP + power);

  writeESC(CH_FR, ESC_STOP - power);
  writeESC(CH_BR, ESC_STOP - power);
}

// ##SETUP##

void setup() {

  setupThruster(THRUSTER_UP_LEFT, CH_UP_LEFT);
  setupThruster(THRUSTER_UP_RIGHT, CH_UP_RIGHT);

  setupThruster(THRUSTER_FRONT_LEFT, CH_FL);
  setupThruster(THRUSTER_FRONT_RIGHT, CH_FR);
  setupThruster(THRUSTER_BACK_LEFT, CH_BL);
  setupThruster(THRUSTER_BACK_RIGHT, CH_BR);

  stopAll();

  delay(3000); // wait for ESCs to arm
}

// LOOP (TEST) 

void loop() {

  moveForward(200);
  delay(3000);

  turnLeft(200);
  delay(3000);

  moveUp(200);
  delay(3000);

  stopAll();
  delay(3000);
}
