#include <Arduino.h>
#include <ESP32Servo.h>

// =====================================================
// THRUSTER PINS
// =====================================================
const int PIN_LEFT_THRUSTER  = 12;
const int PIN_RIGHT_THRUSTER = 13;

Servo thrLeft;
Servo thrRight;

// ESC PWM settings
const int PWM_MIN = 1100;
const int PWM_MID = 1500;
const int PWM_MAX = 1900;
const int PWM_RANGE = 400;

// =====================================================
// SONAR INPUT PINS
// Replace these with your real sonar pins / interfaces
// =====================================================
const int PIN_FRONT_SONAR = 34;
const int PIN_LEFT_SONAR  = 35;
const int PIN_RIGHT_SONAR = 32;

// =====================================================
// SERIAL / CONTROL SETTINGS
// =====================================================
float cmd_v = 0.0f;
float cmd_w = 0.0f;

unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT_MS = 500;   // stop if no command from PC

// =====================================================
// FILTER SETTINGS
// =====================================================
const int FILTER_N = 5;
float frontBuf[FILTER_N];
float leftBuf[FILTER_N];
float rightBuf[FILTER_N];
int filterIdx = 0;

// =====================================================
// UTILS
// =====================================================
float clampf(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

int cmdToPwm(float cmd) {
  cmd = clampf(cmd, -1.0f, 1.0f);
  int pwm = PWM_MID + (int)(cmd * PWM_RANGE);
  if (pwm < PWM_MIN) pwm = PWM_MIN;
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  return pwm;
}

void stopThrusters() {
  thrLeft.writeMicroseconds(PWM_MID);
  thrRight.writeMicroseconds(PWM_MID);
}

void sendMotion(float v, float w) {
  // v: forward speed [0..1]
  // w: steering [-1..1]
  v = clampf(v, 0.0f, 1.0f);
  w = clampf(w, -1.0f, 1.0f);

  float leftCmd  = clampf(v - w, -1.0f, 1.0f);
  float rightCmd = clampf(v + w, -1.0f, 1.0f);

  thrLeft.writeMicroseconds(cmdToPwm(leftCmd));
  thrRight.writeMicroseconds(cmdToPwm(rightCmd));
}

// =====================================================
// FAKE ANALOG-TO-METER CONVERSION
// Replace this with your real sonar conversion
// =====================================================
float adcToMeters(int adc) {
  float x = (float)adc / 4095.0f;
  float meters = 0.2f + x * (4.0f - 0.2f);  // maps roughly 0.2m -> 4.0m
  return clampf(meters, 0.2f, 4.0f);
}

float readFrontSonarRaw() {
  return adcToMeters(analogRead(PIN_FRONT_SONAR));
}

float readLeftSonarRaw() {
  return adcToMeters(analogRead(PIN_LEFT_SONAR));
}

float readRightSonarRaw() {
  return adcToMeters(analogRead(PIN_RIGHT_SONAR));
}

// =====================================================
// MOVING AVERAGE FILTER
// =====================================================
void initFilter(float initVal = 2.0f) {
  for (int i = 0; i < FILTER_N; i++) {
    frontBuf[i] = initVal;
    leftBuf[i]  = initVal;
    rightBuf[i] = initVal;
  }
}

void updateFilter(float f, float l, float r, float &fOut, float &lOut, float &rOut) {
  filterIdx = (filterIdx + 1) % FILTER_N;

  frontBuf[filterIdx] = f;
  leftBuf[filterIdx]  = l;
  rightBuf[filterIdx] = r;

  float fs = 0, ls = 0, rs = 0;
  for (int i = 0; i < FILTER_N; i++) {
    fs += frontBuf[i];
    ls += leftBuf[i];
    rs += rightBuf[i];
  }

  fOut = fs / FILTER_N;
  lOut = ls / FILTER_N;
  rOut = rs / FILTER_N;
}

// =====================================================
// SERIAL RECEIVE FROM PC
// Expected format: v,w
// Example: 0.25,-0.40
// =====================================================
void readControlCommand() {
  static String buffer = "";

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n') {
      int commaPos = buffer.indexOf(',');
      if (commaPos > 0) {
        String vStr = buffer.substring(0, commaPos);
        String wStr = buffer.substring(commaPos + 1);

        float v = vStr.toFloat();
        float w = wStr.toFloat();

        cmd_v = clampf(v, 0.0f, 1.0f);
        cmd_w = clampf(w, -1.0f, 1.0f);
        lastCmdTime = millis();
      }
      buffer = "";
    } else if (c != '\r') {
      buffer += c;
      if (buffer.length() > 50) buffer = "";
    }
  }
}

// =====================================================
// SEND SENSOR DATA TO PC
// Format: df,dl,dr
// Example: 1.24,0.88,1.75
// =====================================================
void sendSensorData(float df, float dl, float dr) {
  Serial.print(df, 3);
  Serial.print(",");
  Serial.print(dl, 3);
  Serial.print(",");
  Serial.println(dr, 3);
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  thrLeft.attach(PIN_LEFT_THRUSTER, PWM_MIN, PWM_MAX);
  thrRight.attach(PIN_RIGHT_THRUSTER, PWM_MIN, PWM_MAX);

  stopThrusters();
  delay(3000);  // ESC arming

  initFilter(2.0f);
  lastCmdTime = millis();
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  static unsigned long lastLoop = 0;
  if (millis() - lastLoop < 100) return;   // 10 Hz
  lastLoop = millis();

  // Read sonar
  float fRaw = readFrontSonarRaw();
  float lRaw = readLeftSonarRaw();
  float rRaw = readRightSonarRaw();

  float df, dl, dr;
  updateFilter(fRaw, lRaw, rRaw, df, dl, dr);

  // Send sensor data to PC
  sendSensorData(df, dl, dr);

  // Receive command from PC
  readControlCommand();

  // Failsafe
  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    stopThrusters();
    return;
  }

  // Apply command
  sendMotion(cmd_v, cmd_w);
}