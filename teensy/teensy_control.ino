#include <Arduino.h>
#include <Servo.h>

// ======= Hardware / tuning defaults =======
const int STEERING_PIN = 5;
const int THROTTLE_PIN = 6;
const int BAUD_RATE = 115200;

const int STEER_CENTER_DEG = 90;
const int STEER_MIN_DEG = 60;
const int STEER_MAX_DEG = 120;

const int ESC_NEUTRAL_US = 1500;
const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;

const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long ARMING_NEUTRAL_MS = 2000;
// ==========================================

Servo steeringServo;
Servo throttleEsc;

String lineBuffer;
unsigned long lastCommandTime = 0;
unsigned long bootTime = 0;
bool armed = false;

int currentSteer = STEER_CENTER_DEG;
int currentThrottleUs = ESC_NEUTRAL_US;

void applySafeStop() {
  currentThrottleUs = ESC_NEUTRAL_US;
  throttleEsc.writeMicroseconds(currentThrottleUs);
}

void applySteering(int steerDeg) {
  currentSteer = constrain(steerDeg, STEER_MIN_DEG, STEER_MAX_DEG);
  steeringServo.write(currentSteer);
}

void applyThrottle(int throttleUs) {
  currentThrottleUs = constrain(throttleUs, ESC_MIN_US, ESC_MAX_US);
  throttleEsc.writeMicroseconds(currentThrottleUs);
}

void processCommand(const String &cmdRaw) {
  String cmd = cmdRaw;
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "STOP") {
    applySafeStop();
    lastCommandTime = millis();
    return;
  }

  if (cmd == "ARM") {
    armed = true;
    applySafeStop();
    lastCommandTime = millis();
    return;
  }

  if (cmd == "DISARM") {
    armed = false;
    applySafeStop();
    lastCommandTime = millis();
    return;
  }

  if (cmd.startsWith("STEER:")) {
    int value = cmd.substring(6).toInt();
    applySteering(value);
    lastCommandTime = millis();
    return;
  }

  if (cmd.startsWith("THROTTLE:")) {
    int value = cmd.substring(9).toInt();
    if (armed) {
      applyThrottle(value);
    } else {
      applySafeStop();
    }
    lastCommandTime = millis();
    return;
  }
}

void setup() {
  Serial.begin(BAUD_RATE);

  steeringServo.attach(STEERING_PIN);
  throttleEsc.attach(THROTTLE_PIN);

  applySteering(STEER_CENTER_DEG);
  applySafeStop();

  lineBuffer.reserve(64);
  lastCommandTime = millis();
  bootTime = millis();
}

void loop() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      processCommand(lineBuffer);
      lineBuffer = "";
    } else if (c != '\r') {
      lineBuffer += c;
      if (lineBuffer.length() > 63) {
        lineBuffer = "";
      }
    }
  }

  if (millis() - bootTime < ARMING_NEUTRAL_MS) {
    applySafeStop();
    return;
  }

  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    armed = false;
    applySafeStop();
  }
}
