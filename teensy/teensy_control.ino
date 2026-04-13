#include <Arduino.h>
#include <Servo.h>

// ======= Hardware / tuning defaults =======
const int STEERING_PIN = 5;
const int THROTTLE_PIN = 6;
const int ENCODER_PIN = 23;
const int BAUD_RATE = 115200;

const int STEER_CENTER_DEG = 90;
const int STEER_MIN_DEG = 60;
const int STEER_MAX_DEG = 120;

const int ESC_NEUTRAL_US = 1500;
const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;
const int ESC_FORWARD_BASE_US = 1520;
const int ESC_FORWARD_LIMIT_US = 1725;

const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long ARMING_NEUTRAL_MS = 2000;
const unsigned long TELEMETRY_PERIOD_MS = 200;

const float WHEEL_DIAMETER_INCHES = 4.2f;
const float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_INCHES * 0.0254f * PI;
const float METERS_PER_ROTATION = (2.0f * PI * (WHEEL_DIAMETER_INCHES / 2.0f)) * 0.0254f;
const float KP_SPEED = 35.0f;
const float MIN_TARGET_SPEED_MPS = 0.4f;
// ==========================================

Servo steeringServo;
Servo throttleEsc;

String lineBuffer;
unsigned long lastCommandTime = 0;
unsigned long bootTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastSpeedSampleMs = 0;
unsigned long workoutStartMs = 0;
volatile long encoderTicks = 0;
long lastSpeedSampleCount = 0;
bool armed = false;
bool workoutActive = false;

int currentSteer = STEER_CENTER_DEG;
int currentThrottleUs = ESC_NEUTRAL_US;
float targetDistanceM = 100.0f;
float targetTimeS = 25.0f;
float targetSpeedMps = 4.0f;
float targetRotations = 0.0f;
float measuredSpeedMps = 0.0f;

void encoderISR() {
  encoderTicks++;
}

float metersForCounts(long counts) {
  return static_cast<float>(counts) * METERS_PER_ROTATION;
}

void applySafeStop() {
  currentThrottleUs = ESC_NEUTRAL_US;
  throttleEsc.writeMicroseconds(currentThrottleUs);
  workoutActive = false;
}

void applySteering(int steerDeg) {
  currentSteer = constrain(steerDeg, STEER_MIN_DEG, STEER_MAX_DEG);
  steeringServo.write(currentSteer);
}

void applyThrottle(int throttleUs) {
  currentThrottleUs = constrain(throttleUs, ESC_MIN_US, ESC_MAX_US);
  throttleEsc.writeMicroseconds(currentThrottleUs);
}

void resetWorkoutTracking() {
  noInterrupts();
  encoderTicks = 0;
  interrupts();
  lastSpeedSampleCount = 0;
  lastSpeedSampleMs = millis();
  measuredSpeedMps = 0.0f;
}

void configureWorkout(float distanceM, float timeS, float speedMps, float rotations) {
  targetDistanceM = distanceM;
  targetTimeS = timeS;
  targetSpeedMps = speedMps;
  targetRotations = rotations;
}

void processWorkoutCommand(const String &payload, bool startNow) {
  int firstComma = payload.indexOf(',');
  int secondComma = payload.indexOf(',', firstComma + 1);
  int thirdComma = payload.indexOf(',', secondComma + 1);
  if (firstComma < 0 || secondComma < 0 || thirdComma < 0) {
    return;
  }

  float distanceM = payload.substring(0, firstComma).toFloat();
  float timeS = payload.substring(firstComma + 1, secondComma).toFloat();
  float speedMps = payload.substring(secondComma + 1, thirdComma).toFloat();
  float rotations = payload.substring(thirdComma + 1).toFloat();
  configureWorkout(distanceM, timeS, speedMps, rotations);

  if (startNow) {
    resetWorkoutTracking();
    workoutStartMs = millis();
    workoutActive = armed;
  }
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

  if (cmd.startsWith("WORKOUTCFG:")) {
    processWorkoutCommand(cmd.substring(11), false);
    lastCommandTime = millis();
    return;
  }

  if (cmd.startsWith("WORKOUT:")) {
    processWorkoutCommand(cmd.substring(8), true);
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

void updateWorkoutControl() {
  unsigned long now = millis();
  long tickCount;
  noInterrupts();
  tickCount = encoderTicks;
  interrupts();

  float elapsedS = (now - lastSpeedSampleMs) / 1000.0f;
  if (elapsedS > 0.0f) {
    long deltaCount = tickCount - lastSpeedSampleCount;
    measuredSpeedMps = metersForCounts(deltaCount) / elapsedS;
    lastSpeedSampleCount = tickCount;
    lastSpeedSampleMs = now;
  }

  float traveledM = metersForCounts(tickCount);
  if (workoutActive && static_cast<float>(tickCount) >= targetRotations) {
    applySafeStop();
  }

  if (workoutActive && armed) {
    float elapsedWorkoutS = static_cast<float>(now - workoutStartMs) / 1000.0f;
    float remainingDistanceM = max(0.0f, targetDistanceM - traveledM);
    float remainingTimeS = max(0.01f, targetTimeS - elapsedWorkoutS);
    float dynamicTargetSpeedMps = max(MIN_TARGET_SPEED_MPS, remainingDistanceM / remainingTimeS);

    float speedError = dynamicTargetSpeedMps - measuredSpeedMps;
    int commandedThrottle = ESC_FORWARD_BASE_US + static_cast<int>(speedError * KP_SPEED);
    commandedThrottle = constrain(commandedThrottle, ESC_FORWARD_BASE_US, ESC_FORWARD_LIMIT_US);
    applyThrottle(commandedThrottle);
  }
}

void emitTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetryTime < TELEMETRY_PERIOD_MS) {
    return;
  }
  lastTelemetryTime = now;

  long tickCount;
  noInterrupts();
  tickCount = encoderTicks;
  interrupts();

  float traveledM = metersForCounts(tickCount);
  int done = (!workoutActive && traveledM >= targetDistanceM) ? 1 : 0;

  Serial.print("TELEMETRY:count=");
  Serial.print(tickCount);
  Serial.print(",meters=");
  Serial.print(traveledM, 3);
  Serial.print(",mps=");
  Serial.print(measuredSpeedMps, 3);
  Serial.print(",done=");
  Serial.println(done);
}

void setup() {
  Serial.begin(BAUD_RATE);

  steeringServo.attach(STEERING_PIN);
  throttleEsc.attach(THROTTLE_PIN);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  applySteering(STEER_CENTER_DEG);
  applySafeStop();
  resetWorkoutTracking();

  lineBuffer.reserve(64);
  lastCommandTime = millis();
  bootTime = millis();
  lastTelemetryTime = millis();
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
    emitTelemetry();
    return;
  }

  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    armed = false;
    applySafeStop();
  }

  updateWorkoutControl();
  emitTelemetry();
}
