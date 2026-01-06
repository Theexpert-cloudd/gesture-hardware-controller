#include <Stepper.h>

const int STEPS_PER_REV = 2048;

// Stepper motor pin order (verified for 28BYJ-48 + ULN2003)
Stepper motor(STEPS_PER_REV, 8, 10, 9, 11);

// LED PWM pin
const int LED_PIN = 6;
bool ledEnabled = false;
int ledBrightness = 180; // 0..255

// Stepper state
long currentPos = 0;
int motorRPM = 12; // default speed

long wrapSteps(long x) {
  x %= STEPS_PER_REV;
  if (x < 0) x += STEPS_PER_REV;
  return x;
}

long shortestDelta(long target, long current) {
  long d = target - current;
  if (d > STEPS_PER_REV / 2) d -= STEPS_PER_REV;
  if (d < -STEPS_PER_REV / 2) d += STEPS_PER_REV;
  return d;
}

void applyLed() {
  if (!ledEnabled) {
    analogWrite(LED_PIN, 0);
  } else {
    analogWrite(LED_PIN, constrain(ledBrightness, 0, 255));
  }
}

void applySpeed() {
  // Safe RPM range for 28BYJ-48
  motor.setSpeed(constrain(motorRPM, 3, 18));
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  applyLed();
  applySpeed();
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  // LED toggle
  if (cmd == "LED") {
    ledEnabled = !ledEnabled;
    applyLed();
    return;
  }

  // Brightness command: B:0..255
  if (cmd.startsWith("B:")) {
    int b = cmd.substring(2).toInt();
    ledBrightness = constrain(b, 0, 255);
    applyLed();
    return;
  }

  // Speed command: S:3..18 (RPM)
  if (cmd.startsWith("S:")) {
    int s = cmd.substring(2).toInt();
    motorRPM = constrain(s, 3, 18);
    applySpeed();
    return;
  }

  // Stepper target position (0..2047)
  long target = cmd.toInt();
  target = wrapSteps(target);

  long delta = shortestDelta(target, currentPos);
  if (delta > -2 && delta < 2) return;

  // Limit steps per update for smoother motion
  if (delta > 120) delta = 120;
  if (delta < -120) delta = -120;

  motor.step((int)delta);
  currentPos = wrapSteps(currentPos + delta);
}
