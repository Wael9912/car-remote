// Self-driving: go forward; if something is closer than STOP_CM, back up and turn right.
// Waits for a start signal: hand in front of the sensor, then away.

const int ENA = 3;           // left speed (PWM)
const int IN1 = 7, IN2 = 6;  // left direction
const int ENB = 11;          // right speed (PWM)
const int IN3 = 5, IN4 = 4;  // right direction
const int TRIG = A1, ECHO = A2;

const int DRIVE_SPEED = 80;  // 0..255, forward/backward
const int TURN_SPEED = 130;  // turning in place needs more push on the floor
const int STOP_CM = 25;

// dir: 1 = forward, -1 = backward, 0 = stop
void left(int dir, int speed) {
  // left forward is IN1 LOW / IN2 HIGH
  digitalWrite(IN1, dir < 0); digitalWrite(IN2, dir > 0);
  analogWrite(ENA, dir ? speed : 0);
}

void right(int dir, int speed) {
  // right forward is IN3 LOW / IN4 HIGH
  digitalWrite(IN3, dir < 0); digitalWrite(IN4, dir > 0);
  analogWrite(ENB, dir ? speed : 0);
}

void stopAll() { left(0, 0); right(0, 0); }

long distanceCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 30000);
  return us == 0 ? 999 : us / 58;  // no echo = nothing in range
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  stopAll();

  // Blink while waiting for a hand within 10 cm, then wait for it to leave.
  while (distanceCm() > 10) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(150);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  while (distanceCm() <= 10) delay(100);
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (distanceCm() < STOP_CM) {
    stopAll();                                      delay(300);
    left(-1, DRIVE_SPEED); right(-1, DRIVE_SPEED);  delay(400);
    left(1, TURN_SPEED);   right(-1, TURN_SPEED);   delay(500);
    stopAll();                                      delay(200);
  } else {
    left(1, DRIVE_SPEED); right(1, DRIVE_SPEED);
  }
  delay(50);
}
