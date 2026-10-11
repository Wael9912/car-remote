// Remote-controlled car: takes one-letter commands over USB serial or Bluetooth
// and reports the ultrasonic distance back on both.
//
// Commands:  F forward  B backward  L turn left  R turn right  S stop
//            0..9 speed level        A auto (obstacle avoid)    M manual
// Reports:   "D <cm>" distance (999 = nothing in range), "MODE auto|manual", "READY"
//
// Safety: in manual mode the car stops if no command arrives for 400 ms
// (the browser repeats the command while a button is held), and it refuses
// to drive forward when something is closer than GUARD_CM.

#include <SoftwareSerial.h>

const int ENA = 3;           // left speed (PWM)
const int IN1 = 7, IN2 = 6;  // left direction
const int ENB = 11;          // right speed (PWM)
const int IN3 = 5, IN4 = 4;  // right direction
const int TRIG = A1, ECHO = A2;
const int BT_RX = 8, BT_TX = 9;  // to Bluetooth TXD, RXD

SoftwareSerial bt(BT_RX, BT_TX);

const int GUARD_CM = 15;     // manual: block forward when closer than this
const int AVOID_CM = 25;     // auto: start avoiding when closer than this
const unsigned long COMMAND_TIMEOUT_MS = 400;

int speed = 80;              // 0..255
bool autoMode = false;
char current = 'S';
unsigned long lastCommandAt = 0;
long distance = 999;
unsigned long lastPingAt = 0;
unsigned long lastReportAt = 0;

// auto mode steps
enum AutoStep { CRUISE, PAUSE, BACKUP, TURN };
AutoStep step = CRUISE;
unsigned long stepUntil = 0;

// dir: 1 = forward, -1 = backward, 0 = stop
void left(int dir, int pwm) {
  // left forward is IN1 LOW / IN2 HIGH
  digitalWrite(IN1, dir < 0); digitalWrite(IN2, dir > 0);
  analogWrite(ENA, dir ? pwm : 0);
}

void right(int dir, int pwm) {
  // right forward is IN3 LOW / IN4 HIGH
  digitalWrite(IN3, dir < 0); digitalWrite(IN4, dir > 0);
  analogWrite(ENB, dir ? pwm : 0);
}

// turning in place on the floor needs more push than driving straight
int turnSpeed() { return min(255, speed + 50); }

void drive(char cmd) {
  switch (cmd) {
    case 'F': left(1, speed);  right(1, speed);  break;
    case 'B': left(-1, speed); right(-1, speed); break;
    case 'L': left(-1, turnSpeed()); right(1, turnSpeed());  break;
    case 'R': left(1, turnSpeed());  right(-1, turnSpeed()); break;
    default:  left(0, 0); right(0, 0);
  }
}

long readDistanceCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 25000);
  return us == 0 ? 999 : us / 58;  // no echo = nothing in range
}

// send a line to both USB and Bluetooth
void say(const String &line) {
  Serial.println(line);
  bt.println(line);
}

void setMode(bool on) {
  autoMode = on;
  step = CRUISE;
  current = 'S';
  drive('S');
  say(on ? "MODE auto" : "MODE manual");
}

void handleCommand(char c) {
  if (c >= '0' && c <= '9') {
    speed = map(c - '0', 0, 9, 55, 255);
    if (!autoMode) drive(current);
    return;
  }
  switch (c) {
    case 'A': setMode(true);  return;
    case 'M': setMode(false); return;
    case 'S':
      if (autoMode) setMode(false);  // stop always wins
      current = 'S'; drive('S'); lastCommandAt = millis();
      return;
    case 'F': case 'B': case 'L': case 'R':
      if (autoMode) return;
      current = c; lastCommandAt = millis();
      if (c == 'F' && distance < GUARD_CM) current = 'S';
      drive(current);
      return;
  }
}

void runAuto(unsigned long now) {
  switch (step) {
    case CRUISE:
      if (distance < AVOID_CM) { drive('S'); step = PAUSE; stepUntil = now + 300; }
      else drive('F');
      break;
    case PAUSE:
      if (now >= stepUntil) { drive('B'); step = BACKUP; stepUntil = now + 400; }
      break;
    case BACKUP:
      if (now >= stepUntil) { drive('R'); step = TURN; stepUntil = now + 500; }
      break;
    case TURN:
      if (now >= stepUntil) { drive('S'); step = CRUISE; }
      break;
  }
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  drive('S');
  Serial.begin(9600);
  bt.begin(9600);
  say("READY");
  say("MODE manual");
}

void loop() {
  while (Serial.available()) handleCommand(Serial.read());
  while (bt.available()) handleCommand(bt.read());

  unsigned long now = millis();

  if (now - lastPingAt >= 100) {
    lastPingAt = now;
    distance = readDistanceCm();
  }

  // Report less often than we measure: SoftwareSerial can't receive while it sends.
  if (now - lastReportAt >= 200) {
    lastReportAt = now;
    say("D " + String(distance));
  }

  if (autoMode) {
    runAuto(now);
  } else if (current != 'S') {
    bool timedOut = now - lastCommandAt > COMMAND_TIMEOUT_MS;
    bool blocked = current == 'F' && distance < GUARD_CM;
    if (timedOut || blocked) { current = 'S'; drive('S'); }
  }
}
