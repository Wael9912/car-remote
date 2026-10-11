// Remote-controlled car: takes commands over USB serial or Bluetooth (BLE module on pins 8/9)
// and reports the ultrasonic distance back on both.
//
// Commands:  J<x>,<y>,<x+y>\n   joystick, x = turn, y = forward, each -100..100
//                               (the sum is a check: SoftwareSerial can drop bytes)
//            S stop   0..9 max speed   A auto (obstacle avoid)   M manual
// Reports:   "D <cm>" distance (999 = nothing in range), "MODE auto|manual", "READY"
//
// Safety: in manual mode the car stops if no command arrives for 400 ms
// (the remote repeats the joystick position while it is held), and it won't
// move forward when something is closer than GUARD_CM (turning away still works).

#include <SoftwareSerial.h>

const int ENA = 3;           // left speed (PWM)
const int IN1 = 7, IN2 = 6;  // left direction
const int ENB = 11;          // right speed (PWM)
const int IN3 = 5, IN4 = 4;  // right direction
const int TRIG = A1, ECHO = A2;
const int BT_RX = 8, BT_TX = 9;  // to Bluetooth TXD, RXD

const int GUARD_CM = 15;     // manual: block forward when closer than this
const int AVOID_CM = 25;     // auto: start avoiding when closer than this
const unsigned long COMMAND_TIMEOUT_MS = 400;

const int MIN_DRIVE_PWM = 60;   // below this the motors just hum
const int MIN_SPIN_PWM = 110;   // turning in place drags the tyres, needs more push
const int AUTO_MAX_PWM = 140;   // self-driving stays gentle whatever the slider says
const int ACCEL_PER_MS = 1;     // 0 -> 255 in about a quarter second
const int DECEL_PER_MS = 3;     // slowing down is quicker than speeding up

SoftwareSerial bt(BT_RX, BT_TX);

int maxSpeed = 150;          // PWM for a full joystick push, set by 0..9
bool autoMode = false;
unsigned long lastCommandAt = 0;
long distance = 999;
unsigned long lastPingAt = 0, lastReportAt = 0, lastRampAt = 0;

int joyX = 0, joyY = 0;      // last joystick position, -100..100
int targetL = 0, targetR = 0;  // signed PWM the motors are heading to
int curL = 0, curR = 0;        // signed PWM the motors have now

// joystick message being received, per source
struct LineIn { bool active = false; char buf[16]; byte len = 0; };
LineIn usbLine, btLine;

// auto mode steps
enum AutoStep { CRUISE, PAUSE, BACKUP, TURN };
AutoStep step = CRUISE;
unsigned long stepUntil = 0;

// ---- motors ----

void left(int pwm) {
  // left forward is IN1 LOW / IN2 HIGH
  digitalWrite(IN1, pwm < 0); digitalWrite(IN2, pwm > 0);
  analogWrite(ENA, abs(pwm));
}

void right(int pwm) {
  // right forward is IN3 LOW / IN4 HIGH
  digitalWrite(IN3, pwm < 0); digitalWrite(IN4, pwm > 0);
  analogWrite(ENB, abs(pwm));
}

void stopNow() {
  joyX = joyY = 0;
  targetL = targetR = curL = curR = 0;
  left(0); right(0);
}

// Move cur toward target, faster when slowing down. Starting from rest jumps
// straight to the minimum useful PWM so the motors don't hum on the way up.
int approach(int cur, int target, int dt) {
  if (cur == target) return cur;
  bool speedingUp = (target > cur && cur >= 0) || (target < cur && cur <= 0);
  int stepSize = max(1, dt * (speedingUp ? ACCEL_PER_MS : DECEL_PER_MS));
  if (speedingUp && abs(cur) < MIN_DRIVE_PWM) {
    int floorPwm = min(abs(target), MIN_DRIVE_PWM);
    cur = target > 0 ? max(cur, floorPwm) : min(cur, -floorPwm);
  }
  return target > cur ? min(target, cur + stepSize) : max(target, cur - stepSize);
}

// -100..100 stick value -> signed PWM, skipping the dead zone and the humming range
int toPwm(int v, int minPwm, int topPwm) {
  if (abs(v) < 5) return 0;
  topPwm = max(topPwm, minPwm);
  int pwm = map(abs(v), 5, 100, minPwm, topPwm);
  return v > 0 ? pwm : -pwm;
}

// Arcade mixing: y drives both sides, x pushes them apart to turn.
void setTargets(int x, int y, int topPwm) {
  int l = y + x, r = y - x;
  int biggest = max(abs(l), abs(r));
  if (biggest > 100) { l = l * 100L / biggest; r = r * 100L / biggest; }
  bool spinning = (l > 0 && r < 0) || (l < 0 && r > 0);
  int minPwm = spinning ? MIN_SPIN_PWM : MIN_DRIVE_PWM;
  targetL = toPwm(l, minPwm, topPwm);
  targetR = toPwm(r, minPwm, topPwm);
}

void applyJoystick() {
  int y = joyY;
  if (y > 0 && distance < GUARD_CM) y = 0;  // no forward into an obstacle
  setTargets(joyX, y, maxSpeed);
}

// ---- sensor ----

long readDistanceCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 25000);
  return us == 0 ? 999 : us / 58;  // no echo = nothing in range
}

// ---- commands ----

// send a line to both USB and Bluetooth
void say(const String &line) {
  Serial.println(line);
  bt.println(line);
}

void setMode(bool on) {
  autoMode = on;
  step = CRUISE;
  stopNow();
  say(on ? "MODE auto" : "MODE manual");
}

void handleJoystick(const char *msg) {
  int x, y, sum;
  if (sscanf(msg, "%d,%d,%d", &x, &y, &sum) != 3) return;
  if (sum != x + y || abs(x) > 100 || abs(y) > 100) return;  // garbled
  if (autoMode) return;
  joyX = x; joyY = y;
  lastCommandAt = millis();
  applyJoystick();
}

void handleCommand(char c) {
  if (c >= '0' && c <= '9') {
    maxSpeed = map(c - '0', 0, 9, 80, 255);
    if (!autoMode) applyJoystick();
    return;
  }
  switch (c) {
    case 'A': setMode(true);  return;
    case 'M': setMode(false); return;
    case 'S':
      if (autoMode) setMode(false);  // stop always wins
      else stopNow();
      return;
  }
}

// Joystick messages span several bytes; everything else is one letter.
// A single-letter command always cuts a half-received joystick message short,
// so a lost newline can never swallow a stop.
void feed(LineIn &in, char c) {
  if (c == 'J') { in.active = true; in.len = 0; return; }
  if (in.active) {
    if (c == '\n' || c == '\r') {
      in.buf[in.len] = 0; in.active = false;
      handleJoystick(in.buf);
      return;
    }
    if ((c >= '0' && c <= '9') || c == '-' || c == ',') {
      if (in.len < sizeof(in.buf) - 1) in.buf[in.len++] = c;
      else in.active = false;
      return;
    }
    in.active = false;
  }
  handleCommand(c);
}

// ---- self-driving ----

void runAuto(unsigned long now) {
  int top = min(maxSpeed, AUTO_MAX_PWM);
  switch (step) {
    case CRUISE:
      if (distance < AVOID_CM) { stopNow(); step = PAUSE; stepUntil = now + 300; }
      else setTargets(0, 100, top);
      break;
    case PAUSE:
      if (now >= stepUntil) { setTargets(0, -100, top); step = BACKUP; stepUntil = now + 400; }
      break;
    case BACKUP:
      if (now >= stepUntil) { setTargets(100, 0, top); step = TURN; stepUntil = now + 500; }
      break;
    case TURN:
      if (now >= stepUntil) { stopNow(); step = CRUISE; }
      break;
  }
}

// ---- main ----

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  stopNow();
  Serial.begin(9600);
  bt.begin(9600);
  say("READY");
  say("MODE manual");
}

void loop() {
  while (Serial.available()) feed(usbLine, Serial.read());
  while (bt.available()) feed(btLine, bt.read());

  unsigned long now = millis();

  if (now - lastPingAt >= 100) {
    lastPingAt = now;
    distance = readDistanceCm();
    if (!autoMode) applyJoystick();  // re-check the forward guard
  }

  // Report less often than we measure: SoftwareSerial can't receive while it sends.
  if (now - lastReportAt >= 200) {
    lastReportAt = now;
    say("D " + String(distance));
  }

  if (autoMode) {
    runAuto(now);
  } else if ((joyX || joyY) && now - lastCommandAt > COMMAND_TIMEOUT_MS) {
    stopNow();  // remote went quiet
  }

  int dt = now - lastRampAt;
  if (dt > 0) {
    lastRampAt = now;
    curL = approach(curL, targetL, dt);
    curR = approach(curR, targetR, dt);
    left(curL); right(curR);
  }
}
