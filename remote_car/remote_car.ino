// Remote-controlled car: takes commands over USB serial or Bluetooth (BLE module on pins 8/9)
// and reports the ultrasonic distance back on both.
//
// Commands:  J<x>,<y>,<x+y>\n   joystick, x = turn, y = forward, each -100..100
//                               (the sum is a check: SoftwareSerial can drop bytes)
//            S stop   0..9 max speed   A auto (obstacle avoid)   M manual
//            G print the trip log (USB only)
// Reports:   "D <cm>" distance (999 = nothing in range), "MODE auto|manual", "READY"
//
// Safety: in manual mode the car stops if no command arrives for 400 ms
// (the remote repeats the joystick position while it is held), and it won't
// move forward when something is closer than GUARD_CM (turning away still works).
//
// Auto mode: crawl forward; when the sensor sees something close twice in a row,
// stop, turn the sensor (servo on pin 10) left and right, then turn toward the
// clearer side. Everything it does is written to EEPROM (the trip log) so a run
// can be read back over USB afterwards with G.

#include <SoftwareSerial.h>
#include <Servo.h>
#include <EEPROM.h>

const int ENA = 3;           // left speed (PWM)
const int IN1 = 7, IN2 = 6;  // left direction
const int ENB = 11;          // right speed (PWM)
const int IN3 = 5, IN4 = 4;  // right direction
const int TRIG = A1, ECHO = A2;
const int BT_RX = 8, BT_TX = 9;  // to Bluetooth TXD, RXD
const int SERVO_PIN = 10;        // turns the ultrasonic sensor

const int GUARD_CM = 15;     // manual: block forward when closer than this
const int AVOID_CM = 30;     // auto: start avoiding when closer than this
const int BACKUP_CM = 15;    // auto: back up first when closer than this
const unsigned long COMMAND_TIMEOUT_MS = 400;

const int MIN_DRIVE_PWM = 60;   // below this the motors just hum
const int MIN_SPIN_PWM = 110;   // turning in place drags the tyres, needs more push
const int AUTO_MAX_PWM = 75;    // self-driving crawls whatever the slider says
const int ACCEL_PER_MS = 1;     // 0 -> 255 in about a quarter second
const int DECEL_PER_MS = 3;     // slowing down is quicker than speeding up

const int LOOK_CENTER = 90;     // servo angles; swap LEFT/RIGHT if the sensor looks the wrong way
const int LOOK_LEFT = 150;
const int LOOK_RIGHT = 30;
const unsigned long LOOK_SETTLE_MS = 450;   // servo travel time for a 60 degree swing
const unsigned long BACKUP_MS = 500;
const unsigned long TURN_MS = 450;          // roughly a quarter turn
const unsigned long UTURN_MS = 900;         // both sides blocked: turn about

SoftwareSerial bt(BT_RX, BT_TX);
Servo look;

int maxSpeed = 150;          // PWM for a full joystick push, set by 0..9
bool autoMode = false;

unsigned long lastCommandAt = 0;
long distance = 999;
unsigned long lastPingAt = 0, lastReportAt = 0, lastRampAt = 0;
unsigned long servoReleaseAt = 0;  // detach after moving so it doesn't jitter

int joyX = 0, joyY = 0;      // last joystick position, -100..100
int targetL = 0, targetR = 0;  // signed PWM the motors are heading to
int curL = 0, curR = 0;        // signed PWM the motors have now

// joystick message being received, per source
struct LineIn { bool active = false; char buf[16]; byte len = 0; };
LineIn usbLine, btLine;

// auto mode steps
enum AutoStep { CRUISE, LOOKING_LEFT, LOOKING_RIGHT, BACKUP, TURN };
AutoStep step = CRUISE;
unsigned long stepUntil = 0;
byte closeCount = 0;         // close readings in a row while cruising
long frontCm = 999, leftCm = 999, rightCm = 999;
bool turnLeft = false, bothBlocked = false;
unsigned long autoStartedAt = 0, lastSampleAt = 0;

// ---- trip log (EEPROM ring buffer) ----
// byte 0 magic, 1 next slot, 2 run number; slots of 8 bytes from LOG_START.
// slot: run, event, tenths of a second since auto started (2 bytes), a, b, c, unused

enum LogEvent : byte { EV_START = 1, EV_CRUISE, EV_BLOCKED, EV_LOOKED, EV_BACKUP, EV_TURN, EV_END };
const byte LOG_MAGIC = 0xA7;
const int LOG_START = 8, LOG_SLOT = 8;
const byte LOG_SLOTS = (1024 - LOG_START) / LOG_SLOT;
byte runNumber = 0;

byte cmByte(long cm) { return cm > 254 ? 255 : (byte)cm; }  // 255 = far / nothing

void logInit() {
  if (EEPROM.read(0) != LOG_MAGIC) {
    for (int i = 0; i < 1024; i++) EEPROM.update(i, 0);
    EEPROM.update(0, LOG_MAGIC);
  }
  runNumber = EEPROM.read(2);
}

void logEvent(byte ev, byte a = 0, byte b = 0, byte c = 0) {
  byte slot = EEPROM.read(1) % LOG_SLOTS;
  unsigned int tenths = (millis() - autoStartedAt) / 100;
  int at = LOG_START + slot * LOG_SLOT;
  EEPROM.update(at, runNumber);
  EEPROM.update(at + 1, ev);
  EEPROM.update(at + 2, tenths & 0xFF);
  EEPROM.update(at + 3, tenths >> 8);
  EEPROM.update(at + 4, a);
  EEPROM.update(at + 5, b);
  EEPROM.update(at + 6, c);
  EEPROM.update(1, (slot + 1) % LOG_SLOTS);
}

void printCm(byte v) {
  if (v == 255) Serial.print(F("far"));
  else { Serial.print(v); Serial.print(F("cm")); }
}

void printLog() {
  Serial.println(F("LOG BEGIN"));
  byte next = EEPROM.read(1) % LOG_SLOTS;
  for (byte i = 0; i < LOG_SLOTS; i++) {
    int at = LOG_START + ((next + i) % LOG_SLOTS) * LOG_SLOT;
    byte ev = EEPROM.read(at + 1);
    if (ev < EV_START || ev > EV_END) continue;  // empty slot
    unsigned int tenths = EEPROM.read(at + 2) | (EEPROM.read(at + 3) << 8);
    byte a = EEPROM.read(at + 4), b = EEPROM.read(at + 5), c = EEPROM.read(at + 6);
    Serial.print(F("run ")); Serial.print(EEPROM.read(at));
    Serial.print(F("  ")); Serial.print(tenths / 10); Serial.print('.'); Serial.print(tenths % 10);
    Serial.print(F("s  "));
    switch (ev) {
      case EV_START:   Serial.print(F("START auto")); break;
      case EV_CRUISE:  Serial.print(F("cruise  front ")); printCm(a);
                       Serial.print(F("  motors L ")); Serial.print(b);
                       Serial.print(F(" R ")); Serial.print(c); break;
      case EV_BLOCKED: Serial.print(F("BLOCKED front ")); printCm(a); break;
      case EV_LOOKED:  Serial.print(F("looked  left ")); printCm(a);
                       Serial.print(F("  right ")); printCm(b); break;
      case EV_BACKUP:  Serial.print(F("BACKUP")); break;
      case EV_TURN:    Serial.print(F("TURN ")); Serial.print(a ? F("left ") : F("right "));
                       Serial.print(b * 10); Serial.print(F("ms")); break;
      case EV_END:     Serial.print(F("END auto")); break;
    }
    Serial.println();
  }
  Serial.println(F("LOG END"));
}

// ---- servo ----

void lookAt(int angle) {
  look.write(angle);
  if (!look.attached()) look.attach(SERVO_PIN);
  servoReleaseAt = millis() + 700;
}

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

// 0 means no usable reading (closer than the sensor's 2 cm minimum is noise)
long readDistanceCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 25000);
  if (us == 0) return 999;  // no echo = nothing in range
  long cm = us / 58;
  return cm < 2 ? 0 : cm;
}

// middle of three readings, so one bad echo doesn't decide which way to turn
long readSteadyCm() {
  long r[3];
  for (int i = 0; i < 3; i++) {
    r[i] = readDistanceCm();
    if (r[i] == 0) r[i] = 999;
    delay(30);
  }
  long lo = min(r[0], min(r[1], r[2])), hi = max(r[0], max(r[1], r[2]));
  return r[0] + r[1] + r[2] - lo - hi;
}

// ---- commands ----

// send a line to both USB and Bluetooth
void say(const String &line) {
  Serial.println(line);
  bt.println(line);
}

void setMode(bool on) {
  if (on && !autoMode) {
    runNumber++;
    EEPROM.update(2, runNumber);
    autoStartedAt = lastSampleAt = millis();
    logEvent(EV_START);
  } else if (!on && autoMode) {
    logEvent(EV_END);
  }
  autoMode = on;
  step = CRUISE;
  closeCount = 0;
  stopNow();
  lookAt(LOOK_CENTER);
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

void handleCommand(char c, bool fromUsb) {
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
    case 'G':
      if (fromUsb) printLog();  // too long for the Bluetooth link
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
  handleCommand(c, &in == &usbLine);
}

// ---- self-driving ----

void startTurn(unsigned long now) {
  unsigned long ms = bothBlocked ? UTURN_MS : TURN_MS;
  setTargets(turnLeft ? -100 : 100, 0, AUTO_MAX_PWM);
  logEvent(EV_TURN, turnLeft, ms / 10);
  step = TURN;
  stepUntil = now + ms;
}

void runAuto(unsigned long now) {
  int top = min(maxSpeed, AUTO_MAX_PWM);
  switch (step) {
    case CRUISE:
      if (closeCount >= 2) {
        frontCm = distance;
        stopNow();
        logEvent(EV_BLOCKED, cmByte(frontCm));
        lookAt(LOOK_LEFT);
        step = LOOKING_LEFT;
        stepUntil = now + LOOK_SETTLE_MS;
      } else {
        setTargets(0, 100, top);
        if (now - lastSampleAt >= 1000) {
          lastSampleAt = now;
          logEvent(EV_CRUISE, cmByte(distance), abs(curL), abs(curR));
        }
      }
      break;
    case LOOKING_LEFT:
      if (now >= stepUntil) {
        leftCm = readSteadyCm();
        lookAt(LOOK_RIGHT);
        step = LOOKING_RIGHT;
        stepUntil = millis() + LOOK_SETTLE_MS * 2;  // swinging all the way across
      }
      break;
    case LOOKING_RIGHT:
      if (now >= stepUntil) {
        rightCm = readSteadyCm();
        lookAt(LOOK_CENTER);
        logEvent(EV_LOOKED, cmByte(leftCm), cmByte(rightCm));
        turnLeft = leftCm > rightCm;
        bothBlocked = max(leftCm, rightCm) < AVOID_CM;
        now = millis();
        if (bothBlocked || frontCm < BACKUP_CM) {
          setTargets(0, -100, top);
          logEvent(EV_BACKUP);
          step = BACKUP;
          stepUntil = now + BACKUP_MS;
        } else {
          startTurn(now);
        }
      }
      break;
    case BACKUP:
      if (now >= stepUntil) startTurn(now);
      break;
    case TURN:
      if (now >= stepUntil) {
        stopNow();
        closeCount = 0;
        lastSampleAt = now;
        step = CRUISE;
      }
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
  logInit();

  // glance left then straight ahead, so you can see the servo works and which way is left
  lookAt(LOOK_CENTER); delay(400);
  lookAt(LOOK_LEFT);   delay(600);
  lookAt(LOOK_CENTER);

  say("READY");
  say("MODE manual");
}

void loop() {
  while (Serial.available()) feed(usbLine, Serial.read());
  while (bt.available()) feed(btLine, bt.read());

  unsigned long now = millis();

  if (now - lastPingAt >= 100) {
    lastPingAt = now;
    long cm = readDistanceCm();
    if (cm != 0) {  // skip noise, keep the last good reading
      distance = cm;
      if (autoMode && step == CRUISE) closeCount = distance < AVOID_CM ? closeCount + 1 : 0;
    }
    if (!autoMode) applyJoystick();  // re-check the forward guard
  }

  // Report less often than we measure: SoftwareSerial can't receive while it sends.
  if (now - lastReportAt >= 200) {
    lastReportAt = now;
    say("D " + String(distance));
  }

  if (servoReleaseAt && now >= servoReleaseAt) {
    servoReleaseAt = 0;
    look.detach();
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
