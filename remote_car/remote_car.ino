// Remote-controlled car: takes commands over USB serial or Bluetooth (BLE module on pins 8/9)
// and reports the ultrasonic distance back on both.
//
// Commands:  J<x>,<y>,<x+y>\n   joystick, x = turn, y = forward, each -100..100
//                               (the sum is a check: SoftwareSerial can drop bytes)
//            S stop   0..9 max speed   A auto (obstacle avoid)   M manual
//            G print the trip log (USB only)   V look test: left, right, ahead (manual only)
//            H go back to the start   P make this spot the start
// Reports:   "D <cm>" distance (999 = nothing in range), "MODE auto|manual|home", "READY",
//            "TRIP <percent of the trip memory used>", "HOME done|too far|already", "PATH FULL"
//
// Safety: in manual mode the car stops if no command arrives for 400 ms
// (the remote repeats the joystick position while it is held), and it won't
// move forward when something is closer than GUARD_CM (turning away still works).
//
// Auto mode: drive forward while the sensor (servo on pin 10) sweeps like a radar
// from 30 to 150 degrees, so the car's corners and sides are covered, not just the
// narrow beam straight ahead. Each reading becomes "this far ahead, this far to the
// side": in the car's lane = stop; next to the lane = nudge away without stopping. Something ahead (or very close at a side, or an echo that
// vanished while closing in on something - angled walls bounce the sound away):
// brake, look left and right, turn toward the clearer side, then check ahead
// before driving on. If the distance stops changing while driving, it's stuck:
// back up and turn round. Everything it does is written to EEPROM (the trip log)
// so a run can be read back over USB afterwards with G.
//
// Going home: there's no compass or wheel counter, so the car records every
// motor command (and how long it lasted) since power-on or P, and H plays the
// list backwards with each command reversed - forward 2 s becomes backward 2 s,
// spin right becomes spin left. Slip on the same floor at the same power is
// much the same both ways, so most of the error cancels, and it retraces a path
// it already drove. The sensor faces forward, so reversing is blind; forward
// stretches stop short of anything under HOME_GUARD_CM.

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

const int GUARD_CM = 15;     // manual: block forward when closer than this when slow...
const int GUARD_FAST_CM = 45;  // ...growing to this at full speed (it needs room to stop)
const int AVOID_CM = 40;     // auto: stop and look when something ahead is closer than this
const int BACKUP_CM = 15;    // auto: back up first when closer than this
const int CLEAR_CM = 50;     // auto: after a turn, ahead must be at least this clear
const int SLOW_CM = 60;      // auto: creep when something ahead is closer than this
const int LANE_HALF_CM = 12;  // auto: half the car's width plus a margin - closer to the middle is in its path
const int NUDGE_LANE_CM = 22;  // auto: something this close to the middle...
const int NUDGE_AHEAD_CM = 35; // ...and this close ahead gets steered away from
const int SUSPECT_CM = 60;   // auto: an echo vanishing while this close is suspicious
const unsigned long COMMAND_TIMEOUT_MS = 400;

const int MIN_DRIVE_PWM = 60;   // below this the motors just hum
const int MIN_SPIN_PWM = 110;   // turning in place drags the tyres, needs more push
const int TURN_PWM_START = 130; // auto turns start here and get stronger if the car doesn't rotate
const int TURN_PWM_STEP = 30, TURN_PWM_MAX = 230;
const int AUTO_MAX_PWM = 75;    // self-driving crawls whatever the slider says
const int AUTO_SLOW_PWM = 65;   // ...and creeps near obstacles
const int ACCEL_PER_MS = 1;     // 0 -> 255 in about a quarter second
const int DECEL_PER_MS = 3;     // slowing down is quicker than speeding up

const int LOOK_CENTER = 90;     // servo angles; swap LEFT/RIGHT if the sensor looks the wrong way
const int LOOK_LEFT = 150;
const int LOOK_RIGHT = 30;
const unsigned long LOOK_SETTLE_MS = 450;   // servo travel time for a 60 degree swing
const unsigned long BACKUP_MS = 500;
const unsigned long TURN_MS = 450;          // roughly a quarter turn
const unsigned long UTURN_MS = 900;         // both sides blocked: turn about
const unsigned long NUDGE_MS = 200;         // small steer away from a side obstacle
const unsigned long BRAKE_MS = 120;         // short the motors to stop instead of rolling on
const unsigned long STUCK_MS = 3000;        // distance ahead unchanged this long = stuck
const int STUCK_CHANGE_CM = 4;
const byte VERIFY_TRIES = 3;                // turns to find a clear way before giving up

// radar sweep while driving: across to the left, back across to the right, and so on
const int SWEEP[] = { 90, 110, 130, 150, 130, 110, 90, 70, 50, 30, 50, 70 };
const byte SWEEP_LEN = sizeof(SWEEP) / sizeof(SWEEP[0]);
const unsigned long SWEEP_PING_MS = 70;     // servo moves 20 degrees in ~35 ms, then settles
const unsigned long PING_GAP_MS = 60;       // let one ping's echoes die away before the next

SoftwareSerial bt(BT_RX, BT_TX);
Servo look;

int maxSpeed = 150;          // PWM for a full joystick push, set by 0..9
bool autoMode = false;

unsigned long lastCommandAt = 0;
long distance = 999;
unsigned long lastPingAt = 0, lastReportAt = 0, lastRampAt = 0;
unsigned long servoReleaseAt = 0;  // detach after moving so it doesn't jitter
int servoAngle = 90;
unsigned long brakeUntil = 0;
bool guardOn = false;          // manual forward guard is holding the car back
byte lostRun = 0;              // no-echo readings in a row after something close

int joyX = 0, joyY = 0;      // last joystick position, -100..100
int targetL = 0, targetR = 0;  // signed PWM the motors are heading to
int curL = 0, curR = 0;        // signed PWM the motors have now

// joystick message being received, per source
struct LineIn { bool active = false; char buf[16]; byte len = 0; };
LineIn usbLine, btLine;

// auto mode steps
enum AutoStep { CRUISE, NUDGE, LOOKING_LEFT, LOOKING_RIGHT, BACKUP, TURN, VERIFY };
AutoStep step = CRUISE;
unsigned long stepUntil = 0;
long frontCm = 999, leftCm = 999, rightCm = 999;  // from the stop-and-look
bool turnLeft = false, bothBlocked = false;
byte verifyTry = 0;
int turnPwm = TURN_PWM_START;
long lastVerifyCm = -1, prevLeftCm = -1, prevRightCm = -1;
unsigned long autoStartedAt = 0, lastSampleAt = 0;

// what the sweep saw while driving
byte sweepIdx = 0;
long aheadCm = 999;                          // nearest thing ahead in the front cone
long nearLeftCm = 999, nearRightCm = 999;    // nearest on each side since the last log line
unsigned long slowUntil = 0;
byte lostEchoes = 0;         // no-echo readings ahead in a row
long stuckRefCm = 999;
unsigned long stuckSince = 0;

// ---- trip log (EEPROM ring buffer) ----
// byte 0 magic, 1 next slot, 2 run number; slots of 8 bytes from LOG_START.
// slot: run, event, tenths of a second since auto started (2 bytes), a, b, c, d

enum LogEvent : byte { EV_START = 1, EV_CRUISE, EV_BLOCKED, EV_LOOKED, EV_BACKUP, EV_TURN, EV_END,
                       EV_NUDGE, EV_VERIFY, EV_STUCK, EV_BOOST, EV_BOOT };
const byte EV_LAST = EV_BOOT;
enum BlockReason : byte { BY_AHEAD, BY_LEFT, BY_RIGHT, BY_LOST_ECHO };
const byte LOG_MAGIC = 0xA8;  // changed for auto v3: wipes the old-format log once
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

void logEvent(byte ev, byte a = 0, byte b = 0, byte c = 0, byte d = 0) {
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
  EEPROM.update(at + 7, d);
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
    if (ev < EV_START || ev > EV_LAST) continue;  // empty slot
    unsigned int tenths = EEPROM.read(at + 2) | (EEPROM.read(at + 3) << 8);
    byte a = EEPROM.read(at + 4), b = EEPROM.read(at + 5), c = EEPROM.read(at + 6), d = EEPROM.read(at + 7);
    Serial.print(F("run ")); Serial.print(EEPROM.read(at));
    Serial.print(F("  ")); Serial.print(tenths / 10); Serial.print('.'); Serial.print(tenths % 10);
    Serial.print(F("s  "));
    switch (ev) {
      case EV_START:   Serial.print(F("START auto")); break;
      case EV_CRUISE:  Serial.print(F("cruise  ahead ")); printCm(a);
                       Serial.print(F("  left ")); printCm(b);
                       Serial.print(F("  right ")); printCm(c);
                       Serial.print(F("  motors ")); Serial.print(d); break;
      case EV_BLOCKED: Serial.print(F("BLOCKED "));
                       if (b == BY_LEFT) Serial.print(F("left corner "));
                       else if (b == BY_RIGHT) Serial.print(F("right corner "));
                       else if (b == BY_LOST_ECHO) Serial.print(F("echo lost, was "));
                       else Serial.print(F("ahead "));
                       printCm(a); break;
      case EV_LOOKED:  Serial.print(F("looked  left ")); printCm(a);
                       Serial.print(F("  right ")); printCm(b); break;
      case EV_BACKUP:  Serial.print(F("BACKUP")); break;
      case EV_TURN:    Serial.print(F("TURN ")); Serial.print(a ? F("left ") : F("right "));
                       Serial.print(b * 10); Serial.print(F("ms  power ")); Serial.print(c); break;
      case EV_END:     Serial.print(F("END auto")); break;
      case EV_NUDGE:   Serial.print(F("nudge away from ")); Serial.print(b ? F("left ") : F("right "));
                       printCm(a); Serial.print(F(" at ")); Serial.print(c); Serial.print(F(" deg")); break;
      case EV_VERIFY:  Serial.print(F("check ahead ")); printCm(a);
                       Serial.print(a >= CLEAR_CM ? F("  clear") : F("  still blocked"));
                       Serial.print(F(" (try ")); Serial.print(b); Serial.print(')'); break;
      case EV_STUCK:   Serial.print(F("STUCK ahead ")); printCm(a); break;
      case EV_BOOST:   Serial.print(F("didn't rotate -> turn power ")); Serial.print(a); break;
      case EV_BOOT:    Serial.print(F("ARDUINO RESTARTED (power-on, USB plug-in, or a power/ground glitch)")); break;
    }
    Serial.println();
  }
  Serial.println(F("LOG END"));
}

// ---- trip memory (for going home) ----

struct Move { int8_t l, r; uint8_t t; };  // motor targets / 2, how long in 20 ms units
const int PATH_MAX = 220;
const int8_t BRAKED = -128;               // l of a stop that braked rather than rolled
const unsigned long HOME_STOP_MS = 300;   // stops are replayed this long at most
const int HOME_GUARD_CM = 10;
Move path[PATH_MAX];
int pathLen = 0;
bool pathFull = false;
Move seg;                                 // the move happening now, not stored yet
unsigned long segStart = 0;
bool segOpen = false;

bool homing = false;
int homeIdx = 0;
unsigned long homeUntil = 0;

// close enough to count as the same move (cruise 65 vs 75 is not)
bool sameMove(int8_t a, int8_t b) {
  return (a > 0) == (b > 0) && (a < 0) == (b < 0) && abs(a - b) <= 3;
}

void closeMove(unsigned long now) {
  if (!segOpen) return;
  segOpen = false;
  unsigned long t = (now - segStart + 10) / 20;
  if (t == 0) return;
  bool stop = seg.l == 0 || seg.l == BRAKED;
  // standing still before the first move, or a stop after a stop, changes nothing
  if (stop && (pathLen == 0 || path[pathLen - 1].l == 0 || path[pathLen - 1].l == BRAKED)) return;
  if (pathLen >= PATH_MAX) {
    if (!pathFull) say("PATH FULL");
    pathFull = true;
    return;
  }
  seg.t = t;
  path[pathLen++] = seg;
}

void recordMotors(unsigned long now) {
  if (homing || pathFull) return;
  int8_t l = targetL / 2, r = targetR / 2;
  if (l == 0 && r == 0 && now < brakeUntil) l = BRAKED;
  if (segOpen && now - segStart < 255UL * 20 &&
      (l == BRAKED ? seg.l == BRAKED : sameMove(l, seg.l) && sameMove(r, seg.r))) return;
  closeMove(now);
  seg.l = l; seg.r = r;
  segStart = now;
  segOpen = true;
}

void clearPath() {
  pathLen = 0;
  pathFull = false;
  segOpen = false;
}

// ---- servo ----

void lookAt(int angle) {
  servoAngle = angle;
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

// Stop hard: both direction pins low with full enable shorts each motor
// (L298N "fast motor stop"), so the car doesn't roll on into what it saw.
void brakeNow() {
  stopNow();
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 255); analogWrite(ENB, 255);
  brakeUntil = millis() + BRAKE_MS;
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

// how close is too close depends on how fast it's going
int guardCm() {
  int fwd = max(0, (curL + curR) / 2);
  return GUARD_CM + (long)(GUARD_FAST_CM - GUARD_CM) * fwd / 255;
}

void applyJoystick() {
  int y = joyY;
  bool block = y > 0 && distance < guardCm();
  if (block) {
    y = 0;  // no forward into an obstacle (turning away still works)
    if (!guardOn && curL > 0 && curR > 0) {
      int x = joyX;
      brakeNow();  // stop hard rather than rolling on into it
      joyX = x; joyY = 0;
    }
  }
  guardOn = block;
  setTargets(joyX, y, maxSpeed);
}

// ---- sensor ----

// 0 means no usable reading (closer than the sensor's 2 cm minimum is noise)
long readDistanceCm() {
  static unsigned long lastTrigAt = 0;
  while (millis() - lastTrigAt < PING_GAP_MS) {}  // an early ping hears the last one's echoes
  lastTrigAt = millis();
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseInLong(ECHO, HIGH, 25000);
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
  }
  long lo = min(r[0], min(r[1], r[2])), hi = max(r[0], max(r[1], r[2]));
  return r[0] + r[1] + r[2] - lo - hi;
}

// Stand still and look both ways, for checking the servo and which way is left.
void lookTest() {
  stopNow();
  lookAt(LOOK_LEFT);   delay(LOOK_SETTLE_MS * 2); long l = readSteadyCm();
  lookAt(LOOK_RIGHT);  delay(LOOK_SETTLE_MS * 2); long r = readSteadyCm();
  lookAt(LOOK_CENTER); delay(LOOK_SETTLE_MS);     long c = readSteadyCm();
  say("LOOK left " + String(l) + " right " + String(r) + " ahead " + String(c));
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
  turnPwm = TURN_PWM_START;
  lastVerifyCm = prevLeftCm = prevRightCm = -1;
  stopNow();
  lookAt(LOOK_CENTER);
  startCruise(millis());
  say(on ? "MODE auto" : "MODE manual");
}

void handleJoystick(const char *msg) {
  int x, y, sum;
  if (sscanf(msg, "%d,%d,%d", &x, &y, &sum) != 3) return;
  if (sum != x + y || abs(x) > 100 || abs(y) > 100) return;  // garbled
  if (autoMode || homing) return;
  joyX = x; joyY = y;
  lastCommandAt = millis();
  applyJoystick();
}

void handleCommand(char c, bool fromUsb) {
  if (c >= '0' && c <= '9') {
    maxSpeed = map(c - '0', 0, 9, 80, 255);
    if (!autoMode && !homing) applyJoystick();
    return;
  }
  switch (c) {
    case 'A': stopHome(); setMode(true);  return;
    case 'M': stopHome(); setMode(false); return;
    case 'S':
      if (homing) stopHome();         // stop always wins
      else if (autoMode) setMode(false);
      else stopNow();
      return;
    case 'H':
      if (!homing) startHome();
      return;
    case 'P':
      if (!homing) { clearPath(); say("TRIP 0"); }
      return;
    case 'G':
      if (fromUsb) printLog();  // too long for the Bluetooth link
      return;
    case 'V':
      if (!autoMode && !homing) lookTest();
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

void startCruise(unsigned long now) {
  step = CRUISE;
  sweepIdx = 0;
  aheadCm = nearLeftCm = nearRightCm = 999;
  lastVerifyCm = -1;
  lostEchoes = 0;
  stuckRefCm = 999;
  stuckSince = lastSampleAt = now;
}

// Stop and look both ways (the servo is already somewhere in its sweep).
void blocked(long cm, byte why, unsigned long now) {
  frontCm = cm;
  brakeNow();
  logEvent(EV_BLOCKED, cmByte(cm), why);
  verifyTry = 0;
  lookAt(LOOK_LEFT);
  step = LOOKING_LEFT;
  stepUntil = now + LOOK_SETTLE_MS;
}

void startTurn(unsigned long now) {
  unsigned long ms = bothBlocked ? UTURN_MS : TURN_MS;
  setTargets(turnLeft ? -100 : 100, 0, turnPwm);
  logEvent(EV_TURN, turnLeft, ms / 10, turnPwm);
  step = TURN;
  stepUntil = now + ms;
}

bool same(long a, long b) { return a >= 0 && b >= 0 && a < 999 && b < 999 && abs(a - b) <= 3; }

// A turn that leaves the sensor seeing the same thing didn't rotate the car:
// the tyres are dragging on this floor, so push harder next time.
void checkRotated(bool rotated) {
  if (rotated || turnPwm >= TURN_PWM_MAX) return;
  turnPwm = min(turnPwm + TURN_PWM_STEP, TURN_PWM_MAX);
  logEvent(EV_BOOST, turnPwm);
}

void startBackup(unsigned long now) {
  setTargets(0, -100, AUTO_MAX_PWM);
  logEvent(EV_BACKUP);
  step = BACKUP;
  stepUntil = now + BACKUP_MS;
}

// One radar reading while driving. angle > 90 is the left side.
void cruiseSaw(int angle, long cm, unsigned long now) {
  int off = abs(angle - LOOK_CENTER);
  bool onLeft = angle > LOOK_CENTER;
  bool front = off <= 20;

  if (cm >= 999) {
    // No echo. Indoors that usually means a slanted or soft surface, not
    // open space - if something was close a moment ago, don't trust it.
    if (front && aheadCm < SUSPECT_CM && ++lostEchoes >= 2) blocked(aheadCm, BY_LOST_ECHO, now);
    if (angle == LOOK_CENTER) { stuckRefCm = 999; stuckSince = now; }  // tells us nothing about being stuck
    return;
  }

  // where the thing is relative to the car: how far ahead, how far from the middle
  float rad = off * (PI / 180.0);
  long ahead = cm * cos(rad), lane = cm * sin(rad);

  if (front) { lostEchoes = 0; aheadCm = ahead; }
  if (off) {
    long &nearest = onLeft ? nearLeftCm : nearRightCm;
    nearest = min(nearest, cm);
  }

  if (lane < LANE_HALF_CM && ahead < AVOID_CM) {
    blocked(cm, front ? BY_AHEAD : onLeft ? BY_LEFT : BY_RIGHT, now);
    return;
  }
  if (lane < NUDGE_LANE_CM && ahead < SLOW_CM) slowUntil = now + 600;
  if (off && lane < NUDGE_LANE_CM && ahead < NUDGE_AHEAD_CM) {
    logEvent(EV_NUDGE, cmByte(cm), onLeft, off);
    setTargets(onLeft ? 100 : -100, 0, turnPwm);  // spin away from it, briefly
    step = NUDGE;
    stepUntil = now + NUDGE_MS;
    return;
  }

  if (angle != LOOK_CENTER) return;
  if (abs(cm - stuckRefCm) > STUCK_CHANGE_CM) { stuckRefCm = cm; stuckSince = now; }
  else if (now - stuckSince > STUCK_MS) {
    logEvent(EV_STUCK, cmByte(cm));
    brakeNow();
    bothBlocked = true;  // back up, then turn about
    turnLeft = nearLeftCm > nearRightCm;
    startBackup(now);
  }
}

void runAuto(unsigned long now) {
  switch (step) {
    case CRUISE: {
      int top = min(maxSpeed, now < slowUntil || lostEchoes ? AUTO_SLOW_PWM : AUTO_MAX_PWM);
      setTargets(0, 100, top);
      if (now - lastSampleAt >= 1000) {
        lastSampleAt = now;
        logEvent(EV_CRUISE, cmByte(aheadCm), cmByte(nearLeftCm), cmByte(nearRightCm), abs(curL));
        nearLeftCm = nearRightCm = 999;
      }
      break;
    }
    case NUDGE:
      if (now >= stepUntil) step = CRUISE;  // keeps sweeping from where it was
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
        // the same view as the last look: the turn in between didn't rotate the car
        checkRotated(!(same(leftCm, prevLeftCm) && same(rightCm, prevRightCm)));
        prevLeftCm = leftCm; prevRightCm = rightCm;
        turnLeft = leftCm > rightCm;
        bothBlocked = max(leftCm, rightCm) < AVOID_CM;
        now = millis();
        if (bothBlocked || frontCm < BACKUP_CM) startBackup(now);
        else startTurn(now);
      }
      break;
    case BACKUP:
      if (now >= stepUntil) startTurn(now);
      break;
    case TURN:
      if (now >= stepUntil) {
        brakeNow();
        lookAt(LOOK_CENTER);
        step = VERIFY;
        stepUntil = now + LOOK_SETTLE_MS;
      }
      break;
    case VERIFY:
      // Don't drive off after a turn without looking: the old code turned
      // straight into the next obstacle.
      if (now >= stepUntil) {
        long cm = readSteadyCm();
        verifyTry++;
        logEvent(EV_VERIFY, cmByte(cm), verifyTry);
        if (verifyTry > 1) checkRotated(!same(cm, lastVerifyCm));
        lastVerifyCm = cm;
        now = millis();
        if (cm >= CLEAR_CM) {
          startCruise(now);
        } else if (cm < BACKUP_CM) {
          bothBlocked = false;  // nose against something: back off before turning again
          startBackup(now);
        } else if (verifyTry < VERIFY_TRIES) {
          bothBlocked = false;  // a bit more the same way
          startTurn(now);
        } else {
          verifyTry = 0;        // boxed in: back out and turn about
          bothBlocked = true;
          startBackup(now);
        }
      }
      break;
  }
}

// ---- going home ----

void startHome() {
  if (autoMode) setMode(false);
  stopNow();
  closeMove(millis());
  if (pathFull) { say("HOME too far"); return; }
  if (pathLen == 0) { say("HOME already"); return; }
  lookAt(LOOK_CENTER);
  homing = true;
  homeIdx = pathLen;
  homeUntil = 0;
  say("MODE home");
}

void stopHome() {
  if (!homing) return;
  homing = false;
  stopNow();
  // where it stopped is unknown, so the trip so far can't be undone any more
  clearPath();
  say("MODE manual");
}

void runHome(unsigned long now) {
  bool forward = targetL > 0 && targetR > 0;
  if (forward && distance < HOME_GUARD_CM) homeUntil = now;  // don't drive into it, skip ahead
  if (now < homeUntil) return;
  if (homeIdx == 0) {
    brakeNow();
    homing = false;
    clearPath();  // here is the start again
    say("HOME done");
    say("MODE manual");
    return;
  }
  Move m = path[--homeIdx];
  unsigned long ms = m.t * 20UL;
  if (m.l == BRAKED) { brakeNow(); ms = min(ms, HOME_STOP_MS); }
  else if (m.l == 0 && m.r == 0) { stopNow(); ms = min(ms, HOME_STOP_MS); }
  else { targetL = -m.l * 2; targetR = -m.r * 2; }
  homeUntil = now + ms;
}

void report(unsigned long now) {
  static unsigned long lastTripAt = 0;
  say("D " + String(distance));
  if (now - lastTripAt >= 2000) {
    lastTripAt = now;
    say("TRIP " + String(pathFull ? 100 : pathLen * 100L / PATH_MAX));
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
  // a restart in the middle of a drive = power or ground trouble (one line per burst)
  byte lastSlot = (EEPROM.read(1) + LOG_SLOTS - 1) % LOG_SLOTS;
  if (EEPROM.read(LOG_START + lastSlot * LOG_SLOT + 1) != EV_BOOT) logEvent(EV_BOOT);

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
  bool sweeping = autoMode && step == CRUISE;

  if (now - lastPingAt >= (sweeping ? SWEEP_PING_MS : 100)) {
    lastPingAt = now;
    long cm = readDistanceCm();
    int angle = servoAngle;
    if (cm != 0 && angle == LOOK_CENTER) {  // skip noise, keep the last good reading
      // An echo that vanishes right after something close is usually a wall at
      // an angle bouncing the sound away, not open space: hold the close reading a while.
      if (cm >= 999 && distance < SUSPECT_CM && lostRun < 8) lostRun++;
      else { distance = cm; lostRun = 0; }
    }
    if (sweeping) {
      if (cm != 0) cruiseSaw(angle, cm, now);
      if (step == CRUISE) {
        sweepIdx = (sweepIdx + 1) % SWEEP_LEN;
        lookAt(SWEEP[sweepIdx]);
        // Report right as the servo starts moving: SoftwareSerial blocks
        // interrupts while sending, which can twitch the servo, and it has
        // a whole ping interval to settle before the next reading.
        if (angle == LOOK_CENTER) { lastReportAt = now; report(now); }
      }
    }
    if (!autoMode && !homing) applyJoystick();  // re-check the forward guard
  }

  // Report less often than we measure: SoftwareSerial can't receive while it sends.
  if (!sweeping && now - lastReportAt >= 200) {
    lastReportAt = now;
    report(now);
  }

  if (servoReleaseAt && now >= servoReleaseAt) {
    servoReleaseAt = 0;
    look.detach();
  }

  if (homing) {
    runHome(now);
  } else if (autoMode) {
    runAuto(now);
  } else if ((joyX || joyY) && now - lastCommandAt > COMMAND_TIMEOUT_MS) {
    stopNow();  // remote went quiet
  }

  recordMotors(now);
  int dt = now - lastRampAt;
  if (now < brakeUntil) {
    lastRampAt = now;  // hold the brake; the ramp starts from rest afterwards
  } else if (dt > 0) {
    lastRampAt = now;
    curL = approach(curL, targetL, dt);
    curR = approach(curR, targetR, dt);
    left(curL); right(curR);
  }
}
