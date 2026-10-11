// Test: drive forward, backward, turn left, turn right — 2 s each, repeating.
const int ENA = 3;           // left speed (PWM)
const int IN1 = 7, IN2 = 6;  // left direction
const int ENB = 11;          // right speed (PWM)
const int IN3 = 5, IN4 = 4;  // right direction

const int SPEED = 60;        // 0..255

// dir: 1 = forward, -1 = backward, 0 = stop
void left(int dir) {
  // left forward is IN1 LOW / IN2 HIGH
  digitalWrite(IN1, dir < 0); digitalWrite(IN2, dir > 0);
  analogWrite(ENA, dir ? SPEED : 0);
}

void right(int dir) {
  // right forward is IN3 LOW / IN4 HIGH
  digitalWrite(IN3, dir < 0); digitalWrite(IN4, dir > 0);
  analogWrite(ENB, dir ? SPEED : 0);
}

void drive(int l, int r, int ms) {
  left(l); right(r); delay(ms);
  left(0); right(0); delay(1000);
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void loop() {
  drive( 1,  1, 2000);  // forward
  drive(-1, -1, 2000);  // backward
  drive(-1,  1, 2000);  // turn left (spin in place)
  drive( 1, -1, 2000);  // turn right (spin in place)
  delay(2000);
}
