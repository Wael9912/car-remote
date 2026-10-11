// Test: spin one motor side continuously so we can check direction and speed.
const int ENA = 3;           // left speed (PWM)
const int IN1 = 7, IN2 = 6;  // left direction
const int ENB = 11;          // right speed (PWM)
const int IN3 = 5, IN4 = 4;  // right direction

// Which test to run: 1 = left forward, 2 = right one way, 0 = all stop
const int TEST = 0;
const int SPEED = 55;        // 0..255

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // left forward is IN1 LOW / IN2 HIGH (the other way ran backward)
  digitalWrite(IN1, LOW); digitalWrite(IN2, TEST == 1);
  analogWrite(ENA, TEST == 1 ? SPEED : 0);

  digitalWrite(IN3, TEST == 2); digitalWrite(IN4, LOW);
  analogWrite(ENB, TEST == 2 ? SPEED : 0);
}

void loop() {}
