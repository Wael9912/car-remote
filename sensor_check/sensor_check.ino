// Is the servo upsetting the ultrasonic sensor? Alternates 10 s with the servo
// held at centre and 10 s with it let go, printing the distance. Motors held stopped.
#include <Servo.h>

const int ENA = 3, ENB = 11;
const int TRIG = A1, ECHO = A2;
const int SERVO_PIN = 10;
Servo look;

long pingCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, 30000) / 58;
}

void phase(bool held) {
  if (held) { look.write(90); look.attach(SERVO_PIN); }
  else look.detach();
  for (int i = 0; i < 20; i++) {
    Serial.print(held ? "held  " : "loose "); Serial.println(pingCm());
    delay(500);
  }
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  Serial.begin(9600);
  Serial.println("SERVO NOISE CHECK");
}

void loop() {
  phase(true);
  phase(false);
}
