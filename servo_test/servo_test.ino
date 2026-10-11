// Servo test: the sensor's servo on pin 10 looks left, centre, right, centre.
// Motors are held stopped. Needs the battery in (servo is powered from the L298N 5V).
#include <Servo.h>

const int SERVO_PIN = 10;
const int ENA = 3, ENB = 11;
const int TRIG = A1, ECHO = A2;

Servo look;

long readDistanceCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 25000);
  return us == 0 ? 999 : us / 58;
}

void lookAt(int angle) {
  look.write(angle);
  delay(800);
  Serial.print("angle "); Serial.print(angle);
  Serial.print("  distance "); Serial.println(readDistanceCm());
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  Serial.begin(9600);
  look.attach(SERVO_PIN);
  Serial.println("SERVO TEST");
}

void loop() {
  lookAt(90);
  lookAt(30);
  lookAt(90);
  lookAt(150);
}
