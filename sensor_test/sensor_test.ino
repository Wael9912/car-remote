// Test: read the HC-SR04 ultrasonic sensor and print the distance in cm.
const int TRIG = A1;
const int ECHO = A2;

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 30000);  // 0 = no echo within ~5 m
  if (us == 0) Serial.println("no echo");
  else { Serial.print(us / 58); Serial.println(" cm"); }
  delay(200);
}
