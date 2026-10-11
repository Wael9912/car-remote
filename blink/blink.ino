// Test: fast blink on the onboard "L" LED, and say hello over USB serial.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH); delay(150);
  digitalWrite(LED_BUILTIN, LOW);  delay(150);
  Serial.println("Hello from Arduino!");
}
