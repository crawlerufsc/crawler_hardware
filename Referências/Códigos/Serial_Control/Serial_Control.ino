void setup() {
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    ang = Serial.read();
    Serial.print("O angulo é de:");
    Serial.println(data);
  }
}
