void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW); // Initialize pin 7 as HIGH
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim(); // Remove any leading/trailing whitespace
    if (data == "open") {
      digitalWrite(7, LOW); // Set pin 7 to LOW
      Serial.println("gripper opened");
    } else if (data == "close") {
      digitalWrite(7, HIGH); // Set pin 7 to HIGH
      Serial.println("gripper closed");
    }
  }
}
