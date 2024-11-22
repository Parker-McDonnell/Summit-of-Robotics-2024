#define PIN_1_2 7  // Shared pin for Q1 (PNP) and Q2 (NPN)
#define P3_4 5       // Q3: PNP transistor for backward

void setup() {
  pinMode(PIN_1_2, OUTPUT); // Shared pin for Q1 and Q2
  pinMode(P3_4, OUTPUT);      // Separate pin for Q3
  stopMotor();              // Stop motor at the start
}

void forwardMotor() {
  digitalWrite(PIN_1_2, LOW);  // Turn ON Q1 (PNP), OFF Q2 (NPN)
  digitalWrite(P3_4, HIGH);      // Turn OFF Q3 (PNP)
}

void backwardMotor() {
  digitalWrite(PIN_1_2, HIGH); // Turn OFF Q1 (PNP), ON Q2 (NPN)
  digitalWrite(P3_4, LOW);       // Turn ON Q3 (PNP)
}

void stopMotor() {
  digitalWrite(PIN_1_2, HIGH); // Turn OFF Q1 and Q2
  digitalWrite(P3_4, HIGH);      // Turn OFF Q3
}

void loop() {
  forwardMotor(); // Move forward
  delay(1000);    // Run for 2 seconds
  stopMotor();    // Stop the motor
  delay(1000);    // Wait for 1 second
  backwardMotor(); // Move backward
  delay(2000);     // Run for 2 seconds
  stopMotor();     // Stop the motor
  delay(1000);     // Wait for 1 second
}

