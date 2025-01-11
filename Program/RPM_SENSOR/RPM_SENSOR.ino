const int IR_PIN = 2;  // IR sensor input pin

volatile unsigned int counter = 0;  // Counter variable for revolutions
unsigned long previousMillis = 0;  // Variable to store previous time
unsigned int rpm = 0;  // Variable to store RPM value

void Interrupt() {
  counter++;
}

void setup() {
  Serial.begin(9600);  // Inisialisasi komunikasi serial
  pinMode(IR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), Interrupt, FALLING);
  delay(2000);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    rpm = (counter / 2) * 60;  // Calculate RPM
    Serial.print("RPM: ");
    Serial.println(rpm);
    counter = 0;  // Reset counter
  }
}
