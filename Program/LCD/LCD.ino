#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusMaster.h>
#include <Adafruit_INA219.h>
#include <MAX6675_Thermocouple.h>
#include <Servo.h>

//Fuel Sensor
int TankValue0;
int fuelPin = A1;

//Servo
Servo myServo;
int servo_1 = 3;
int pos = 0;
int varDeg;

//RELAY
int relay_1 = 11;
int relay_2 = 4;
int relay_3 = 5;
int relay_4 = 6;
int relay_5 = 7;
int relay_6 = 8;

//MAX_RS485
const uint8_t MAX485_DE = A2;
const uint8_t MAX485_RE_NEG = A3;
ModbusMaster node;
int dataStart;
int dataStop;
int function1;
int function2;
int function3;
int function4;

//Proximity Sensor
const int IR_PIN = 2;  // IR sensor input pin
volatile unsigned int counter = 0;  // Counter variable for revolutions
unsigned long previousMillis = 0;  // Variable to store previous time
unsigned int rpm = 0;  // Variable to store RPM value

//THERMOCOUPLE MAX6675
#define SCK_PIN 13
#define CS_PIN 9
#define CS_PIN2 10
#define SO_PIN 12
Thermocouple* thermocouple;
Thermocouple* thermocouple2;

//INA219
Adafruit_INA219 ina219(0x40);
float current_mA = 0;

//VOLTAGE SENSOR
#define ANALOG_IN_PIN A0
float adc_voltage = 0.0;
float in_voltage = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
float ref_voltage = 5.0;
int adc_value = 0;

//LCD 12C
LiquidCrystal_I2C lcd(0x27, 20, 4);
byte fullBlock[8] = { 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111 };
byte emptyBlock[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };

unsigned long lcdUpdateTime = 0;
unsigned long rs485UpdateTime = 0;
const unsigned long lcdInterval = 200; // Update every 200 milliseconds
const unsigned long rs485Interval = 2000; // 1 second

void setup()
{
  // Setting up RS-485 / MAX 485
  //  pinMode(MAX485_RE_NEG, OUTPUT);
  //  pinMode(MAX485_DE, OUTPUT);
  //  digitalWrite(MAX485_RE_NEG, 0);
  //  digitalWrite(MAX485_DE, 0);

  //Setting up Servo
  myServo.attach(servo_1);

  //Setting up Relay
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);
  pinMode(relay_5, OUTPUT);
  pinMode(relay_6, OUTPUT);

  //OFF RELAY
  digitalWrite(relay_1, HIGH);
  digitalWrite(relay_2, HIGH);
  digitalWrite(relay_3, HIGH);
  digitalWrite(relay_4, HIGH);
  digitalWrite(relay_5, HIGH);
  digitalWrite(relay_6, HIGH);
  //Setting up Sensor (RPM, FUEL, Thermocouple 1, Thermocouple 2, Current, Voltage)
  pinMode(IR_PIN, INPUT_PULLUP);
  pinMode(fuelPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), Interrupt, FALLING);
  thermocouple = new MAX6675_Thermocouple(SCK_PIN, CS_PIN, SO_PIN);
  thermocouple2 = new MAX6675_Thermocouple(SCK_PIN, CS_PIN2, SO_PIN);
  //  if (!ina219.begin())
  //  {
  //    //    Serial.println("Failed to find INA219 chip");
  //    while (1)
  //    {
  //      delay(10);
  //    }
  //  }
  pinMode(ANALOG_IN_PIN, INPUT);

  //OFF STARTER
  for (pos = 35; pos <= 100; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits 15 ms for the servo to reach the position
  }
  //  for (pos = 70; pos >= 10; pos -= 1) { // goes from 0 degrees to 180 degrees
  //    // in steps of 1 degree
  //    myServo.write(pos);              // tell servo to go to position in variable 'pos'
  //    delay(50);                       // waits 15 ms for the servo to reach the position
  //  }
  delay(1000);


  //LCD init and modbus init
  delay(2000);
  Serial.begin(9600);
  //  node.begin(1, Serial);
  lcd.begin();
  lcd.backlight();
  bootScreen();
  lcd.clear();
  Serial.println("DONE");
  //  node.preTransmission(preTransmission);         //Callback for configuring RS-485 Transreceiver correctly
  //  node.postTransmission(postTransmission);
}

void loop()
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[6];
  unsigned long currentMillis = millis();

  //FUEL
  int fuel = analogRead(fuelPin);
  fuel = map(fuel, 548, 1024, 100, 0);
  //
  //RPM Sensor
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    rpm = (counter / 2) * 60;  // Calculate RPM
    counter = 0;  // Reset counter
  }

  //THERMOCOUPLER
  const double celsius = thermocouple->readCelsius();
  const double celsius2 = thermocouple2->readCelsius();

  //ina2109
  current_mA = ina219.getCurrent_mA();

  //voltage sensor
  adc_value = analogRead(ANALOG_IN_PIN);
  adc_voltage  = (adc_value * ref_voltage) / 1024.0;
  in_voltage = adc_voltage / (R2 / (R1 + R2)) ;

  lcd.setCursor(8, 0);
  lcd.print("ECU");
  lcd.setCursor(0, 1);
  lcd.print("V:" + String(in_voltage) + "V");
  lcd.setCursor(0, 2);
  lcd.print("I:" + String(current_mA) + "A");
  lcd.setCursor(9, 1);
  lcd.print("tmp1:" + String(celsius) + "C"); // Use previousCelsius
  lcd.setCursor(9, 2);
  lcd.print("tmp2:" + String(celsius2) + "C"); // Use previousCelsius2
  lcd.setCursor(0, 3);
  lcd.print("RPM:" + String(rpm) + "rpm" + " Fuel:" + String(fuel) + "%");

  if (Serial.available() > 0) {
    // Baca byte yang masuk
    char incomingByte = Serial.read();
    int degVar = Serial.parseInt(); // Baca nilai integer

    // Pastikan degVar memiliki nilai yang valid
    if (degVar >= 0 && degVar <= 100) {
      // Pemetaan nilai
      varDeg = map(degVar, 0, 100, 32, 10);
      Serial.print("derajat : ");
      Serial.println(varDeg); // Cetak nilai yang dipetakan
    } else {
      Serial.println("Input tidak valid. Masukkan angka antara 0 dan 100.");
    }

    if (incomingByte == 'a') {
      Serial.println("Starting");
      digitalWrite(relay_1, LOW);
      delay(4000);
      for (pos = 100; pos >= 35; pos -= 1) {
        myServo.write(pos);
        delay(50);
      }
      delay(4000);
      digitalWrite(relay_2, LOW);
      unsigned long startTime = millis();
      unsigned long elapsedTime = 0;
      bool safe = true;
      while (elapsedTime < 5000) {
        if (rpm > 1000) {
          Serial.println("RPM aman, melanjutkan...");
          safe = true;
          break;
        }
        Serial.println("Mencoba membaca rpm");
        Serial.println(rpm);
        elapsedTime = millis() - startTime;
        delay(100);
      }
      if (safe && rpm <= 1000) {
        Serial.println("RPM tidak aman, mematikan relay dan mengembalikan servo...");
        digitalWrite(relay_2, HIGH);
        delay(5000);
        for (pos = 100; pos >= 35; pos -= 1) {
          myServo.write(pos);
          delay(50);
        }
        delay(5000);
        digitalWrite(relay_1, HIGH);
        delay(5000);

      }
    } else if (incomingByte == 'b') { // STOP
      Serial.println("Stopping");
      for (pos = 35; pos <= 100; pos += 1) {
        myServo.write(pos);
        delay(50);
      }
      delay(4000);
      digitalWrite(relay_2, HIGH);
      delay(4000);
      digitalWrite(relay_1, HIGH);
      unsigned long startTime2 =  millis();
      unsigned long elapsedTime2 = 0;
      boolean safe2 = true;
      while (elapsedTime2 < 5000){
        if (rpm == 0){
          Serial.println("Engine berhenti sempurna");
          safe2 = true;
          break;
        }
        Serial.println("Engine sedang proses berhenti");
        elapsedTime2 = millis() - startTime2;
        delay(100);
      }
      if (safe2 && rpm > 0){
        Serial.println("Engine gagal mati");
      }
    } else if (incomingByte == 'c') {
      for (pos = 70; pos >= 10; pos -= 1) {
        myServo.write(pos);
        delay(50);
      }
    }
  }
  // Send data via RS485 every rs485Interval milliseconds
  //  if (currentMillis - rs485UpdateTime >= rs485Interval) {
  //    rs485UpdateTime = currentMillis;
  //    node.writeSingleRegister(0x40001, in_voltage);
  //    node.writeSingleRegister(0x40002, current_mA);
  //    node.writeSingleRegister(0x40003, celsius);
  //    node.writeSingleRegister(0x40004, celsius2);
  //    node.writeSingleRegister(0x40005, rpm);
  //    node.writeSingleRegister(0x40006, fuel);
  //    delay(2000);
  //    //    result = node.readHoldingRegisters(0x40001, 6);
  //    //    if (result == node.ku8MBSuccess) {
  //    //      dataStart = node.getResponseBuffer(0);
  //    //      dataStop = node.getResponseBuffer(1);
  //    //      function1 = node.getResponseBuffer(2);
  //    //      function2 = node.getResponseBuffer(3);
  //    //      function3 = node.getResponseBuffer(4);
  //    //      function4 = node.getResponseBuffer(5);
  //    //
  //    //      // Mencetak hasil pembacaan register
  //    //      Serial.print("Data Start: ");
  //    //      Serial.println(dataStart);
  //    //      Serial.print("Data Stop: ");
  //    //      Serial.println(dataStop);
  //    //      Serial.print("Function 1: ");
  //    //      Serial.println(function1);
  //    //      Serial.print("Function 2: ");
  //    //      Serial.println(function2);
  //    //      Serial.print("Function 3: ");
  //    //      Serial.println(function3);
  //    //      Serial.print("Function 4: ");
  //    //      Serial.println(function4);
  //    //
  //    //      //Relay Logic
  //    //      if (dataStart >= 1) {
  //    //        for (pos = 70; pos >= 10; pos -= 1) { // goes from 180 degrees to 0 degrees
  //    //          myServo.write(pos);              // tell servo to go to position in variable 'pos'
  //    //          delay(15);                       // waits 15 ms for the servo to reach the position
  //    //        }
  //    //        digitalWrite(relay_1, LOW);
  //    //      } else {
  //    //        Serial.println("Start Function OFF");
  //    //        for (pos = 10; pos <= 70; pos += 1) { // goes from 0 degrees to 180 degrees
  //    //          // in steps of 1 degree
  //    //          myServo.write(pos);              // tell servo to go to position in variable 'pos'
  //    //          delay(15);                       // waits 15 ms for the servo to reach the position
  //    //        }
  //    //        digitalWrite(relay_1, HIGH);
  //    //      }
  //    //
  //    //      if (dataStop >= 1) {
  //    //        digitalWrite(relay_2, LOW);
  //    //      } else {
  //    //        digitalWrite(relay_2, HIGH);
  //    //      }
  //    //
  //    //      if (function1 >= 1) {
  //    //        digitalWrite(relay_3, LOW);
  //    //      } else {
  //    //        digitalWrite(relay_3, HIGH);
  //    //      }
  //    //
  //    //      if (function2 >= 1) {
  //    //        digitalWrite(relay_4, LOW);
  //    //      } else {
  //    //        digitalWrite(relay_4, HIGH);
  //    //      }
  //    //
  //    //      if (function3 >= 1) {
  //    //        digitalWrite(relay_5, LOW);
  //    //      } else {
  //    //        digitalWrite(relay_5, HIGH);
  //    //      }
  //    //
  //    //      if (function4 >= 1) {
  //    //        digitalWrite(relay_6, HIGH);
  //    //      } else {
  //    //        digitalWrite(relay_6, HIGH);
  //    //      }
  //    //    } else {
  //    //      Serial.print("Data : ");
  //    //      Serial.println(result);
  //    //    }
  //  }
}

//Function for boot LCD
void bootScreen()
{
  lcd.setCursor(5, 1);
  lcd.print("ELECTRICAL");
  lcd.setCursor(4, 2);
  lcd.print("CONTROL UNIT");
  delay(1000);
  lcd.clear();
  lcd.createChar(0, fullBlock);
  lcd.createChar(1, emptyBlock);
  lcd.setCursor(1, 1);
  lcd.print("LOADING...");

  lcd.setCursor(0, 2);
  lcd.write(byte(1));
  for (int i = 1; i < 19; i++) {
    lcd.write(byte(1));
  }
  lcd.write(byte(1));

  for (int progress = 1; progress <= 18; progress++)
  {
    lcd.setCursor(progress, 2);
    lcd.write(byte(0));
    delay(150);
  }
  delay(1000);
  lcd.clear();
}

//Interrupt function for RPM Sensor
void Interrupt() {
  counter++;
}

//
void preTransmission()            //Function for setting state of Pins DE & RE of RS-485
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}
