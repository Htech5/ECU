// -------------------------- Component Library -------------------------------- //
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusMaster.h>
#include <Adafruit_INA219.h>
#include <MAX6675_Thermocouple.h>
#include <Servo.h>

// ----------------------------- Fuel Sensor ---------------------------------- //
int TankValue;
const uint8_t fuelPin = A1;

// -------------------------------- Servo ------------------------------------- //
Servo myServo;
int servo = 3;
int pos = 0;
int varDeg;

// -------------------------------- Relay ------------------------------------- //
int relay1 = 11;
int relay2 = 4;
int relay3 = 5;
int relay4 = 6;
int relay5 = 7;
int relay6 = 8;

// ------------------------------ MAX_RS485 ----------------------------------- //
const uint8_t MAX485_DE = A2;
const uint8_t MAX485_RE_NEG = A3;
ModbusMaster node;
int startReg; //Reg for starting function when receive true value
int stopReg; //Reg for stopping function when receive true value
int mapDeg; //Reg for saving custom servo degree based on persentation (0% - 100%)
int changeDeg; //Move servo degree based on mapDeg when receive true value
int function1; //function 1 - 4 for further use
int function2;
int function3;
int function4;

// ------------------------------ RPM Sensor ---------------------------------- //
const int IR_PIN = 2;
volatile unsigned int counter = 0;
unsigned long previousMillis = 0;
unsigned long rpm = 0;
void Interrupt() {
  counter++;
}

// ------------------------------- MAX_6675 ---------------------------------- //
#define SCK_PIN 13
#define CS_PIN 9
#define CS_PIN2 10
#define SO_PIN 12
Thermocouple* thermocouple;
Thermocouple* thermocouple2;
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

// -------------------------------- INA_219 ---------------------------------- //
Adafruit_INA219 ina219(0x40);
float current_mA = 0;

// ----------------------------- Voltage Sensor ------------------------------ //
const uint8_t ANALOG_IN_PIN = A0;
float adc_voltage = 0.0;
float in_voltage = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
float ref_voltage = 5.0;
int adc_value = 0;

// -------------------------------- LCD_I2C --------------------------------- //
LiquidCrystal_I2C lcd(0x27, 20, 4);
byte fullBlock[8] = { 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111 };
byte emptyBlock[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
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

void setup() {
  // ----- RS485 ------ //
  Serial.begin(9600);
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // ----- Servo ------ //
  myServo.attach(servo);
  myServo.write(100);

  // ----- Relay ------ //
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  pinMode(relay5, OUTPUT);
  pinMode(relay6, OUTPUT);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, HIGH);
  digitalWrite(relay4, HIGH);
  digitalWrite(relay5, HIGH);
  digitalWrite(relay6, HIGH);

  // ------ RPM ------- //
  pinMode(IR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), Interrupt, FALLING);

  // ----- Fuel ------- //
  pinMode(fuelPin, INPUT_PULLUP);

  // -- Thermocouple -- //
  thermocouple = new MAX6675_Thermocouple(SCK_PIN, CS_PIN, SO_PIN);
  thermocouple2 = new MAX6675_Thermocouple(SCK_PIN, CS_PIN2, SO_PIN);

  // ----- Voltage ---- //
  pinMode(ANALOG_IN_PIN, INPUT);

  // ------ LCD ------- //
  lcd.begin();
  lcd.backlight();
  bootScreen();
  lcd.clear();
  //  Serial.println("SETUP DONE");
}

void loop() {
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[6];
  unsigned long currentMillis = millis();
  startReg = 0;
  stopReg = 0;
  changeDeg = 0;
  function1 = 0;
  function2 = 0;
  function3 = 0;
  function4 = 0;

  // ----- Fuel ------- //
  int fuel = analogRead(fuelPin);
  fuel = map(fuel, 548, 1024, 100, 0);

  // ------ RPM ------- //
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    rpm = (counter / 2) * 60;
    counter = 0;
  }

  // -- Thermocouple -- //
  const double celsius = thermocouple->readCelsius();
  const double celsius2 = thermocouple2->readCelsius();

  // ----- INA219 ----- //
  current_mA = ina219.getCurrent_mA();

  // ----- Voltage ---- //
  adc_value = analogRead(ANALOG_IN_PIN);
  adc_voltage  = (adc_value * ref_voltage) / 1024.0;
  in_voltage = adc_voltage / (R2 / (R1 + R2)) ;

  // ------ LCD ------- //
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

  // ----- RS485 ------ //
  result = node.readHoldingRegisters(0x40001, 8);
  startReg = node.getResponseBuffer(0);
  stopReg = node.getResponseBuffer(1);
  mapDeg = node.getResponseBuffer(2);
  changeDeg = node.getResponseBuffer(3);
  function1 = node.getResponseBuffer(4);
  function2 = node.getResponseBuffer(5);
  function3 = node.getResponseBuffer(6);
  function4 = node.getResponseBuffer(7);
  node.writeSingleRegister(0x40001, in_voltage);
  node.writeSingleRegister(0x40002, current_mA);
  node.writeSingleRegister(0x40003, celsius);
  node.writeSingleRegister(0x40004, celsius2);
  node.writeSingleRegister(0x40005, rpm);
  node.writeSingleRegister(0x40006, fuel);

  // ----- Logic ------ //
  if (mapDeg >= 0 && mapDeg <= 100) {
    varDeg = map(mapDeg, 0, 100, 32, 10);
    //Serial.print("Derajat Servo diubah : ");
    //Serial.println(varDeg);
  } else {
    //Serial.print("Nilai persentase tidak valid : ");
    //Serial.println(mapDeg);
  }

  if (startReg == 1) {
    node.writeSingleRegister(0x40007, 0);
    //Serial.println("Starting Sequence");
    digitalWrite(relay1, LOW);
    delay(4000);
    for (pos = 100; pos >= 35; pos -= 1) {
      myServo.write(pos);
      delay(50);
    }
    delay(4000);
    digitalWrite(relay2, LOW);
    unsigned long startTime = millis();
    unsigned long elapsedTime = 0;
    bool safe = true;
    while (elapsedTime < 10000) {
      if (rpm >= 1000) {
        //Serial.println("Engine start safely");
        node.writeSingleRegister(0x40007, 0);
        safe = true;
        break;
      }
      //Serial.println("Checking for RPM");
      //Serial.print("RPM : ");
      //Serial.println(rpm);
      elapsedTime = millis() - startime;
      delay(100);
    }
    if (safe && rpm < 1000) {
      //Serial.println("Engine Failure");
      node.writeSingleRegister(0x40007, 1);
      digitalWrite(relay2, HIGH);
      delay(4000);
      for (pos = 35; pos <= 100; pos += 1) {
        myServo.write(pos);
        delay(50);
      }
      delay(4000);
      digitalWrite(relay1, HIGH);
    }
  }

  if (stopReg == 1) {
    //Serial.println("Stopping");
    node.writeSingleRegister(0x40008, 0)
    digitalWrite(relay2, HIGH);
    delay(4000);
    for (pos = 35; pos <= 100; post += 1) {
      myServo.write(pos);
      delay(50);
    }
    delay(4000);
    digitalWrite(relay1, HIGH);
    unsigned long startTime2 = millis();
    unsigned long elapsedTime2 = 0;
    boolean safe2 = true;
    while (elapsedTime2 < 10000) {
      if (rpm <= 500) {
        //Serial.println("Engine stopped");
        node.writeSingleRegister(0x40008, 0)
        safe2 = true
                break;
      }
      //Serial.println("Engine stopping");
      elapsedTime2 = millis() - startTime2;
      delay(100);
    }
    if (safe && rpm > 500) {
      node.writeSingleRegister(0x40008, 1)
    }
  }

  if (changeDeg == 1) {
    myServo.write(varDeg);
  }

  //  if (function1 == 1) {
  //  }
  //
  //  if (function2 == 1) {
  //  }
  //
  //  if (function3 == 1){
  //  }
  //
  //  if (function4 == 1){
  //  }
}
