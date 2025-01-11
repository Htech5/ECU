#include "max6675.h"

// Temperature Sensor1
int thermo1D0 = 4; // SO
int thermo1CS = 5;
int thermo1CLK = 6; // SCK
float temp1 = 0;
MAX6675 thermocouple1(thermo1CLK, thermo1CS, thermo1D0);

// Temperature Sensor2
int thermo2D0 = 8; // SO
int thermo2CS = 9;
int thermo2CLK = 10; // SCK
float temp2 = 0;
MAX6675 thermocouple2(thermo2CLK, thermo2CS, thermo2D0);

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);
  delay(2000); // Wait a bit for the system to stabilize
}

void loop() {
  // Read temperatures from each sensor
  temp1 = thermocouple1.readCelsius();
  delay(100);  // Delay to allow for sensor response

  temp2 = thermocouple2.readCelsius();
  delay(100);  // Delay to allow for sensor response

  // Print temperature readings to Serial Monitor
  Serial.print("Temp1: ");
  Serial.print(temp1);
  Serial.println(" C");

  Serial.print("Temp2: ");
  Serial.print(temp2);
  Serial.println(" C");

  delay(1000);  // Delay before next readout
}
