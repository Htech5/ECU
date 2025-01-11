#include <ModbusMaster.h>

#define MAX485_DE      A2
#define MAX485_RE_NEG  A3

ModbusMaster node;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup() {
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  pinMode(4, INPUT);
  pinMode(5, INPUT);

  Serial.begin(115200);
  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  uint16_t value = 100;
  node.writeSingleRegister(0x40000, value); // Mengirimkan nilai acak ke register 0x40000

  delay(1000); // Delay 1 detik sebelum iterasi berikutnya
}
