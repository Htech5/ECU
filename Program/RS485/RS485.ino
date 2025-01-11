#include <CRC8.h>  // Library CRC8 (Install melalui Library Manager)

CRC8 crc;  // Objek CRC8

const int RE = A2;   // Pin untuk kontrol Receive Enable
const int DE = A3;   // Pin untuk kontrol Driver Enable

void setup() {
  // Setup komunikasi Serial
  Serial.begin(9600);    // Serial Monitor
  Serial1.begin(9600);   // Serial untuk RS485 pada TX/RX (Arduino Nano)

  // Setup pin kontrol
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  // Inisialisasi RS485 di mode menerima
  digitalWrite(RE, LOW);   // Aktifkan receive mode
  digitalWrite(DE, LOW);   // Nonaktifkan transmit mode

  Serial.println("RS485 Full-Duplex with CRC Initialized...");
}

void loop() {
  // Contoh mengirim data dengan CRC
  sendDataWithCRC("Hello from Arduino!");

  // Delay untuk memberi waktu mengirim dan menerima
  delay(1000);

  // Menerima data dengan CRC
  receiveDataWithCRC();
}

// Fungsi untuk menghitung dan mengirim data dengan CRC
void sendDataWithCRC(String message) {
  // Set ke mode transmit
  digitalWrite(RE, HIGH);
  digitalWrite(DE, HIGH);

  // Hitung CRC untuk data
  crc.reset();  // Reset CRC sebelum menghitung
  for (int i = 0; i < message.length(); i++) {
    crc.add(message[i]);
  }
  uint8_t crcValue = crc.getCRC();  // Dapatkan nilai CRC

  // Kirim data dan CRC
  Serial1.print(message);  // Kirim data
  Serial1.print(",");      // Separator antara data dan CRC
  Serial1.println(crcValue, DEC);  // Kirim CRC dalam bentuk desimal

  Serial.print("Sent: ");
  Serial.print(message);
  Serial.print(" | CRC: ");
  Serial.println(crcValue);

  // Kembali ke mode receive setelah mengirim
  digitalWrite(RE, LOW);
  digitalWrite(DE, LOW);
}

// Fungsi untuk menerima data dengan CRC
void receiveDataWithCRC() {
  if (Serial1.available()) {
    String received = Serial1.readStringUntil('\n');  // Membaca data hingga newline
    int separatorIndex = received.lastIndexOf(',');   // Mencari posisi koma pemisah

    if (separatorIndex != -1) {
      // Pisahkan data dan CRC
      String message = received.substring(0, separatorIndex);
      uint8_t receivedCRC = received.substring(separatorIndex + 1).toInt();

      // Hitung CRC ulang untuk data yang diterima
      crc.reset();
      for (int i = 0; i < message.length(); i++) {
        crc.add(message[i]);
      }
      uint8_t calculatedCRC = crc.getCRC();

      // Verifikasi CRC
      if (receivedCRC == calculatedCRC) {
        Serial.print("Received: ");
        Serial.print(message);
        Serial.println(" | CRC Check: OK");
      } else {
        Serial.print("Received: ");
        Serial.print(message);
        Serial.println(" | CRC Check: FAILED");
      }
    } else {
      Serial.println("Invalid Data Format");
    }
  }
}
