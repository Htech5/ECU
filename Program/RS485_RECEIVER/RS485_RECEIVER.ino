#define LED 2      // Pin untuk LED
#define RE 4       // Pin untuk Receiver Enable
#define DE 16      // Pin untuk Driver Enable

// Fungsi untuk menghitung CRC-8
uint8_t calculateCRC(uint8_t data) {
  uint8_t crc = 0xFF; // Inisialisasi CRC
  crc ^= data;        // XOR dengan data
  for (int i = 0; i < 8; i++) {
    if (crc & 0x80) { // Jika bit paling signifikan adalah 1
      crc = (crc << 1) ^ 0x31; // Shift left dan XOR dengan polynomial
    } else {
      crc <<= 1; // Shift left
    }
  }
  return crc; // Kembalikan nilai CRC
}

void setup() {
  pinMode(LED, OUTPUT);                        // Declare LED pin as output
  pinMode(RE, OUTPUT);                         // Declare RE pin as output
  pinMode(DE, OUTPUT);                         // Declare DE pin as output
  Serial.begin(9600);                          // Set serial communication baudrate 
  digitalWrite(RE, LOW);                       // Set RE low (Receiving mode ON)
  digitalWrite(DE, LOW);                       // Set DE low (Sending mode OFF)
}

void loop() {
  if (Serial.available() >= 2) {               // Pastikan ada setidaknya 2 byte yang tersedia
    uint8_t data = Serial.read();              // Baca data pertama
    uint8_t receivedCRC = Serial.read();       // Baca CRC yang diterima

    // Hitung CRC dari data yang diterima
    uint8_t calculatedCRC = calculateCRC(data);
    
    // Debugging: Tampilkan nilai CRC yang dihitung dan diterima
    Serial.print("Calculated CRC: ");
    Serial.println(calculatedCRC, HEX); // Tampilkan dalam format heksadesimal
    Serial.print("Received CRC: ");
    Serial.println(receivedCRC, HEX);    // Tampilkan dalam format heksadesimal

    // Periksa apakah data yang diterima adalah 'A' dan CRC cocok
    if (data == 'A' && receivedCRC == calculatedCRC) {
      digitalWrite(LED, !digitalRead(LED)); // Blink LED
      Serial.println("LED Toggled!");        // Tampilkan pesan bahwa LED telah berkedip
    } else {
      Serial.println("Data or CRC mismatch!"); // Tampilkan pesan jika  
}
