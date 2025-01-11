void setup() {  
    Serial.begin(9600);  
}  
  
void loop() {  
    if (Serial.available() > 0) {  
        int inputValue = Serial.parseInt(); // Read an integer from the serial input  
  
        // Use map() to scale the input value  
        int mappedValue = map(inputValue, 0, 100, 32, 10);  
  
        // Check if the input value is within the range  
        if (inputValue >= 0 && inputValue <= 100) {  
            Serial.print("Input: ");  
            Serial.print(inputValue);  
            Serial.print(", Mapped Value: ");  
            Serial.println(mappedValue);  
        } else {  
            Serial.print("Input: ");  
            Serial.print(inputValue);  
            Serial.println(" is out of range.");  
        }  
    }  
}  
