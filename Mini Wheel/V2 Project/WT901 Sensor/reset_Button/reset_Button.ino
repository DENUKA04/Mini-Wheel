#include <Wire.h>

#define WT901_ADDRESS 0x68  // Change if necessary
#define BUTTON_PIN 4   // Pin connected to the reset button

volatile bool resetRequested = false;

void IRAM_ATTR handleButtonPress() {
    resetRequested = true;
}

void setup() {
    Serial.begin(115200); // Start Serial communication at 115200 baud
    Wire.begin(); // Initialize I2C communication
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING); // Interrupt on button press

    // Initialize the WT901 sensor (if needed, add configuration here)
}

void loop() {
    // Check if the reset button was pressed
    if (resetRequested) {
        Serial.println("Resetting values...");
        // You can add additional reset logic here if needed
        resetRequested = false; // Reset the flag
    }

    // Read gyroscope values from the WT901 sensor
    Wire.beginTransmission(WT901_ADDRESS);
    Wire.write(0x1D); // Address of gyroscope data register (change as needed)
    Wire.endTransmission();
    
    Wire.requestFrom(WT901_ADDRESS, 6); // Request 6 bytes for x, y, z axes
    
    if (Wire.available() == 6) {
        int16_t gyroX = (Wire.read() << 8) | Wire.read(); // Read x-axis
        int16_t gyroY = (Wire.read() << 8) | Wire.read(); // Read y-axis
        int16_t gyroZ = (Wire.read() << 8) | Wire.read(); // Read z-axis
        
        // Print the values to the Serial Monitor
        Serial.print("Gyroscope X: "); Serial.print(gyroX);
        Serial.print(" | Y: "); Serial.print(gyroY);
        Serial.print(" | Z: "); Serial.println(gyroZ);
    }

    delay(500); // Delay for readability
}
