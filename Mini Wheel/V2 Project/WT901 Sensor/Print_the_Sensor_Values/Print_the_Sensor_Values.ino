#include <Wire.h>

// Define the I2C address for the WT901 sensor
#define WT901_ADDRESS 0x68  // Change if necessary

void setup() {
    Serial.begin(115200); // Start Serial communication at 115200 baud
    Wire.begin(); // Initialize I2C communication

    // Initialize the WT901 sensor (if needed, add configuration here)
    // Example: Wire.beginTransmission(WT901_ADDRESS);
    // Wire.write(0x00); // Register to configure
    // Wire.write(0x01); // Value to set
    // Wire.endTransmission();
}

void loop() {
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
