#include <Wire.h>

#define WT901_ADDRESS 0x68  // Change if necessary

const gpio_num_t BUTTON_PIN = (gpio_num_t)4; // Pin connected to the reset button

volatile bool resetRequested = false;
volatile bool sleepMode = false;
unsigned long buttonPressStart = 0;

void IRAM_ATTR handleButtonPress() {
    if (sleepMode) {
        // Wake up from sleep
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
        resetRequested = true; // Set the flag to wake up
    } else {
        buttonPressStart = millis(); // Start timing the button press
    }
}

void setup() {
    Serial.begin(115200); // Start Serial communication at 115200 baud
    Wire.begin(); // Initialize I2C communication

    pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING); // Interrupt on button press

    // Initialize the WT901 sensor (if needed, add configuration here)
}

void loop() {
    if (resetRequested) {
        if (sleepMode) {
            Serial.println("Waking up...");
            sleepMode = false; // Reset sleep mode flag
            delay(100); // Brief delay to ensure stability
        } else {
            Serial.println("Resetting values...");
            // You can add additional reset logic here if needed
        }
        resetRequested = false; // Reset the flag
    }

    // Check if the button has been held for 10 seconds
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (!sleepMode && (millis() - buttonPressStart >= 10000)) {
            Serial.println("Entering sleep mode...");
            sleepMode = true;
            esp_sleep_enable_ext0_wakeup(BUTTON_PIN, HIGH); // Wake up on button release
            esp_deep_sleep_start(); // Enter deep sleep mode
        }
    } else {
        // Reset the timer if the button is released
        buttonPressStart = 0;
    }

    if (!sleepMode) {
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
}
