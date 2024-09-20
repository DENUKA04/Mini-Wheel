#define BLYNK_TEMPLATE_ID "TMPL6WoQZtceZ"
#define BLYNK_TEMPLATE_NAME "Mini Wheel Alignment"
#define BLYNK_AUTH_TOKEN "1_XV_q2QiPJog4ElxM9BLx7aKUP4zAmO"

#include <HardwareSerial.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>

// WT901 Communication
HardwareSerial wt901Serial(1); // Use Serial1 for WT901 (TX: GPIO 17, RX: GPIO 16)
#define BAUD_RATE 9600 // WT901's default baud rate

// Constants
const int wakePin = 14; // Pin for wake-up button
const int ledPin = 16;
const int buttonPin = 14; // Pin for reset button
const int batteryPin = 35; // Pin for battery voltage sensor
const int batteryEnablePin = 32; // Pin to enable battery sensor

char auth[] = "1_XV_q2QiPJog4ElxM9BLx7aKUP4zAmO";
char ssid[] = "ADME.GROUP-2.4G_EXT";
char pass[] = "9232adme";

BlynkTimer timer;

float yaw = 0, pitch = 0, roll = 0;
bool buttonPressed = false;
unsigned long buttonPressTime = 0;
const unsigned long longPressDuration = 10000; // 10 seconds

void setup() {
    Serial.begin(115200);
    wt901Serial.begin(BAUD_RATE, SERIAL_8N1, 16, 17); // Start communication with WT901
    pinMode(ledPin, OUTPUT);
    pinMode(wakePin, INPUT_PULLUP);
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(batteryPin, INPUT);
    pinMode(batteryEnablePin, OUTPUT);

    Blynk.begin(auth, ssid, pass);
    
    timer.setInterval(500L, sendSensorData);
    timer.setInterval(20L, readWT901Data);
}

void loop() {
    if (!digitalRead(buttonPin)) {
        if (!buttonPressed) {
            buttonPressTime = millis(); // Start timing
            buttonPressed = true;
        } else if (millis() - buttonPressTime >= longPressDuration) {
            // Button pressed for 10 seconds
            Serial.println("Entering light sleep...");
            WiFi.mode(WIFI_OFF);
            esp_sleep_enable_timer_wakeup(10 * 1000000); // Sleep for 10 seconds
            esp_deep_sleep_start();
        }
    } else {
        if (buttonPressed) {
            if (millis() - buttonPressTime < longPressDuration) {
                resetReferenceAngles(); // Button released before 10 seconds, reset reference angles
            }
            buttonPressed = false;
        }
    }

    timer.run();
    Blynk.run();
}

void readWT901Data() {
    if (wt901Serial.available() > 0) {
        uint8_t buffer[11];
        wt901Serial.readBytes(buffer, 11);
        
        if (buffer[0] == 0x55 && buffer[1] == 0x53) { // Data identifier for angle data
            yaw = ((int16_t)(buffer[7] << 8 | buffer[6])) / 32768.0 * 180.0;
            pitch = ((int16_t)(buffer[5] << 8 | buffer[4])) / 32768.0 * 180.0;
            roll = ((int16_t)(buffer[3] << 8 | buffer[2])) / 32768.0 * 180.0;
        }
    }
}

void sendSensorData() {
    Blynk.virtualWrite(V1, yaw);
    Blynk.virtualWrite(V2, pitch);
    Blynk.virtualWrite(V3, roll);

    digitalWrite(batteryEnablePin, HIGH);
    delay(10); // Wait for the sensor to stabilize
    float batteryVoltage = analogRead(batteryPin) * (3.3 / 4095); // Adjust for ESP32 ADC resolution
    Blynk.virtualWrite(V4, batteryVoltage);
    digitalWrite(batteryEnablePin, LOW);
}

void resetReferenceAngles() {
    yaw = 0;
    pitch = 0;
    roll = 0;
    Serial.println("Reference angles reset.");
}
