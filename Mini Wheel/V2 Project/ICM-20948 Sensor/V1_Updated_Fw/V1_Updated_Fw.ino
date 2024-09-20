#define BLYNK_TEMPLATE_ID "TMPL6WoQZtceZ"
#define BLYNK_TEMPLATE_NAME "Mini Wheel Alignment"
#define BLYNK_AUTH_TOKEN "1_XV_q2QiPJog4ElxM9BLx7aKUP4zAmO"

#include <Wire.h>
#include <ICM_20948.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>

// Constants
const int wakePin = 14; // Pin for wake-up button
const int ledPin = 16;
const int buttonPin = 14; // Pin for reset button
const int batteryPin = A0; // Pin for battery voltage sensor
const int batteryEnablePin = 16; // Pin to enable battery sensor

char auth[] = "1_XV_q2QiPJog4ElxM9BLx7aKUP4zAmO";
char ssid[] = "ADME.GROUP-2.4G_EXT";
char pass[] = "9232adme";

ICM_20948_I2C icm; // Create ICM-20948 sensor object
BlynkTimer timer;

float yaw = 0, pitch = 0, roll = 0;
float prevGyroX = 0, prevGyroY = 0, prevGyroZ = 0;
float gyroScale = 100.0; // Adjust as needed
bool isStationary = true;
const float movementThreshold = 0.05; // Adjust as needed

bool buttonPressed = false;
unsigned long buttonPressTime = 0;
const unsigned long longPressDuration = 1000; // 10 seconds

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22); // SDA: GPIO21, SCL: GPIO22 for ESP32 Mini
    
    pinMode(ledPin, OUTPUT);
    pinMode(wakePin, INPUT_PULLUP);
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(batteryPin, INPUT);
    pinMode(batteryEnablePin, OUTPUT);

    Blynk.begin(auth, ssid, pass);

    // Initialize ICM-20948
    icm.begin(Wire, 0x68); // I2C address 0x68
    if (icm.status != ICM_20948_Stat_Ok) {
        Serial.println("ICM-20948 not detected. Check connections.");
        while (1);
    }
    Serial.println("ICM-20948 connected!");

    calibrateGyro();
    timer.setInterval(500L, sendSensorData);
    timer.setInterval(20L, updateSensorData);
}

void loop() {
    if (!digitalRead(buttonPin)) {
        if (!buttonPressed) {
            buttonPressTime = millis(); // Start timing
            buttonPressed = true;
        } else if (millis() - buttonPressTime >= longPressDuration) {
            // Button pressed for 10 seconds
            Serial.println("Entering deep sleep...");
            esp_sleep_enable_ext0_wakeup(GPIO_NUM_14, 0); // Wake up on button press
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

void updateSensorData() {
    icm.getAGMT(); // Read accelerometer, gyroscope, magnetometer, and temperature data

    float gyroX = icm.gyrX();
    float gyroY = icm.gyrY();
    float gyroZ = icm.gyrZ();

    if (abs(gyroX) > movementThreshold || abs(gyroY) > movementThreshold || abs(gyroZ) > movementThreshold) {
        isStationary = false;
    }

    if (isStationary) {
        prevGyroX = 0;
        prevGyroY = 0;
        prevGyroZ = 0;
        yaw = 0;
        pitch = 0;
        roll = 0;
    } else {
        float dt = 3.7;

        float gyroX_degPerSec = gyroX / gyroScale;
        float gyroY_degPerSec = gyroY / gyroScale;
        float gyroZ_degPerSec = gyroZ / gyroScale;

        yaw += (gyroZ_degPerSec + prevGyroZ) / 2.0 * dt;
        pitch += (gyroY_degPerSec + prevGyroY) / 2.0 * dt;
        roll += (gyroX_degPerSec + prevGyroX) / 2.0 * dt;

        yaw = yaw + 0.000099;
        
        prevGyroX = gyroX_degPerSec;
        prevGyroY = gyroY_degPerSec;
        prevGyroZ = gyroZ_degPerSec;
    }
}

void calibrateGyro() {
    Serial.println("Calibrating gyroscope. Please keep the sensor stationary...");
    delay(500);

    int sampleCount = 0;
    float gyroX_bias = 0, gyroY_bias = 0, gyroZ_bias = 0;

    while (sampleCount < 1000) { // Calibration for 1000 samples
        icm.getAGMT(); // Read gyroscope data
        gyroX_bias += icm.gyrX();
        gyroY_bias += icm.gyrY();
        gyroZ_bias += icm.gyrZ();
        sampleCount++;
        delay(10);
    }

    gyroX_bias /= 1000;
    gyroY_bias /= 1000;
    gyroZ_bias /= 1000;

    Serial.println("Gyroscope calibration complete.");
    Serial.print("GyroX bias: ");
    Serial.print(gyroX_bias);
    Serial.print("\tGyroY bias: ");
    Serial.print(gyroY_bias);
    Serial.print("\tGyroZ bias: ");
    Serial.println(gyroZ_bias);
}

void sendSensorData() {
    Blynk.virtualWrite(V1, yaw);
    Blynk.virtualWrite(V2, pitch);
    Blynk.virtualWrite(V3, roll);

    digitalWrite(batteryEnablePin, HIGH);
    delay(10); // Wait for the sensor to stabilize
    float batteryVoltage = analogRead(batteryPin) * (3.3 / 4095); // Adjust for ESP32's 12-bit ADC
    Blynk.virtualWrite(V4, batteryVoltage);
    digitalWrite(batteryEnablePin, LOW);
}

void resetReferenceAngles() {
    yaw = 0;
    pitch = 0;
    roll = 0;
    Serial.println("Reference angles reset.");
}
