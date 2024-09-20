#define BLYNK_TEMPLATE_ID "TMPL6WoQZtceZ"
#define BLYNK_TEMPLATE_NAME "Mini Wheel Alignment"
#define BLYNK_AUTH_TOKEN "1_XV_q2QiPJog4ElxM9BLx7aKUP4zAmO"

#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>
#include <esp8266_peri.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFi.h>

// Constants
const int wakePin = 14; // Pin for wake-up button
const int ledPin = 16;

char auth[] = "1_XV_q2QiPJog4ElxM9BLx7aKUP4zAmO";
char ssid[] = "ADME.GROUP-2.4G_EXT";
char pass[] = "9232adme";

// Create a new sensor object
BMI270 imu;
BlynkTimer timer;

// I2C address selection
uint8_t i2cAddress = BMI2_I2C_SEC_ADDR; // 0x69

const int buttonPin = 14; // Pin for reset button
const int batteryPin = A0; // Pin for battery voltage sensor
const int batteryEnablePin = 16; // Pin to enable battery sensor

float prevGyroX = 0, prevGyroY = 0, prevGyroZ = 0;
float yaw = 0, pitch = 0, roll = 0;
int calibrationSamples = 1000;
float gyroX_bias = 0, gyroY_bias = 0, gyroZ_bias = 0;
float gyroScale = 100.0; // Adjust as needed

bool isStationary = true;
const float movementThreshold = 0.05; // Adjust as needed

unsigned long buttonPressTime = 0;
bool buttonPressed = false;
const unsigned long longPressDuration = 1000; // 10 seconds

void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for Serial to initialize

    pinMode(ledPin, OUTPUT);
    pinMode(wakePin, INPUT_PULLUP);
    pinMode(buttonPin, INPUT_PULLUP);

    Blynk.begin(auth, ssid, pass);
    Wire.begin(4, 5); // SDA: GPIO4 (D2), SCL: GPIO5 (D1)
    Blynk.run();

    Wire.begin();

    while (imu.beginI2C(i2cAddress) != BMI2_OK) {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }

    Serial.println("BMI270 connected!");

    // Sensor configuration code...

    pinMode(batteryPin, INPUT);
    pinMode(batteryEnablePin, OUTPUT);

    calibrateGyro();

    timer.setInterval(500L, sendSensorData);
    timer.setInterval(1L, updateSensorData);

    Serial.println("Configuration valid! Beginning measurements");
    delay(1000);
}

void loop() {
    if (!digitalRead(buttonPin)) {
        if (!buttonPressed) {
            buttonPressTime = millis(); // Start timing
            buttonPressed = true;
        } else if (millis() - buttonPressTime >= longPressDuration) {
            // Button pressed for long duration, enter deep sleep
            Serial.println("Entering deep sleep...");
            ESP.deepSleep(0); // Deep sleep indefinitely, wakes up when the wakePin is pressed
        }
    } else {
        if (buttonPressed) {
            if (millis() - buttonPressTime < longPressDuration) {
                // Button released before long duration, reset reference angles
                resetReferenceAngles();
            }
            buttonPressed = false;
        }
    }

    timer.run();
    printSensorData();

    delay(20);    
}

void updateSensorData() {
    imu.getSensorData();

    float gyroX = imu.data.gyroX;
    float gyroY = imu.data.gyroY;
    float gyroZ = imu.data.gyroZ;

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
        gyroX -= gyroX_bias;
        gyroY -= gyroY_bias;
        gyroZ -= gyroZ_bias;

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
    gyroX_bias = 0;
    gyroY_bias = 0;
    gyroZ_bias = 0;

    while (sampleCount < calibrationSamples) {
        imu.getSensorData();

        gyroX_bias += imu.data.gyroX;
        gyroY_bias += imu.data.gyroY;
        gyroZ_bias += imu.data.gyroZ;

        sampleCount++;
        delay(10);
    }

    gyroX_bias /= calibrationSamples;
    gyroY_bias /= calibrationSamples;
    gyroZ_bias /= calibrationSamples;

    Serial.println("Gyroscope calibration complete.");
    Serial.print("GyroX bias: ");
    Serial.print(gyroX_bias);
    Serial.print("\tGyroY bias: ");
    Serial.print(gyroY_bias);
    Serial.print("\tGyroZ bias: ");
    Serial.println(gyroZ_bias);
}

void printSensorData() {
    Serial.print("Yaw: ");
    Serial.print(yaw);
    Serial.print("\tPitch: ");
    Serial.print(pitch);
    Serial.print("\tRoll: ");
    Serial.println(roll);
}

void sendSensorData() {
    Blynk.run();

    Blynk.virtualWrite(V1, yaw);
    Blynk.virtualWrite(V2, pitch);
    Blynk.virtualWrite(V3, roll);

    digitalWrite(batteryEnablePin, HIGH);
    delay(10); // Wait for the sensor to stabilize

    float batteryVoltage = analogRead(batteryPin) * (3.3 / 1024);
    Blynk.virtualWrite(V4, batteryVoltage);
    digitalWrite(batteryEnablePin, LOW);
}

void resetReferenceAngles() {
    yaw = 0;
    pitch = 0;
    roll = 0;
    Serial.println("Reference angles reset.");
}
