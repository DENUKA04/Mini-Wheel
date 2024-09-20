#define BLYNK_TEMPLATE_ID "TMPL6WoQZtceZ"
#define BLYNK_TEMPLATE_NAME "Mini Wheel Alignment"
#define BLYNK_AUTH_TOKEN "1_XV_q2QiPJog4ElxM9BLx7aKUP4zAmO"

#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>

// Constants
const uint64_t DEEP_SLEEP_TIME = 0; // Use 0 for infinite deep sleep
const int wakePin = 14; // GPIO16 (D0) for deep sleep wake-up

char auth[] = "1_XV_q2QiPJog4ElxM9BLx7aKUP4zAmO";
char ssid[] = "ADME.GROUP-2.4G_EXT";
char pass[] = "9232adme";

// Create a new sensor object
BMI270 imu;
BlynkTimer timer;

// I2C address selection
//uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68
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

void setup(){

   Serial.begin(115200);
    while (!Serial); // Wait for Serial to initialize

    // Setup wake-up pin and deep sleep
    pinMode(wakePin, INPUT_PULLUP);
    pinMode(buttonPin, INPUT_PULLUP);

    // Check if button is pressed during boot
    if (digitalRead(wakePin) == LOW) {
        unsigned long startTime = millis();
        while (digitalRead(wakePin) == LOW) {
            if (millis() - startTime >= longPressDuration) {
                break; // Button held for 15 seconds, continue boot
            }
        }
        if (millis() - startTime < longPressDuration) {
            // Button not held long enough, go back to deep sleep
            Serial.println("Button not held long enough, going back to deep sleep...");
            ESP.deepSleep(DEEP_SLEEP_TIME);
        }
    }

    Blynk.begin(auth, ssid, pass);
    Wire.begin(4, 5); // SDA: GPIO4 (D2), SCL: GPIO5 (D1)
    Blynk.run();

    // Initialize the I2C library
    Wire.begin();

    // Check if sensor is connected and initialize
    while(imu.beginI2C(i2cAddress) != BMI2_OK)
    {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }

    Serial.println("BMI270 connected!");

    int8_t err = BMI2_OK;

    // Set accelerometer config
    bmi2_sens_config accelConfig;
    accelConfig.type = BMI2_ACCEL;
    accelConfig.cfg.acc.odr = BMI2_ACC_ODR_50HZ;
    accelConfig.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    accelConfig.cfg.acc.range = BMI2_ACC_RANGE_2G;
    err = imu.setConfig(accelConfig);

    // Set gyroscope config
    bmi2_sens_config gyroConfig;
    gyroConfig.type = BMI2_GYRO;
    gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
    gyroConfig.cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.ois_range = BMI2_GYR_OIS_250;
    gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_125;
    gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    err = imu.setConfig(gyroConfig);

    // Check whether the config settings above were valid
    while(err != BMI2_OK)
    {
        if(err == BMI2_E_ACC_INVALID_CFG)
        {
            Serial.println("Accelerometer config not valid!");
        }
        else if(err == BMI2_E_GYRO_INVALID_CFG)
        {
            Serial.println("Gyroscope config not valid!");
        }
        else if(err == BMI2_E_ACC_GYR_INVALID_CFG)
        {
            Serial.println("Both configs not valid!");
        }
        else
        {
            Serial.print("Unknown error: ");
            Serial.println(err);
        }
        delay(1000);
    }

    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(batteryPin, INPUT);
    pinMode(batteryEnablePin, OUTPUT);

    calibrateGyro();

    timer.setInterval(500L, sendSensorData);
    timer.setInterval(1L, updateSensorData);
  

    Serial.println("Configuration valid! Beginning measurements");
    delay(1000);
}

void loop(){
if (!digitalRead(buttonPin)) {
        if (!buttonPressed) {
            buttonPressTime = millis(); // Start timing
            buttonPressed = true;
        } else if (millis() - buttonPressTime >= longPressDuration) {
            // Button pressed for 15 seconds
            Serial.println("Entering deep sleep...");
            ESP.deepSleep(DEEP_SLEEP_TIME);
        }
    } else {
        if (buttonPressed) {
            if (millis() - buttonPressTime < longPressDuration) {
                // Button released before 15 seconds, reset reference angles
                resetReferenceAngles();
            }
            buttonPressed = false;
        }
    }

    timer.run();
    printSensorData();

    // Print 50x per second
    delay(20);    
}

void updateSensorData() {
    // Get measurements from the sensor
    imu.getSensorData();

    // Read gyroscope data
    float gyroX = imu.data.gyroX;
    float gyroY = imu.data.gyroY;
    float gyroZ = imu.data.gyroZ;

    // Check if the device is moving
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

        // Convert gyroscope data to degrees per second
        float gyroX_degPerSec = gyroX / gyroScale;
        float gyroY_degPerSec = gyroY / gyroScale;
        float gyroZ_degPerSec = gyroZ / gyroScale;

        yaw += (gyroZ_degPerSec + prevGyroZ) / 2.0 * dt;
        pitch += (gyroY_degPerSec + prevGyroY) / 2.0 * dt ;
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
        // Get measurements from the sensor
        imu.getSensorData();

        // Read gyroscope data
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

    // Enable the battery sensor
    digitalWrite(batteryEnablePin, HIGH);
    delay(10); // Wait for the sensor to stabilize

    float batteryVoltage = analogRead(batteryPin) * (3.3 / 1023.0); // Assuming 3.3V reference
    int batteryPercentage = convertVoltageToPercentage(batteryVoltage);
    Blynk.virtualWrite(V4, batteryPercentage);

    // Disable the battery sensor to save power
    digitalWrite(batteryEnablePin, LOW);
}

int convertVoltageToPercentage(float voltage) {
    // Assuming battery voltage range is 3.0V (0%) to 4.2V (100%)
    float minVoltage = 3.0;
    float maxVoltage = 4.2;

    if (voltage <= minVoltage) {
        return 0;
    } else if (voltage >= maxVoltage) {
        return 100;
    } else {
        return (int)(((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100);
    }
}

void resetReferenceAngles() {
    yaw = 0;
    pitch = 0;
    roll = 0;
    prevGyroX = 0;
    prevGyroY = 0;
    prevGyroZ = 0;
    isStationary = true;
    Serial.println("Reference angles reset. Waiting for movement...");
}
