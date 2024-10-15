#include <Stepper.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Constants for stepper motor
const int stepsPerRevolution = 200;  // Motor steps per revolution

// Pin assignments for stepper motor control
int E1 = 6;//5;  // Motor 1 Speed Control
int E2 = 5;//6;  // Motor 2 Speed Control
int M1 = 7;//4;  // Motor 1 Direction Control
int M2 = 4;//7;   // Motor 2 Direction Control
int inPin = 8;  // Limit switch pin
int motorPin = 9; 

// Initialize stepper motor
Stepper myStepper(stepsPerRevolution, M1, E2, E1, M2);

// MPU6050 object for gyroscope
MPU6050 accelgyro;

// Gyroscope data and angles
int16_t gx, gy, gz;
float angleX = 0, angleY = 0, angleZ = 0;
float offsetX = 0, offsetY = 0, offsetZ = 0;  // Calibration offsets
bool augerOn = 0;


// Timestamps
unsigned long lastGyroTime = 0;
unsigned long currentMillis = 0;

void setup() {
    // Initialize Serial and I2C communication
    Serial.begin(9600);
    Wire.begin();

    // Initialize the stepper motor
    pinMode(inPin, INPUT_PULLUP);  // Set limit switch pin as input
    myStepper.setSpeed(6);  // Set motor speed to 60 RPM
    
    // Move motor to home position using the limit switch
    while ((digitalRead(inPin) == HIGH) ){
        myStepper.step(-1);  // Step backward until switch is pressed
    }
    Serial.println("Reached limit switch.");
    Serial.println("Enter steps to rotate (positive for clockwise, negative for counterclockwise):");

    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    accelgyro.initialize();
    Serial.println(accelgyro.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");
    // Capture initial offsets for calibration
    int16_t rawGx, rawGy, rawGz;
    accelgyro.getRotation(&rawGx, &rawGy, &rawGz);
    offsetX = rawGx / 131.0;
    offsetY = rawGy / 131.0;
    offsetZ = rawGz / 131.0;

    lastGyroTime = micros();  // Initialize the timestamp
    pinMode(motorPin, OUTPUT);  // Initialize motor pin as an output
    digitalWrite(motorPin, HIGH);
}

void loop() {

    // *** Task 1: Handle Serial Input for Motor Control ***
    if (Serial.available() > 0) {  // Check for user input
        String input = Serial.readStringUntil('\n');  // Read input from Serial Monitor
        int steps = input.toInt();  // Convert input to integer

        if (steps != 0) {  // Check if the input is valid
            if (steps > 0) {
                Serial.print("Rotating Clockwise by ");
            } else {
                Serial.print("Rotating Counterclockwise by ");
            }
            Serial.print(abs(steps));
            Serial.println(" steps.");

            myStepper.step(steps);  // Rotate motor by specified steps
        } else {
            Serial.println("Invalid input. Please enter a valid integer.");
        }

        delay(500);  // Small delay to stabilize
        Serial.println("Enter steps to rotate (positive for clockwise, negative for counterclockwise):");
    }

    // *** Task 2: Read Gyroscope Data Every 10ms ***
    unsigned long currentTime = micros();  // Current timestamp
    if (currentTime - lastGyroTime >= 10000) {  // 10ms interval
        lastGyroTime = currentTime;

        accelgyro.getRotation(&gx, &gy, &gz);  // Read gyroscope data

        // Calculate deltaTime in seconds
        float deltaTime = 0.01;

        // Integrate gyroscope readings to get angles
        angleX += ((gx / 131.0) - offsetX) * deltaTime;
        angleY += ((gy / 131.0) - offsetY) * deltaTime;
        angleZ += ((gz / 131.0) - offsetZ) * deltaTime;
    }

    // *** Task 3: Plot Angles Every 100ms ***
    currentMillis = millis();  // Current time in milliseconds
    if (currentMillis % 100 < 10) {  // Every 100ms
        Serial.print(angleX); Serial.print("\t");
        Serial.print(angleY); Serial.print("\t");
        Serial.println(angleZ);
    }

} 