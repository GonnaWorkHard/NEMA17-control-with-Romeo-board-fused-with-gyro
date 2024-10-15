Overview
This project demonstrates the integration of an MPU6050 gyroscope sensor with an Arduino-based stepper motor control system. The system performs two primary tasks:

Control of a stepper motor based on user input.
Reading and plotting gyroscope data from the MPU6050 sensor.
The motor can be rotated based on user input through the Serial Monitor, and it also homes itself using a limit switch during setup. The gyroscope data is periodically read to compute and display rotation angles over time.

Components Used
Arduino Micro (or any compatible Arduino board)
NEMA17 Stepper Motor
MX1508 H-Bridge Driver
MPU6050 Gyroscope & Accelerometer Sensor
Limit Switch (for homing the stepper motor)
24V Power Supply (or appropriate power source for your motor)
Wires 
Pin Assignments
Component	Arduino Pin
Stepper Motor (E1)	6
Stepper Motor (E2)	5
Stepper Motor (M1)	7
Stepper Motor (M2)	4
Limit Switch	8
Motor Control Pin	9
MPU6050 (I2C SDA)	A4 (or SDA pin)
MPU6050 (I2C SCL)	A5 (or SCL pin)
Code Functionalities
1. Stepper Motor Control
The Stepper library is used to control the stepper motor.
The motor homes itself using the limit switch at startup.
The motor can be rotated by entering the number of steps through the Serial Monitor. Positive steps rotate the motor clockwise and negative steps rotate it counterclockwise.
2. MPU6050 Gyroscope Sensor
The MPU6050 gyroscope is initialized using the I2Cdev library.
The initial offsets for calibration are captured during setup.
Gyroscope readings (gx, gy, and gz) are integrated over time to compute the rotation angles (angleX, angleY, angleZ).
3. Serial Communication for Monitoring and Input
The system prompts the user to input step values for motor control.
Gyroscope readings are printed every 100ms for monitoring.
How to Use
Upload the code to your Arduino board.
Open the Serial Monitor (9600 baud rate).
The motor will automatically move to its home position using the limit switch.
Enter step values in the Serial Monitor to rotate the motor. For example:
Enter 200 for one full clockwise rotation (assuming 200 steps per revolution).
Enter -100 for half a counterclockwise rotation.
Monitor the angles from the gyroscope in the Serial Monitor, which will be updated every 100ms.
Libraries Required
Stepper.h: Controls the stepper motor.
I2Cdev.h: Provides I2C communication for MPU6050.
MPU6050.h: Manages MPU6050 sensor operations.
Make sure to install these libraries if they are not already available.

Circuit Diagram
Stepper Motor Driver (MX1508): Connect E1, E2, M1, and M2 to the Arduino pins as per the code.
MPU6050 Sensor: Connect SDA to A4 and SCL to A5 (or the corresponding I2C pins on your Arduino board).
Limit Switch: Connect one terminal to pin 8 and the other to ground.
Motor Power Pin: Connect pin 9 to enable/disable motor operation.
