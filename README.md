Creating a drone that can be controlled via keyboard keys using an Arduino Uno requires several components and a good understanding of serial communication. Here's an example of how you can set this up.

Components Needed
Arduino Uno
MPU6050 (Gyroscope and Accelerometer)
4 Brushless DC Motors (with ESCs)
4 Propellers
Li-Po Battery
Motor Driver (ESCs)
Frame and other mechanical parts for the drone
USB cable for connecting Arduino to a computer
Software Requirements
Arduino IDE
Python for sending keyboard commands to the Arduino via serial communication
Wiring Overview
Connect the MPU6050 to the Arduino using I2C (SDA to A4, SCL to A5).
Connect the ESCs to the digital pins of the Arduino (e.g., D3, D5, D6, D9).
Arduino Code
The Arduino code will receive commands via serial communication and control the motors accordingly.

cpp
Copy code
#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

MPU6050 mpu;
Servo esc1, esc2, esc3, esc4;

const int escPin1 = 3;
const int escPin2 = 5;
const int escPin3 = 6;
const int escPin4 = 9;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

int motor1Speed = 1000;
int motor2Speed = 1000;
int motor3Speed = 1000;
int motor4Speed = 1000;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  esc1.attach(escPin1);
  esc2.attach(escPin2);
  esc3.attach(escPin3);
  esc4.attach(escPin4);

  initializeESCs();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    handleCommand(command);
  }
  
  updateIMU();
  
  esc1.writeMicroseconds(motor1Speed);
  esc2.writeMicroseconds(motor2Speed);
  esc3.writeMicroseconds(motor3Speed);
  esc4.writeMicroseconds(motor4Speed);
}

void handleCommand(char command) {
  switch (command) {
    case 'w': // Increase throttle
      motor1Speed += 10;
      motor2Speed += 10;
      motor3Speed += 10;
      motor4Speed += 10;
      break;
    case 's': // Decrease throttle
      motor1Speed -= 10;
      motor2Speed -= 10;
      motor3Speed -= 10;
      motor4Speed -= 10;
      break;
    case 'a': // Roll left
      motor1Speed -= 10;
      motor2Speed += 10;
      motor3Speed += 10;
      motor4Speed -= 10;
      break;
    case 'd': // Roll right
      motor1Speed += 10;
      motor2Speed -= 10;
      motor3Speed -= 10;
      motor4Speed += 10;
      break;
    case 'i': // Pitch forward
      motor1Speed -= 10;
      motor2Speed -= 10;
      motor3Speed += 10;
      motor4Speed += 10;
      break;
    case 'k': // Pitch backward
      motor1Speed += 10;
      motor2Speed += 10;
      motor3Speed -= 10;
      motor4Speed -= 10;
      break;
  }
  
  motor1Speed = constrain(motor1Speed, 1000, 2000);
  motor2Speed = constrain(motor2Speed, 1000, 2000);
  motor3Speed = constrain(motor3Speed, 1000, 2000);
  motor4Speed = constrain(motor4Speed, 1000, 2000);
}

void updateIMU() {
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
}

void initializeESCs() {
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(7000);
}
Python Code for Sending Keyboard Commands
To control the drone using keyboard keys, you'll need a Python script that reads keyboard input and sends corresponding commands to the Arduino via serial communication.

Install the pyserial library if you haven't already:

bash
Copy code
pip install pyserial
Here's a basic Python script to send keyboard commands to the Arduino:

python
Copy code
import serial
import keyboard

arduino = serial.Serial('COM3', 9600)  # Adjust 'COM3' to your Arduino's COM port

def send_command(command):
    arduino.write(command.encode())

def main():
    while True:
        if keyboard.is_pressed('w'):
            send_command('w')
        elif keyboard.is_pressed('s'):
            send_command('s')
        elif keyboard.is_pressed('a'):
            send_command('a')
        elif keyboard.is_pressed('d'):
            send_command('d')
        elif keyboard.is_pressed('i'):
            send_command('i')
        elif keyboard.is_pressed('k'):
            send_command('k')

if __name__ == "__main__":
    main()
Explanation
Arduino Code:
Initialization: The MPU6050 and ESCs are initialized.
Loop: The main loop reads serial commands and adjusts motor speeds based on the received command.
Python Code:
Keyboard Input: The script continuously checks for specific key presses and sends the corresponding command to the Arduino.
Safety Note
Building and flying a drone involves risks. Ensure all components are properly secured and test the system thoroughly in a controlled environment. Always follow safety protocols, especially with Li-Po batteries and spinning propellers.
