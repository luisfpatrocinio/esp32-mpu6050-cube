# 🎲 ESP32 MPU6050 Cube Orientation Detector

A simple ESP32 project that uses the MPU6050 accelerometer and gyroscope sensor to detect the orientation of a cube in 3D space. The project reads sensor data and determines which face of the cube is currently facing up.

## ✨ Features

- Real-time orientation detection using MPU6050
- Roll, Pitch, and Yaw angle calculations
- Cube face detection (6 faces: Top, Bottom, Front, Back, Left, Right)
- Serial output for monitoring orientation data
- Complementary filter implementation for stable readings

## 🔧 Hardware Requirements

- ESP32 development board
- MPU6050 sensor
- Jumper wires
- USB cable for programming and power

## 🔌 Wiring

Connect the MPU6050 to the ESP32 as follows:

| MPU6050 | ESP32 |
|---------|-------|
| VCC     | 3.3V  |
| GND     | GND   |
| SDA     | GPIO21|
| SCL     | GPIO22|

## 💻 Software Requirements

- Arduino IDE or PlatformIO
- ESP32 board support
- Wire library (included with Arduino IDE)

## 📥 Installation

1. Clone this repository
2. Open the project in your preferred IDE
3. Install required dependencies
4. Upload the code to your ESP32

## 🚀 Usage

1. Connect the hardware as described in the Wiring section
2. Upload the code to your ESP32
3. Open Serial Monitor at 115200 baud
4. The program will output:
   - Roll angle
   - Pitch angle
   - Current cube face orientation

## 📁 Project Structure

- `src/main.cpp` - Main program file
- `src/gyro.h` - Header file for MPU6050 functions
- `src/gyro.cpp` - Implementation of MPU6050 functions

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Feel free to submit issues and enhancement requests! 