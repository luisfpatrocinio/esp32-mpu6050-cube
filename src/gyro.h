/**
 * @file gyro.h
 * @brief Header file for MPU6050 accelerometer and gyroscope sensor interface.
 *
 * This file defines constants, declares global variables, and prototypes
 * functions for initializing and reading data from the MPU6050 sensor. It
 * provides functions for reading raw accelerometer data and calculating
 * inclination angles.
 *
 * @author Luis Felipe Patrocinio (https://github.com/luisfpatrocinio)
 * Adapted for ESP32
 */
#ifndef GYRO_H
#define GYRO_H

#define MAX_ROLL 12
#define MAX_PITCH 12

// Hardware
#include <Arduino.h>
#include <Wire.h> ///< ESP32: I2C interface
#include <math.h> ///< Standard C: Mathematical functions

// --- MPU6050 Configuration Constants ---

#define MPU6050_ADDR 0x68                     ///< I2C address of the MPU6050 sensor.
#define SDA_PIN 21                            ///< GPIO pin for I2C SDA line (ESP32 default)
#define SCL_PIN 22                            ///< GPIO pin for I2C SCL line (ESP32 default)
#define ACCEL_FS_SEL_2G_SENSITIVITY 16384.0f  // LSB/g for ±2g range
#define GYRO_FS_SEL_250DPS_SENSITIVITY 131.0f // LSB/(º/s) for ±250dps
#define ALPHA 0.96f                           // Complementary filter coefficient

// Dados do sensor
typedef struct
{
  int16_t raw_x, raw_y, raw_z;
  float g_x, g_y, g_z;
  float roll, pitch, yaw;
} MPU6050_data_t;

// --- Cube Face Definitions ---
// Enum to represent the cube faces based on roll and pitch angles.
typedef enum
{
  FACE_UNKNOWN,
  FACE_Z_POS,
  FACE_Z_NEG,
  FACE_X_POS,
  FACE_X_NEG,
  FACE_Y_POS,
  FACE_Y_NEG
} CubeFace_e;

// Global variable to hold the current face of the cube based on roll and pitch.
extern CubeFace_e current_face;

/**
 * @brief Determines the cube face based on roll and pitch angles.
 *
 * This function calculates which face of a cube corresponds to the given roll
 * and pitch angles. The angles are expected to be in degrees, where:
 * - Roll (r) is rotation around the X-axis.
 * - Pitch (p) is rotation around the Y-axis.
 *
 * @param r Roll angle in degrees.
 * @param p Pitch angle in degrees.
 * @return CubeFace_e Enum value representing the cube face.
 */
CubeFace_e getCubeFace(float r, float p); // r = roll, p = pitch

// --- Function Prototypes ---

/**
 * @brief Initializes the MPU6050 sensor.
 *
 * This function sends a reset command to the MPU6050's Power Management 1
 * register to wake it up and prepare it for operation. It must be called before
 * any other MPU6050 functions.
 *
 * @note This function assumes I2C has been properly initialized.
 */
void initMPU6050();

/**
 * @brief Reads raw accelerometer data from the MPU6050 sensor.
 *
 * This function reads 6 bytes from the MPU6050, starting from the ACCEL_XOUT_H
 * register (0x3B), which contain the high and low bytes for X, Y, and Z-axis
 * acceleration. The raw 16-bit values are then assembled and stored in the
 * provided MPU6050_data_t structure.
 *
 * @param data Pointer to an MPU6050_data_t structure to store the raw X, Y, and
 * Z-axis acceleration data.
 * @note The raw values are in the range of -32768 to 32767.
 */
void updateAccelerometerData(MPU6050_data_t *data);

/**
 * @brief Reads raw gyroscope data from the MPU6050 sensor.
 *
 * This function reads 6 bytes from the MPU6050, starting from the GYRO_XOUT_H
 * register (0x43), which contain the high and low bytes for X, Y, and Z-axis
 * angular velocity. The raw 16-bit values are then assembled and stored in the
 * provided MPU6050_data_t structure.
 *
 * @param data Pointer to an MPU6050_data_t structure to store the raw gyroscope
 * data.
 * @note The raw values are in the range of -32768 to 32767.
 */
void updateGyroscopeData(MPU6050_data_t *data);

/**
 * @brief Calculates the roll and pitch angles from accelerometer data.
 *
 * This function converts the raw accelerometer readings stored in the provided
 * MPU6050_data_t structure into acceleration in g's and then uses these values
 * to compute the roll and pitch angles, which represent the inclination of the
 * sensor. The calculated angles are stored back into the `roll` and `pitch`
 * members of the same `MPU6050_data_t` structure.
 *
 * @param data Pointer to an MPU6050_data_t structure containing raw
 * accelerometer data and where the calculated roll and pitch angles will be
 * stored.
 * @note Roll is rotation around X-axis, Pitch is rotation around Y-axis.
 * @note The angles are calculated using atan2 and are in the range of -180 to
 * 180 degrees.
 */
void calculateInclinationAngles(MPU6050_data_t *data);

void initOrientation(MPU6050_data_t *data);

void updateOrientation(MPU6050_data_t *data);

#endif // GYRO_H
