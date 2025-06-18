/**
 * @file gyro.c
 * @brief Implementation file for MPU6050 accelerometer and gyroscope sensor
 * interface.
 *
 * This file implements the functions declared in gyro.h for initializing
 * and reading data from the MPU6050 sensor. It includes the implementation
 * of functions for reading raw accelerometer data and calculating inclination
 * angles.
 *
 * @author Luis Felipe Patrocinio (https://github.com/luisfpatrocinio)
 * Adapted for ESP32
 */
#include "gyro.h"

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static unsigned long last_update_time_us = 0; // Global variable to store the last update time in microseconds
CubeFace_e current_face;                      // Global variable to hold the current face of the cube based on roll and pitch

// Adicionar um filtro de média móvel para o yaw
#define YAW_FILTER_SIZE 5
float yaw_history[YAW_FILTER_SIZE];
int yaw_index = 0;

// Adicionar offset de calibração
float yaw_offset = 0;

#define YAW_THRESHOLD 5.0

/**
 * @brief Initialize the MPU6050 sensor.
 */
void initMPU6050()
{
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    // Wake up MPU6050 - Power Management 1 register
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0x00); // Clear sleep bit
    Wire.endTransmission();
}

/**
 * @brief Update the accelerometer data.
 * @param data Pointer to store the accelerometer data.
 */
void updateAccelerometerData(MPU6050_data_t *data)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Accelerometer data register address (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);

    data->raw_x = (Wire.read() << 8) | Wire.read();
    data->raw_y = (Wire.read() << 8) | Wire.read();
    data->raw_z = (Wire.read() << 8) | Wire.read();
}

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
void updateGyroscopeData(MPU6050_data_t *data)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43); // Gyroscope data register address (GYRO_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);

    data->g_x = (Wire.read() << 8) | Wire.read();
    data->g_y = (Wire.read() << 8) | Wire.read();
    data->g_z = (Wire.read() << 8) | Wire.read();
}

/**
 * @brief Calculates the roll and pitch angles from accelerometer data.
 *
 * This function converts the raw accelerometer readings into acceleration in
 * g's and then uses these values to compute the roll and pitch angles, which
 * represent the inclination of the sensor.
 *
 * @param roll Pointer to store the calculated roll angle in degrees.
 * @param pitch Pointer to store the calculated pitch angle in degrees.
 * @note Roll is rotation around X-axis, Pitch is rotation around Y-axis.
 * @note The angles are calculated using atan2 and are in the range of -180 to
 * 180 degrees.
 * @note The conversion factor ACCEL_FS_SEL_2G_SENSITIVITY (16384.0) is specific
 * to the MPU6050's ±2g range.
 */
void calculateInclinationAngles(MPU6050_data_t *data)
{
    // Convert raw accelerometer values to g's
    data->g_x = data->raw_x / ACCEL_FS_SEL_2G_SENSITIVITY;
    data->g_y = data->raw_y / ACCEL_FS_SEL_2G_SENSITIVITY;
    data->g_z = data->raw_z / ACCEL_FS_SEL_2G_SENSITIVITY;

    // Calculate Roll (rotation around X-axis)
    data->roll = atan2(data->g_y, data->g_z) * (180.0 / M_PI);

    // Calculate Pitch (rotation around Y-axis)
    data->pitch =
        atan2(-data->g_x, sqrt(data->g_y * data->g_y + data->g_z * data->g_z)) *
        (180.0 / M_PI);
}

CubeFace_e getCubeFace(float r, float p)
{
    float pitch_abs = fabsf(p);
    float roll_abs = fabsf(r);
    float threshold_flat =
        30.0f; // Ângulo para considerar "plano" (face Z para cima/baixo)
    float threshold_side =
        70.0f; // Ângulo para considerar "de lado" (outras faces)

    if (pitch_abs < threshold_flat && roll_abs < threshold_flat)
    {
        return FACE_Z_POS; // Face Z+ para cima
    }
    else if (pitch_abs < threshold_flat &&
             roll_abs > (180.0f - threshold_flat))
    {
        return FACE_Z_NEG; // Face Z- para cima (invertido)
    }
    else if (p < -threshold_side)
    {                      // Pitch negativo acentuado
        return FACE_X_POS; // Face X+ para cima (Ex: nariz do avião para cima)
    }
    else if (p > threshold_side)
    {                      // Pitch positivo acentuado
        return FACE_X_NEG; // Face X- para cima (Ex: nariz do avião para baixo)
    }
    else if (r > threshold_side)
    {                      // Roll positivo acentuado
        return FACE_Y_POS; // Face Y+ para cima (Ex: Asa direita para baixo)
    }
    else if (r < -threshold_side)
    {                      // Roll negativo acentuado
        return FACE_Y_NEG; // Face Y- para cima (Ex: Asa esquerda para baixo)
    }
    return FACE_UNKNOWN;
}

void initOrientation(MPU6050_data_t *data)
{
    // Ler dados iniciais para calcular o primeiro roll/pitch do acelerômetro
    updateAccelerometerData(data);
    // converter para g
    data->roll = atan2(data->g_y, data->g_z) * (180.0 / M_PI);
    data->pitch =
        atan2(-data->g_x, sqrt(data->g_y * data->g_y + data->g_z * data->g_z)) *
        (180.0 / M_PI);
    data->yaw = 0.0f;

    last_update_time_us = micros();
}

void updateOrientation(MPU6050_data_t *data)
{
    unsigned long now = micros();
    float dt = (now - last_update_time_us) / 1000000.0f; // dt em segundos
    last_update_time_us = now;

    // Ler Accel e Gyro
    updateAccelerometerData(data);
    updateGyroscopeData(data);

    // Converter para g e dps
    float ax_g = data->raw_x / ACCEL_FS_SEL_2G_SENSITIVITY;
    float ay_g = data->raw_y / ACCEL_FS_SEL_2G_SENSITIVITY;
    float az_g = data->raw_z / ACCEL_FS_SEL_2G_SENSITIVITY;

    float gx_dps = data->g_x / GYRO_FS_SEL_250DPS_SENSITIVITY;
    float gy_dps = data->g_y / GYRO_FS_SEL_250DPS_SENSITIVITY;
    float gz_dps = data->g_z / GYRO_FS_SEL_250DPS_SENSITIVITY;

    float roll_accel = atan2(ay_g, az_g) * (180.0f / M_PI);
    float pitch_accel =
        atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * (180.0f / M_PI);

    // Atualizar roll e pitch com o filtro
    data->roll = ALPHA * (data->roll + gx_dps * dt) + (1.0f - ALPHA) * roll_accel;
    data->pitch =
        ALPHA * (data->pitch + gy_dps * dt) + (1.0f - ALPHA) * pitch_accel;

    // Para Yaw (propenso a drift)
    data->yaw = (atan2(ay_g, ax_g) * RAD_TO_DEG) - yaw_offset;

    // Normalização para valores entre 0 e 360
    float normalized_yaw = data->yaw;
    if (normalized_yaw < 0)
    {
        normalized_yaw += 360;
    }
    int yaw_int = (int)(normalized_yaw / 360 * MAX_ROLL);

    // Adicionar um filtro de média móvel para o yaw
    yaw_history[yaw_index] = data->yaw;
    yaw_index = (yaw_index + 1) % YAW_FILTER_SIZE;

    float filtered_yaw = 0;
    for (int i = 0; i < YAW_FILTER_SIZE; i++)
    {
        filtered_yaw += yaw_history[i];
    }
    data->yaw = filtered_yaw / YAW_FILTER_SIZE;
}