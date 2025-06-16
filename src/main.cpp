#include <Arduino.h>
#include "gyro.h"

MPU6050_data_t sensor_data; // Declare a struct to hold sensor data

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    Serial.println("Initializing MPU6050...");
    initMPU6050();
    initOrientation(&sensor_data);
    Serial.println("MPU6050 initialized!");
}

void loop()
{
    // Update sensor data
    updateOrientation(&sensor_data);
    calculateInclinationAngles(&sensor_data);

    // Get current face
    current_face = getCubeFace(sensor_data.roll, sensor_data.pitch);

    // Print orientation data
    Serial.print("Roll: ");
    Serial.print(sensor_data.roll);
    Serial.print("°\t Pitch: ");
    Serial.print(sensor_data.pitch);
    Serial.print("°\t Face: ");

    // Print face name
    switch (current_face)
    {
    case FACE_Z_POS:
        Serial.println("Z+ (Top)");
        break;
    case FACE_Z_NEG:
        Serial.println("Z- (Bottom)");
        break;
    case FACE_X_POS:
        Serial.println("X+ (Front)");
        break;
    case FACE_X_NEG:
        Serial.println("X- (Back)");
        break;
    case FACE_Y_POS:
        Serial.println("Y+ (Right)");
        break;
    case FACE_Y_NEG:
        Serial.println("Y- (Left)");
        break;
    default:
        Serial.println("Unknown");
        break;
    }

    delay(100); // Small delay to make the output readable
}
