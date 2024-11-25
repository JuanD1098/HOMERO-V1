#include "MPU6050.h"
#include <cmath> // Para usar M_PI

// Constructor con dirección I2C personalizada
MPU6050Sensor::MPU6050Sensor(uint8_t address) : i2cAddress(address) {}

// Inicializa el MPU6050 con la dirección I2C
bool MPU6050Sensor::begin() {
    if (!mpu.begin(i2cAddress)) {
        return false; // Falló la inicialización
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG); // Rango de ±250°/s
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Filtro para reducir ruido
    return true; // Inicialización exitosa
}

// Lee los datos del sensor
bool MPU6050Sensor::readData() {
    mpu.getEvent(&accel, &gyro, &temp); // Captura los datos del sensor
    return true;
}

// Obtiene los valores de aceleración
void MPU6050Sensor::getAcceleration(float &x, float &y, float &z) {
    x = accel.acceleration.x;
    y = accel.acceleration.y;
    z = accel.acceleration.z;
}

// Obtiene los valores del giroscopio en grados/segundo
void MPU6050Sensor::getGyroscope(float &x, float &y, float &z) {
    x = radiansToDegrees(gyro.gyro.x); // Convertir de rad/s a °/s
    y = radiansToDegrees(gyro.gyro.y);
    z = radiansToDegrees(gyro.gyro.z);
}

// Convierte radianes a grados
float MPU6050Sensor::radiansToDegrees(float radians) {
    return radians * (180.0 / M_PI); // Fórmula de conversión
}

// Obtiene la temperatura en grados Celsius
float MPU6050Sensor::getTemperature() {
    return temp.temperature;
}
