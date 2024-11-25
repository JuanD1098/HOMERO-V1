#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <Adafruit_MPU6050.h>

class MPU6050Sensor {
  private:
    Adafruit_MPU6050 mpu;       // Instancia del sensor MPU6050
    sensors_event_t accel;      // Estructura para datos del acelerómetro
    sensors_event_t gyro;       // Estructura para datos del giroscopio
    sensors_event_t temp;       // Estructura para datos de temperatura
    uint8_t i2cAddress;         // Dirección I2C del MPU6050

    // Conversión de radianes a grados
    float radiansToDegrees(float radians);

  public:
    MPU6050Sensor(uint8_t address = 0x68); // Constructor con dirección I2C por defecto
    bool begin();                          // Inicializa el sensor
    bool readData();                       // Lee datos del sensor
    void getAcceleration(float &x, float &y, float &z); // Obtiene valores de aceleración
    void getGyroscope(float &x, float &y, float &z);    // Obtiene valores del giroscopio en grados/segundo
    float getTemperature();                // Obtiene la temperatura
};

#endif
