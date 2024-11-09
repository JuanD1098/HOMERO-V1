#ifndef MPU9250SENSOR_H
#define MPU9250SENSOR_H

#include "MPU9250.h"  // Asegura que la clase MPU9250 esté definida
#include <Wire.h>

class MPU9250Sensor {

 private:
    MPU9250 imu;
    int status;
public:
    MPU9250Sensor(uint8_t address = 0x69);
    bool begin();
    void setConfiguration();
    void readData();
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    float getMagX();
    float getMagY();
    float getMagZ();
    float getTemperature();
};

#endif