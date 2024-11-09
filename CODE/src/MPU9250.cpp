#include "MPU9250Sensor.h"

MPU9250Sensor::MPU9250Sensor(uint8_t address) : imu(Wire, address), status(0) {}

bool MPU9250Sensor::begin() {
    status = imu.begin();
    return status >= 0;
}

void MPU9250Sensor::setConfiguration() {
    imu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    imu.setSrd(19);
}

void MPU9250Sensor::readData() {
    imu.readSensor();
}

float MPU9250Sensor::getAccelX() { return imu.getAccelX_mss(); }
float MPU9250Sensor::getAccelY() { return imu.getAccelY_mss(); }
float MPU9250Sensor::getAccelZ() { return imu.getAccelZ_mss(); }
float MPU9250Sensor::getGyroX() { return imu.getGyroX_rads(); }
float MPU9250Sensor::getGyroY() { return imu.getGyroY_rads(); }
float MPU9250Sensor::getGyroZ() { return imu.getGyroZ_rads(); }
float MPU9250Sensor::getMagX() { return imu.getMagX_uT(); }
float MPU9250Sensor::getMagY() { return imu.getMagY_uT(); }
float MPU9250Sensor::getMagZ() { return imu.getMagZ_uT(); }
float MPU9250Sensor::getTemperature() { return imu.getTemperature_C(); }
