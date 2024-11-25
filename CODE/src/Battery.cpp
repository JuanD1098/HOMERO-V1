#include "Battery.h"

// Constructor
BatteryReader::BatteryReader(int pin, float r1, float r2, float refVoltage, float adcMax, float offset)
    : pinADC(pin), resistor1(r1), resistor2(r2), referenceVoltage(refVoltage), adcMaxValue(adcMax), calibrationOffset(offset) {
    pinMode(pinADC, INPUT); // Configura el pin como entrada
}

// Configura el offset de calibración
void BatteryReader::setCalibrationOffset(float offset) {
    calibrationOffset = offset;
}

// Lee el voltaje de la batería
float BatteryReader::readBatteryVoltage() {
    int adcValue = analogRead(pinADC); // Lee el valor del ADC
    float voltageADC = (adcValue / adcMaxValue) * referenceVoltage; // Convierte ADC a voltaje

    // Calcula el voltaje de la batería usando la relación del divisor
    float divisionFactor = (resistor1 + resistor2) / resistor2;
    float batteryVoltage = voltageADC * divisionFactor + calibrationOffset; // Aplicar factor de divisor y offset

    return batteryVoltage;
}
