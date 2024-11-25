#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

class BatteryReader {
  private:
    int pinADC;            // Pin ADC donde se conecta el divisor de tensión
    float referenceVoltage; // Voltaje de referencia del ESP32
    float adcMaxValue;      // Máximo valor de lectura del ADC
    float resistor1;        // Resistencia superior del divisor
    float resistor2;        // Resistencia inferior del divisor
    float calibrationOffset; // Offset de calibración para precisión

  public:
    // Constructor: inicializa los valores del divisor y configuración del ADC
    BatteryReader(int pin, float r1, float r2, float refVoltage = 3.3, float adcMax = 4095.0, float offset = 0.0);

    // Configura el offset de calibración
    void setCalibrationOffset(float offset);

    // Lee el voltaje de la batería
    float readBatteryVoltage();
};

#endif
