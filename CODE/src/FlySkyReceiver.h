// FlySkyReceiver.h
#ifndef FLYSKY_RECEIVER_H
#define FLYSKY_RECEIVER_H

#include <Arduino.h>
#include <IBusBM.h>

class FlySkyReceiver {
  private:
    IBusBM IBus;             // Objeto IBus para la comunicación iBUS
    int channelValues[10];   // Array para almacenar los valores de los 10 canales
    int PWM_CH[10];          // Array para almacenar los valores PWM de los 10 canales}

  public:
    // Constructor que inicializa el receptor
FlySkyReceiver(HardwareSerial& serial, int timer);

    // Método para leer y mapear los valores de los canales
    void AllChannels();

    // Método para imprimir los valores mapeados de los canales en el monitor serial
    void printChannelValues();


    int getChannelPWM(int channelNumber);

};

#endif // FLYSKY_RECEIVER_H