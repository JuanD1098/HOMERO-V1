// FlySkyReceiver.cpp 
#include "FlySkyReceiver.h"


 //Constructor que inicializa el receptor  ESP32
FlySkyReceiver::FlySkyReceiver(HardwareSerial& serial, int timer) {
  IBus.begin(serial, timer);
}



// Método para leer y mapear los valores de los canales
void FlySkyReceiver::AllChannels() {
  for (int ch = 0; ch < 10; ch++) {
    channelValues[ch] = IBus.readChannel(ch);  // Leer el valor del canal actual
    PWM_CH[ch] = map(channelValues[ch], 1000, 2000, -255, 255);
    PWM_CH[ch] = constrain(PWM_CH[ch], -200, 200);  // Asegurar que el valor esté en el rango 0-255
  } 
}   

// Método para imprimir los valores mapeados de los canales en el monitor serial
void FlySkyReceiver::printChannelValues() {
  Serial.print("Valores de los canales: ");
  for (int ch = 0; ch < 10; ch++) {
    Serial.print("PWM"); Serial.print(ch + 1); Serial.print(": ");
    Serial.print(PWM_CH[ch]); Serial.print("  ");
  }
  Serial.println();
}


// Método para obtener el valor PWM de un canal específico
int FlySkyReceiver::getChannelPWM(int channelNumber) {
  if (channelNumber >= 0 && channelNumber < 10) {  // Verificar rango válido
    return PWM_CH[channelNumber-1];
    } 
    else {
    return -1;  // Valor de error si el canal está fuera de rango
  }
}