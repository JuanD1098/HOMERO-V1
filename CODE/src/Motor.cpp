//Motor.cpp

#include "Motor.h"

// Frecuencia de PWM en Hz (20 kHz) y resolución de 8 bits (0-255)
int pwmFreq = 8000;       // Alta frecuencia para reducir ruido del motor
int pwmResolution = 8;     // Resolución de 8 bits para el PWM
int pwmFreq2 = 5000; 

// Constructor de la clase Motor
// Inicializa los pines y canales PWM para el control de dirección y velocidad
Motor::Motor(int pinA, int pinB, int channelA, int channelB)
    : inPinA(pinA), inPinB(pinB), pwmChannelA(channelA), pwmChannelB(channelB) {

    // Configuración de los pines del puente H como salida
    pinMode(inPinA, OUTPUT);
    pinMode(inPinB, OUTPUT);

    // Configura los canales PWM para inPinA y inPinB del puente H
    ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
    ledcSetup(pwmChannelB, pwmFreq, pwmResolution);

    // Asignar pines a los canales PWM correspondientes
    ledcAttachPin(inPinA, pwmChannelA);  // Canal PWM para inPinA
    ledcAttachPin(inPinB, pwmChannelB);  // Canal PWM para inPinB
}

// Constructor de la clase RMotor
RMotor::RMotor(int rPin, int rChannel) 
    : rodilloPin(rPin), rodilloChannel(rChannel) {

    pinMode(rodilloPin, OUTPUT);

    // Cambia el canal PWM para el rodillo
    ledcSetup(rodilloChannel, pwmFreq2, pwmResolution);
    ledcAttachPin(rodilloPin, rodilloChannel);

}

// Constructor de la clase ESC
ESC::ESC(int EPin) : ESCPin(EPin) {}

// Configura el ESC con el rango de pulsos
void ESC::attach(int minPulse, int maxPulse) {
    esc.attach(ESCPin, minPulse, maxPulse);
}

// Control de velocidad del ESC
void ESC::speedESC(int speed) {
    if (speed > 0) {
        int escSignal = map(speed, 0, 255, 1000, 2000);  // Convierte a rango de pulsos
        esc.writeMicroseconds(escSignal);                  // Enviar señal PWM
    } else {
        esc.writeMicroseconds(1000);  // Apagar ESC con señal mínima
    }
}


void Motor::setSpeed(int speed) {
    if (speed > 0) {
        // Movimiento en una dirección: PWM en inPinA, inPinB en bajo
        ledcWrite(pwmChannelA, speed);  // PWM en inPinA
        ledcWrite(pwmChannelB, 0);      // inPinB en bajo
    }
    else if (speed < 0) {
        // Movimiento en la dirección opuesta: PWM en inPinB, inPinA en bajo
        ledcWrite(pwmChannelA, 0);           // inPinA en bajo
        ledcWrite(pwmChannelB, -speed);      // PWM en inPinB
    }
    else {
        // Parar el motor: ambos pines en bajo
        ledcWrite(pwmChannelA, 0);
        ledcWrite(pwmChannelB, 0);
    }
}

void RMotor::speedrodillo(int speed) {
    if (speed > -250 ) {
         int speedM= map(speed, -255, 255, 0, 255);
    // Aplica el valor absoluto del PWM al canal del rodillo
    ledcWrite(rodilloChannel, speedM);  // PWM directo en el canal del rodillo
} 
else { 
    ledcWrite(rodilloChannel, 0);
}

}

