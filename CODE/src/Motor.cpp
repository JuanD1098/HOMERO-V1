//Motor.cpp

#include "Motor.h"

// Frecuencia de PWM en Hz (20 kHz) y resolución de 8 bits (0-255)
int pwmFreq = 20000;       // Alta frecuencia para reducir ruido del motor
int pwmResolution = 8;     // Resolución de 8 bits para el PWM

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
RMotor::RMotor(int rPin, int rChannel) : rodilloPin(rPin), rodilloChannel(rChannel) {
    // Configuración del pin del rodillo como salida
    pinMode(rodilloPin, OUTPUT);

    // Configura el canal PWM para el rodillo (TIP122)
    ledcSetup(rodilloChannel, pwmFreq, pwmResolution);
    ledcAttachPin(rodilloPin, rodilloChannel);  // Canal PWM para el rodillo
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
    if (speed > 0) {
    // Aplica el valor absoluto del PWM al canal del rodillo
    ledcWrite(rodilloChannel, abs(speed));  // PWM directo en el canal del rodillo
} 
else { 
    ledcWrite(rodilloChannel, 0);
}

}
