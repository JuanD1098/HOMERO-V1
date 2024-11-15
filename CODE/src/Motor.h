// Motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
    private:
    int inPinA;             // Pin para la entrada A del puente H
    int inPinB;             // Pin para la entrada B del puente H
    int pwmChannelA;        // Canal PWM para la entrada A
    int pwmChannelB;        // Canal PWM para la entrada B

    public:
    // Constructor de la clase Motor
    Motor(int pinA, int pinB, int channelA, int channelB);

   
    // Función para controlar el motor en ambas direcciones usando el puente H
    void setSpeed(int speed);
};

class RMotor {
private: 
    int rodilloPin;         // Pin exclusivo para el control del rodillo 
    int rodilloChannel;     // Canal PWM exclusivo para el control del rodillo

    public:
    //Constructor de la clase RMotor
     RMotor(int rPin, int rChannel);

    // Función para controlar la velocidad del rodillo usando el TIP122
     void speedrodillo(int speed);

};

#endif // MOTOR_H