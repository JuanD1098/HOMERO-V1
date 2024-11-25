// Motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <ESP32Servo.h>

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

class ESC {
private:
    int ESCPin;             // Pin exclusivo para el control del ESC
    Servo esc;              // Objeto de la clase Servo para control del ESC

public:
    ESC(int EPin);                     // Constructor
    void attach(int minPulse, int maxPulse); // Configurar el ESC
    void speedESC(int speed);          // Control de velocidad del ESC
};

#endif // MOTOR_H

