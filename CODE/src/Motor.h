// Motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>




class Motor {
    private:
    int inPinA;
    int inPinB;

    public:
    /**
     * @brief Constructor de la clase Motor.
     *
     * @param pinA El número del pin de entrada A.
     * @param pinB El número del pin de entrada B.
     */
    Motor(int pinA, int pinB);

    /**
     * @brief Establece la velocidad del motor.
     *
     * @param speed La velocidad del motor.
     */
    void setSpeed(int speed);
    
};

#endif // MOTOR_H