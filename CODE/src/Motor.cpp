//Motor.cpp

#include "Motor.h"


Motor::Motor(int pinA, int pinB) : inPinA(pinA), inPinB(pinB) {
    pinMode(inPinA, OUTPUT);
    pinMode(inPinB, OUTPUT);
    digitalWrite(inPinA, LOW);
    digitalWrite(inPinB, LOW);
}

/**
 * Establece la velocidad del motor.
 *
 * @param speed La velocidad a establecer. Debe estar en el rango de -255 a 255.
 */
void Motor::setSpeed(int speed) {

    if (speed > 0) {
        analogWrite(inPinA, speed);
        digitalWrite(inPinB, LOW);
    }
    else if (speed < 0) {
        digitalWrite(inPinA, LOW);
        analogWrite(inPinB, -speed);
    }
    else {
        digitalWrite(inPinA, LOW);
        digitalWrite(inPinB, LOW);
    }
}


