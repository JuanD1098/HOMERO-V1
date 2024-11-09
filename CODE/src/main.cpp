#include <Arduino.h>
#include "FlySkyReceiver.h"
#include "Motor.h"
#include "MPU9250Sensor.h"
#include "OLED_1306.h"

//engines
Motor motor_a(26, 27);
Motor motor_b(9, 32);



// Crear instancia de FlySkyReceiver usando Serial y el temporizador 1 para esp32
FlySkyReceiver Homero(Serial2, 1);  


OLED screen;  // Crear una instancia de la clase OLED

//Crear instancia de MPU9250
//MPU9250Sensor imuSensor;



void setup() {
  
  Serial.begin(115200);      // Iniciar comunicación serial para depuración
  Serial.println("Iniciando lectura de 10 canales de FlySky");
  screen.begin();
  screen.clearDisplay();

/*
    while (!Serial) {}
    if (!imuSensor.begin()) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        while (1) {}
    }
    imuSensor.setConfiguration();
*/

}

void loop() {
  Homero.AllChannels();     // Leer y mapear valores de los canales
 // imuSensor.readData();
  Homero.printChannelValues(); // Imprimir valores mapeados en el monitor serial
  int pwmSpeed = Homero.getChannelPWM(9);  // Obtener PWM calculado para el canal 3
    motor_a.setSpeed(pwmSpeed);              // Configurar la velocidad del motor A
    screen.printText("PWM Canal 9:",20,10 );
    screen.printText(String(pwmSpeed).c_str(),25,10 );
    delay(100);  // Pequeño retraso                  

}
