#include <Arduino.h>
#include "FlySkyReceiver.h"
#include "Motor.h"
#include "OLED_1306.h"


// Inicialización de motores en sus respectivos pines y canales PWM
Motor motor_a(26, 27, 0, 1);  // Motor A en pines 26 y 27, usando canales PWM 0 y 1
Motor motor_b(14, 12, 2, 3);  // Motor B en pines 33 y 32, usando canales PWM 2 y 3
RMotor rodillo(35,4);

// Crear instancia de FlySkyReceiver usando Serial2 y el temporizador 1 para ESP32
FlySkyReceiver Homero(Serial2, 1);

OLED screen;  // Crear una instancia de la clase OLED

void setup() {
  Serial.begin(115200);          // Comunicación serial para depuración
  Serial.println("Iniciando lectura de 10 canales de FlySky");
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // Configura UART2 en GPIO16 (RX2) y GPIO17 (TX2)
  screen.begin();
  screen.clearDisplay();
}

void loop() {
  Homero.AllChannels();             // Leer y mapear valores de los canales
  Homero.printChannelValues();       // Imprimir valores mapeados en el monitor serial
  screen.printTextM("OK",64,32);
  int pwmSpeed = Homero.getChannelPWM(7);  // Obtener PWM calculado para el canal #
  rodillo.speedrodillo(pwmSpeed);              // Configurar la velocidad del motor A
  delay(200);                        // Pequeño retraso


}
