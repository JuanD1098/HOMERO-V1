
#include <Arduino.h>
#include "FlySkyReceiver.h"
#include "Motor.h"
#include "OLED_1306.h"
#include "Battery.h"
#include "MPU6050.h"
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "EpicIntro.h"


// Configura las resistencias del divisor (1.8kΩ y 560Ω)
#define RESISTOR_1 1788.0
#define RESISTOR_2 560.2

// Pin ADC del ESP32 donde está conectado el divisor
#define ADC_PIN 25

#define BUZZER_PIN 23

#define FAN 2



unsigned long previousMillis = 0; // Variable para almacenar el tiempo anterior
const unsigned long interval = 250; // Intervalo en milisegundos


// Crea una instancia de la clase BatteryReader
BatteryReader battery(ADC_PIN, RESISTOR_1, RESISTOR_2);

MPU6050Sensor mpuSensor(0x68); // Crear una instancia del MPU6050 con dirección I2C
// Crear instancia de FlySkyReceiver usando Serial2 y el temporizador 1 para ESP32
FlySkyReceiver Homero(Serial2, 1);

OLED screen;  // Crear una instancia de la clase OLED

Adafruit_MLX90614 mlx = Adafruit_MLX90614(); //Se declara una variable u objeto para el sensor  

// Inicialización de motores en sus respectivos pines y canales PWM
Motor motor_a(12, 14, 8, 9);  // Motor A en pines 26 y 27, usando canales PWM 0 y 1
Motor motor_b(26, 27, 3, 4);  // Motor B en pines 33 y 32, usando canales PWM 2 y 3
RMotor rodillo(5,5);
ESC esc(15);


/*

void setup() {
    Serial.begin(115200);
   
void loop() {
    // Leer datos del sensor
    if (mpuSensor.readData()) {
        float ax, ay, az, gx, gy, gz;
        mpuSensor.getAcceleration(ax, ay, az);
        mpuSensor.getGyroscope(gx, gy, gz);

        // Imprimir valores en consola
        Serial.printf("Aceleración (m/s^2): X=%.2f, Y=%.2f, Z=%.2f\n", ax, ay, az);
        Serial.printf("Giroscopio (°/s): X=%.2f, Y=%.2f, Z=%.2f\n", gx, gy, gz);
    }

    delay(100); // Esperar 1 segundo entre lecturas
}

*/








void setup() {
    screen.begin();
    playIntro(BUZZER_PIN);
    pinMode(FAN, OUTPUT);
    Serial.begin(115200);
    mlx.begin(); // Inicia el sensor MLX90614
    Wire.begin();
    
    esc.attach(1000, 2000); 
    esc.speedESC(-255);      // Asegúrate de iniciar en 0 (apagado)

    Serial2.begin(115200, SERIAL_8N1, 16, 17); // Configura UART2 en GPIO16 (RX2) y GPIO17 (TX2)
   

    // Inicializar MPU6050
    if (mpuSensor.begin()) {
        Serial.println("MPU6050 inicializado correctamente.");
    } else {
        Serial.println("Error al inicializar el MPU6050.");
    }

    // *** Secuencia de armado del ESC ***
    Serial.println("Iniciando secuencia de armado del ESC...");
    esc.speedESC(-255);  // Mínimo (1000 µs)
    delay(2000);         // Espera 2 segundos para permitir el armado
    esc.speedESC(0);     // Enviar señal neutra (si es necesario)
    delay(1000);         // Espera adicional para asegurar el armado

    Serial.println("ESC armado correctamente.");


    battery.setCalibrationOffset(0.5); // Ajusta según las pruebas

    // Comunicación serial para depuración
    Serial.println("Iniciando lectura de 10 canales de FlySky");
    
    screen.clearDisplay();
    
}

void loop() {
    yield();
    Homero.AllChannels();
    float voltage = battery.readBatteryVoltage();
    float temperatura = mlx.readObjectTempC();
    int CHON = Homero.getChannelPWM(10);
    int OK = Homero.getChannelPWM(6);

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Actualizar la pantalla OLED con los valores actuales
        screen.clearDisplay();
        screen.printTextS(("CH1: " + String(Homero.getChannelPWM(1))).c_str(), 0, 0);
        screen.printTextS(("CH2: " + String(Homero.getChannelPWM(2))).c_str(), 0, 8);
        screen.printTextS(("ESC: " + String(Homero.getChannelPWM(3))).c_str(), 0, 16);
        screen.printTextS(("CH4: " + String(Homero.getChannelPWM(4))).c_str(), 0, 24);
        screen.printTextS(("CH5: " + String(Homero.getChannelPWM(5))).c_str(), 0, 32);
        screen.printTextS(("CH6: " + String(Homero.getChannelPWM(6))).c_str(), 64, 0);
        screen.printTextS(("CH7: " + String(Homero.getChannelPWM(7))).c_str(), 64, 8);
        screen.printTextS(("CH8: " + String(Homero.getChannelPWM(8))).c_str(), 64, 16);
        screen.printTextS(("SPD: " + String(Homero.getChannelPWM(9))).c_str(), 64, 24);
        screen.printTextS(("CH10: " + String(Homero.getChannelPWM(10))).c_str(), 64, 32);
        screen.printTextS(("BAT: " + String(voltage)).c_str(), 20, 40);

        if (temperatura > -40 && temperatura < 125) {
            screen.printTextS((String("Temp: ") + String(temperatura) + "C").c_str(), 10, 45);
        } else {
            screen.printTextS("Temp: Error", 10, 45);
        }

        screen.display();
    }

    // Control del ventilador
    if (temperatura >= 40.0) {
        digitalWrite(FAN, HIGH);
    } else {
        digitalWrite(FAN, LOW);
    }

    // Control de motores y rodillo
    if (CHON != -255) {
        motor_a.setSpeed(Homero.getChannelPWM(2));
        motor_b.setSpeed(Homero.getChannelPWM(1));
        rodillo.speedrodillo(Homero.getChannelPWM(7));
    } else {
        motor_a.setSpeed(0);
        motor_b.setSpeed(0);
        rodillo.speedrodillo(-255);
    }

  // Control del ESC con validación
    if (OK < -250) {
        esc.speedESC(0);  // Apaga el ESC
    } else {
        int speedESC = map(Homero.getChannelPWM(3), -255,255,0,255);
        esc.speedESC(speedESC);
    }
}
