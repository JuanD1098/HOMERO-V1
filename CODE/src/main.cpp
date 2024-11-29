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

#define LED_1 13
#define LED_2 32
#define LED_3 33

// Constantes del filtro complementario
float alpha = 0.97; // Peso del giroscopio
float anguloX = 0, anguloY = 0;
unsigned long lastTime = 0; //

// Variables para almacenar offsets y ángulos calibrados
float accelOffsetX = 0.0, accelOffsetY = 0.0, accelOffsetZ = 0.0;
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0;

// Instancias de sensores y periféricos
BatteryReader battery(ADC_PIN, RESISTOR_1, RESISTOR_2);
MPU6050Sensor mpuSensor(0x68);
FlySkyReceiver Homero(Serial2, 1);
OLED screen;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Motores y ESC
Motor motor_a(12, 14, 8, 9);
Motor motor_b(26, 27, 3, 4);
RMotor rodillo(5, 5);
ESC esc(15);

// Temporización
unsigned long previousMillis = 0;
const unsigned long interval = 250;

void LED_1H() {
    digitalWrite(LED_1, HIGH);
    digitalWrite(LED_2, LOW);
    digitalWrite(LED_3, LOW);
}
void LED_2H() {
    digitalWrite(LED_1, LOW);
    digitalWrite(LED_2, HIGH);
    digitalWrite(LED_3, LOW);
}
void LED_3H() {
    digitalWrite(LED_1, LOW);
    digitalWrite(LED_2, LOW);
    digitalWrite(LED_3, HIGH);
}
void LED_N() {
    digitalWrite(LED_1, HIGH);
    digitalWrite(LED_2, HIGH);
    digitalWrite(LED_3, LOW);
}

void calibrarMPU6050() {
    int numSamples = 3000;
    float tempAccelX = 0.0, tempAccelY = 0.0, tempAccelZ = 0.0;
    float tempGyroX = 0.0, tempGyroY = 0.0, tempGyroZ = 0.0;

    Serial.println("Calibrando MPU6050, mantén el dispositivo inmóvil...");
    for (int i = 0; i < numSamples; i++) {
        if (mpuSensor.readData()) {
            float ax, ay, az, gx, gy, gz;
            mpuSensor.getAcceleration(ax, ay, az);
            mpuSensor.getGyroscope(gx, gy, gz);

            tempAccelX += ax;
            tempAccelY += ay;
            tempAccelZ += az;
            tempGyroX += gx;
            tempGyroY += gy;
            tempGyroZ += gz;
        }
        yield(); // Evita watchdog en caso de bucle largo
    }

    // Calcular promedios
    accelOffsetX = tempAccelX / numSamples;
    accelOffsetY = tempAccelY / numSamples;
    accelOffsetZ = tempAccelZ / numSamples;
    gyroOffsetX = tempGyroX / numSamples;
    gyroOffsetY = tempGyroY / numSamples;
    gyroOffsetZ = tempGyroZ / numSamples;

    Serial.println("Calibración completada.");
}

void calcularAngulos(float ax, float ay, float az, float gx, float gy, float dt) {
    // Verificar que dt sea válido para evitar errores
    if (dt <= 0 || dt > 1) return;

    // Calcular ángulos a partir del acelerómetro
    float anguloAccelX = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    float anguloAccelY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // Calcular el cambio de ángulo por giroscopio
    anguloX += gx * dt; // gx está en °/s, dt en segundos
    anguloY += gy * dt;

    // Aplicar filtro complementario
    anguloX = alpha * anguloX + (1.0 - alpha) * anguloAccelX;
    anguloY = alpha * anguloY + (1.0 - alpha) * anguloAccelY;
}

void setup() {
    playIntro(BUZZER_PIN);
    screen.begin();
    pinMode(FAN, OUTPUT);
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    Serial.begin(115200);
    mlx.begin();
    Wire.begin();

    esc.attach(1000, 2000);
    esc.speedESC(-255); // Inicia apagado

    Serial2.begin(115200, SERIAL_8N1, 16, 17);

    if (mpuSensor.begin()) {
        Serial.println("MPU6050 inicializado correctamente.");
        calibrarMPU6050(); // Calibrar al inicio
    } else {
        Serial.println("Error al inicializar el MPU6050.");
    }

    Serial.println("Iniciando lectura de 10 canales de FlySky");
    battery.setCalibrationOffset(0.25);

    // Secuencia de armado del ESC
    esc.speedESC(-255);
    delay(2000);
    esc.speedESC(0);
    delay(1000);
    Serial.println("ESC armado correctamente.");
}

void leerMPU6050(float& correctedAx, float& correctedAy, float& correctedAz,
                 float& correctedGx, float& correctedGy, float& correctedGz) {
    if (mpuSensor.readData()) {
        float ax, ay, az, gx, gy, gz;
        mpuSensor.getAcceleration(ax, ay, az);
        mpuSensor.getGyroscope(gx, gy, gz);

        // Aplicar corrección de offsets
        correctedAx = ax - accelOffsetX;
        correctedAy = ay - accelOffsetY;
        correctedAz = az - accelOffsetZ;
        correctedGx = gx - gyroOffsetX;
        correctedGy = gy - gyroOffsetY;
        correctedGz = gz - gyroOffsetZ;
    }
}



void loop() {
    yield();
    Homero.AllChannels();
    float voltage = battery.readBatteryVoltage();
    float temperatura = mlx.readObjectTempC();
    int CHON = Homero.getChannelPWM(10);
    int OK = Homero.getChannelPWM(6);
    int BATT = Homero.getChannelPWM(5);
    bool ESC_ON = false;
    bool M_ON = false;
    bool batteryAlertTriggered = false; // Bandera para controlar las alertas de batería
  
    // Validar rango de entrada de valores
    voltage = constrain(voltage, 0, 13); // Para una batería 3S
    temperatura = constrain(temperatura, -40, 125);


unsigned long currentMillis = millis();
  

if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float dt = (currentMillis - lastTime) / 1000.0;
    if (dt <= 0 || dt > 1) dt = 0.01; // Valor mínimo razonable

    lastTime = currentMillis;

    float correctedAx, correctedAy, correctedAz, correctedGx, correctedGy, correctedGz;
    leerMPU6050(correctedAx, correctedAy, correctedAz, correctedGx, correctedGy, correctedGz);

    // Calcular los ángulos con el filtro complementario
    calcularAngulos(correctedAx, correctedAy, correctedAz, correctedGx, correctedGy, dt);

    // Imprimir los ángulos
    //Serial.printf("Ángulo X: %.2f, Ángulo Y: %.2f\n", anguloX, anguloY);

    // Obtener valores de los canales
    int ch1 = Homero.getChannelPWM(1);
    int ch2 = Homero.getChannelPWM(2);
    int ch3 = Homero.getChannelPWM(3);
    int ch10 = Homero.getChannelPWM(10);
    int ch6 = Homero.getChannelPWM(6);
    int ch5 = Homero.getChannelPWM(5);


    // Mapear el valor de CH3 de -255 a 255 a 0 a 100 %
    int ch3Percentage = map(ch3, -255, 255, 0, 100);


    // Actualizar la pantalla OLED con los valores seleccionados
    screen.clearDisplay();

    // Mostrar valores de los canales
    screen.printTextS(("CH1: " + String(ch1)).c_str(), 0, 0);
    screen.printTextS(("CH2: " + String(ch2)).c_str(), 0, 8);

    if (ch6 == 255 ){
    screen.printTextS(("CH3: " + String(ch3Percentage) + " %").c_str(), 0, 16);
    }
    else {screen.printTextS("CH3: 0 %", 0, 16);
    }

    screen.printTextS(("AngX: " + String(anguloX, 1)).c_str(), 0, 24);
    screen.printTextS(("AngY: " + String(anguloY, 1)).c_str(), 0, 32);
        screen.printTextS(("GZ: " + String(correctedGz, 2)).c_str(), 0, 40);

    // Mostrar el voltaje de la batería
    screen.printTextS(("BAT: " + String(voltage, 2) + "V").c_str(), 0, 48);

    // Mostrar la temperatura
    if (temperatura > -40 && temperatura < 125) {
        screen.printTextS(("Temp: " + String(temperatura, 1) + "C").c_str(), 0, 56);
    } else {
        screen.printTextS("Temp: Error", 0, 56);
    }

    if (ch10 == -255){
        screen.printTextS("MOTORS OFF", 64, 0);
    }
    else {
        screen.printTextS("MOTORS ON", 64, 0);
    }
     if (ch6 == -255){
        screen.printTextS("EDF OFF", 64, 8);
    }
    else {
        screen.printTextS("EDF ON", 64, 8);
    }

    if (ch5 == -255) {
        screen.printTextS("BATTERY MODE", 55, 16 );
    }
    else {
        screen.printTextS("SOURCE MODE", 60, 16 );
    }

    // Refrescar pantalla
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
        M_ON = true;
    } else {
        motor_a.setSpeed(0);
        motor_b.setSpeed(0);
        rodillo.speedrodillo(-255);
        M_ON = false;
    }

  // Control del ESC con validación
    if (OK < -250) {
        esc.speedESC(0);  // Apaga el ESC
         ESC_ON = false;
    } else {
        int speedESC = map(Homero.getChannelPWM(3), -255,255,0,255);
        esc.speedESC(speedESC);
        digitalWrite(LED_2, HIGH);
        ESC_ON = true;
    }

    if (ESC_ON && M_ON){
        LED_3H();
    }
    else if (M_ON) {
        LED_N();
    }
    else if (ESC_ON) {
        LED_2H();
    }
    else {
        LED_1H();
    }

if (BATT == -255 && !ESC_ON) {

    if (voltage <= 9.5 && !batteryAlertTriggered) {
        // Mostrar mensaje de alerta solo la primera vez
        screen.clearDisplay();
        screen.printTextM("LOW", 40, 6);
        screen.printTextM("BATTERY", 10, 30);
        screen.display();

        tone(BUZZER_PIN, 2000, 300); // Emitir tono de alerta
        delay(300);                  // Permitir que el buzzer termine
        noTone(BUZZER_PIN);          // Apagar el buzzer
        batteryAlertTriggered = true; // Activar la bandera de alerta
        esc.speedESC(0); // Apagar ESC
        return; // Salir del loop para evitar actualizaciones adicionales
    } 
    
    else {
        batteryAlertTriggered = false; // Resetear bandera si el voltaje vuelve a niveles normales
    }
}
}