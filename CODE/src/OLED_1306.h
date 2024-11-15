#ifndef OLED_1306_H
#define OLED_1306_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

class OLED {
public:
    OLED();                             // Constructor
    void begin();                       // Inicializa la pantalla
    void clearDisplay();                // Borra la pantalla
    void printTextS(const char* text, int x, int y);
    void printTextM(const char* text, int x, int y);  // Imprime texto en coordenadas específicas
    void display();                     // Actualiza la pantalla

private:
    Adafruit_SSD1306 screen;  // Instancia de la pantalla SSD1306
};

#endif
