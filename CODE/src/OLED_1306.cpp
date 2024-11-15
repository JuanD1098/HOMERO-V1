#include "OLED_1306.h"

// Constructor
OLED::OLED() : screen(128, 64, &Wire, -1) {
    // Configura el tamaño de la pantalla y usa I2C con la librería Adafruit
}

// Método para inicializar la pantalla
void OLED::begin() {
    if (!screen.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Dirección I2C: 0x3C
        Serial.println("SSD1306 allocation failed");
        for (;;);  // Si falla, detener ejecución
    }
    screen.clearDisplay();  // Borra la pantalla al iniciar
    screen.display();       // Actualiza la pantalla
}

// Método para borrar la pantalla
void OLED::clearDisplay() {
    screen.clearDisplay();
}

// Método para imprimir texto en la pantalla
void OLED::printTextS(const char* text, int x, int y) {
    screen.setTextSize(1);           // Tamaño de texto
    screen.setTextColor(SSD1306_WHITE);  // Color de texto
    screen.setCursor(x, y);          // Posición del cursor
    screen.print(text);              // Imprime el texto en la pantalla
}
// Método para imprimir texto en la pantalla
void OLED::printTextM(const char* text, int x, int y) {
    screen.setTextSize(2);           // Tamaño de texto
    screen.setTextColor(SSD1306_WHITE);  // Color de texto
    screen.setCursor(x, y);          // Posición del cursor
    screen.print(text);              // Imprime el texto en la pantalla
}

// Método para actualizar la pantalla (mostrar cambios)
void OLED::display() {
    screen.display();
}
