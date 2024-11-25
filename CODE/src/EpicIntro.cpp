#include "EpicIntro.h"
#include <Arduino.h>

// Función para reproducir la melodía de Iron Maiden
void playIntro(int buzzerPin) {
    int melodyLength = sizeof(ironMelody) / sizeof(ironMelody[0]);

    for (int i = 0; i < melodyLength; i++) {
        // Calcula la duración de la nota
        int noteDuration = 1000 / ironDurations[i];

        // Genera el tono en el buzzer
        tone(buzzerPin, ironMelody[i], noteDuration);

        // Pausa entre notas
        int pauseBetweenNotes = noteDuration * 1.15;
        delay(pauseBetweenNotes);

        // Detiene el buzzer
        noTone(buzzerPin);
    }
}