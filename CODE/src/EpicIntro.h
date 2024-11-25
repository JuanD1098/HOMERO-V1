#ifndef EPIC_INTRO_H
#define EPIC_INTRO_H


// Define las frecuencias de las notas musicales
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659

// Melodía simplificada de "The Trooper"
const int ironMelody[] = {
    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_B4,  // Ascenso rápido
    NOTE_A4, NOTE_G4, NOTE_E4, NOTE_B4,  // Descenso dinámico
    NOTE_E5, NOTE_D5, NOTE_B4, NOTE_A4,  // Clímax
    NOTE_G4, NOTE_A4, NOTE_E4            // Resolución
};

// Duración de cada nota (4 = negra, 8 = corchea, etc.)
const int ironDurations[] = {
    8, 8, 8, 4,  // Ritmo rápido
    8, 8, 8, 4,  // Más dinámico
    8, 8, 4, 8,  // Épico
    8, 4, 2      // Final potente
};

// Declaración de la función para reproducir la melodía
void playIntro(int buzzerPin);

#endif
