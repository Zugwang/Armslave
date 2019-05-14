#include <Arduino.h>

// Envoyer autant de caractères par ligne que de N_MOTEURS
// E pour éteint,  T pour trigo et A pour anti trigo
// Ex: ET\n pour envoyer E T et N

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

#define N_MOTEURS 8
uint8_t PINS[N_MOTEURS][2] = {{53,52}, {51, 50}, {48, 49}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
unsigned int index = 0; // Designe le moteur qu''on vient controler
long long lastCMDMillis = 0;

void setup() {
    Serial.begin(115200);
    for(unsigned int i = 0; i < N_MOTEURS; i++) {
        pinMode(PINS[i][0], OUTPUT);
        pinMode(PINS[i][1], OUTPUT);
    }
}

void loop() {
    // Si pas de trame depuis + d'une seconde on arrête tout
    if (millis() - lastCMDMillis > 1000) {
        for(unsigned int i = 0; i < N_MOTEURS; i++) {
            digitalWrite(PINS[i][0], HIGH);
            digitalWrite(PINS[i][1], HIGH);
        }
    }
    while(Serial.available() > 0) {
        char c = Serial.read();
        Serial << index << " " << c << "\n"; // on print la commande
        if(c == '\n') { // Fin de transmission
            index = 0;
            continue;
        }
        switch(c) {
            case 'E': // E pour éteint
                digitalWrite(PINS[index][0], HIGH);
                digitalWrite(PINS[index][1], HIGH);
                break;
            case 'T': // Sens trigo
                digitalWrite(PINS[index][0], LOW);
                digitalWrite(PINS[index][1], HIGH);
                break;
            case 'A': // Sens anti trigonométrique
                digitalWrite(PINS[index][0], HIGH);
                digitalWrite(PINS[index][1], LOW);
                break;
            default:
                Serial.println("Unknown char ! Possibilities are E (OFF), T (Trigonometric) and A (Anti Trigonometric)");
                break;
        }
        index++;
        lastCMDMillis = millis();
    }
}
