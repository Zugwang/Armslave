#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_MMA8451.h"
#include <Adafruit_Sensor.h>
// Envoyer autant de caractères par ligne que de N_MOTEURS
// E pour éteint,  T pour trigo et A pour anti trigo
// Ex: ET\n pour envoyer E T et

Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_MMA8451 mma2 = Adafruit_MMA8451();

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

#define N_MOTEURS 8
uint8_t PINS[N_MOTEURS][2] = {{53,52}, {51, 50}, {48, 49}, {46, 47}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
unsigned int index = 0; // Designe le moteur qu''on vient controler
long long lastCMDMillis = 0;


void getSensorOrientation(Adafruit_MMA8451 *mma, float *x, float *y) {
    sensors_event_t event;
    mma->getEvent(&event);
    float accelX = event.acceleration.x;
    float accelY = event.acceleration.y;
    float accelZ = event.acceleration.z;

    *x = atan2(accelY, accelZ) * 180/M_PI;
    *y = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180/M_PI;
}


void setup() {
    Serial.begin(115200);
    for(unsigned int i = 0; i < N_MOTEURS; i++) {
        pinMode(PINS[i][0], OUTPUT);
        pinMode(PINS[i][1], OUTPUT);
    }
    if (! mma.begin(0x1C)) {
        Serial.println("Couldnt start");
        while (1);
    }
    if (! mma2.begin(0x1D)) {
        Serial.println("Couldnt start");
        while (1);
    }

    Serial.println("MMA8451 found!");
    mma.setRange(MMA8451_RANGE_2_G);
    mma2.setRange(MMA8451_RANGE_2_G);

}


void loop() {
    float x_1 = 0, x_2 = 0, y_1 = 0, y_2 = 0;
    getSensorOrientation(&mma, &x_1, &y_1);
    getSensorOrientation(&mma2, &x_2, &y_2);

    Serial << x_1 << "," << x_2 << "\n";

    // Si pas de trame depuis + de 150 ms on arrête tout
    if (millis() - lastCMDMillis > 150) {
        for(unsigned int i = 0; i < N_MOTEURS; i++) {
            digitalWrite(PINS[i][0], HIGH);
            digitalWrite(PINS[i][1], HIGH);
        }
    }
    while(Serial.available() > 0) {
        char c = Serial.read();
        //Serial << index << " " << c << "\n"; // on print la commande
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
                // Serial.println("Unknown char ! Possibilities are E (OFF), T (Trigonometric) and A (Anti Trigonometric)");
                break;
        }
        index++;
        lastCMDMillis = millis();
    }
    delay(5);
}
