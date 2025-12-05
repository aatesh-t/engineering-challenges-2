// Temperature sensing + heater control
#include <Arduino.h>

const int analogPin = A7;
const float Vcc = 3.3;
const float Rref = 10000.0;
unsigned long pre = 0;
const long interval = 1000;

const int pwmPin = 12;

float MeasTemp = 0.0;
float hysteresis = 0.5;

float SetpointTemp = 35;   // Example setpoint

// Lookup table: Temp ↔ Resistance
struct TRPair {
    float temp;
    float resistance;
};

TRPair table[] = {
    {24.5, 10622},
    {25.0, 10200},
    {25.5, 9927},
    {26.0, 9650},
    {26.5, 9583},
    {27.0, 9400},
    {27.5, 9200},
    {28.0, 9100},
    {28.5, 8920},
    {29.0, 8690},
    {29.5, 8520},
    {30.0, 8380},
    {30.5, 8215},
    {31.0, 8070},
    {31.5, 7930},
    {32.0, 7750},
    {32.5, 7600},
    {33.0, 7507},
    {33.5, 7380},
    {34.0, 7200},
    {34.5, 7040},
    {35.0, 6850},
    {35.5, 6150}
};

const int tableSize = sizeof(table) / sizeof(table[0]);

// Convert resistance → temperature using piecewise linear interpolation
float resistanceToTemp(float R) {

    if (R >= table[0].resistance) return table[0].temp;
    if (R <= table[tableSize - 1].resistance) return table[tableSize - 1].temp;

    for (int i = 0; i < tableSize - 1; i++) {
        float R1 = table[i].resistance;
        float R2 = table[i + 1].resistance;

        if (R <= R1 && R >= R2) {
            float T1 = table[i].temp;
            float T2 = table[i + 1].temp;

            float ratio = (R - R2) / (R1 - R2);
            return T2 + ratio * (T1 - T2);
        }
    }
    return -999; // Shouldn't happen
}

void setup() {
    Serial.begin(9600);
    pinMode(pwmPin, OUTPUT);
}

void loop() {

    unsigned long now = millis();

    if (now - pre >= interval) {
        pre = now;

        // Read ADC
        float vadc = analogRead(analogPin);
        float Vout = (vadc / 4095.0) * Vcc;

        // Avoid division by zero if Vout == Vcc
        if (Vout >= Vcc * 0.999) {
            Serial.println("Error: Vout too high");
            return;
        }

        // Convert to resistance
        float Rx = Rref * (Vout / (Vcc - Vout));

        // Convert to temperature
        MeasTemp = resistanceToTemp(Rx);

        // Print all information
        Serial.print("Vout = "); Serial.print(Vout, 3);
        Serial.print(" V, R = "); Serial.print(Rx);
        Serial.print(" ohms, Temp = "); 
        Serial.println(MeasTemp, 2);

        // === Heater control with hysteresis ===
        float duty = 0.3;
        int pwmV = duty * 255;

        if (MeasTemp < SetpointTemp - hysteresis) {
            analogWrite(pwmPin, pwmV);
            Serial.println("Heating ON");
        }
        else if (MeasTemp > SetpointTemp) {
            digitalWrite(pwmPin, LOW);
            Serial.println("Heating OFF");
        }

        Serial.println();
    }
}
