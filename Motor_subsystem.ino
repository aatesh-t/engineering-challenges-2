#include <Arduino.h>

// ==========================
// Motor parameters (unchanged)
// ==========================
const float Kv = 200;
const float T = 0.15;
const float Npulses = 70;
const int RPMmax = 1000;
const int Tmin = 6e7 / 2000 / Npulses; // "Impossible RPM of 2000 -> noise"

const float wn = 1 / T;
const float zeta = 1;

const float wo = 1 / T;
const float Kp = (2 * zeta * wn / wo - 1) / Kv;
const float KI = wn * wn / Kv / wo;
const float freqtoRPM = 60 / Npulses;

// ==========================
// ESP32 PIN SETUP
// ==========================
const int encoderpin = 5;      // interrupt
const int motorpin = 10;       // PWM pin (motor control)

// ==========================
// Variables
// ==========================
float measspeed = 0, meanmeasspeed = 0, freq = 0, error = 0;
float KIinterror = 0, deltaT = 0;
float setspeed = 500;          // EXAMPLE SETPOINT

long currtime = 0, prevtime = 0, pulseT = 0, prevpulseT = 0, T1 = 0;
int Vmotor = 0, trig = 0;


volatile unsigned long lastPulseTime = 0;

void IRAM_ATTR freqcount() {
    unsigned long now = micros();

    // Reject pulses that are too close (noise)
    if (now - lastPulseTime < Tmin) return;

    unsigned long period = now - lastPulseTime;
    lastPulseTime = now;

    pulseT = now;

    // Compute frequency safely
    float newFreq = 1e6f / float(period);   // Hz = 1,000,000 / period_us

    // Low-pass filter to avoid jitter
    freq = 0.8f * freq + 0.2f * newFreq;

    // LED blink (non-blocking)
    trig++;
    if (trig >= Npulses/2) {
        trig = 0;
        digitalWrite(13, !digitalRead(13));
    }
}

// ==========================
// SETUP
// ==========================
void setup() {
    delay(1500); //Prevent boot ISR lockup
    pinMode(13, OUTPUT);
    pinMode(encoderpin, INPUT_PULLUP);
    pinMode(motorpin, OUTPUT);

    attachInterrupt(encoderpin, freqcount, RISING);

    Serial.begin(115200);

    // ---- ESP32-compatible analogWrite ----
    analogWriteResolution(motorpin, 8);      // 8-bit (0–255)
    analogWriteFrequency(motorpin, 20000);   // 20 kHz
}

// ==========================
// MAIN LOOP
// ==========================
void loop() {
    currtime = micros();
    deltaT = (currtime - prevtime) * 1e-6;

    if (currtime > T1) {
        prevtime = currtime;
        T1 = T1 + 10000;          // 10ms loop (100 Hz update)

        // Compute RPM
        measspeed = freq * freqtoRPM;

        // PI control
        error = setspeed - measspeed;

        // Integral term
        KIinterror += KI * error * deltaT;
        KIinterror = constrain(KIinterror, 0, 1);   // anti-windup: 0–1

        float u = Kp * error + KIinterror;
        u = constrain(u, 0.0f, 1.0f);

        // To PWM (0–255)
        Vmotor = (int)(u * 255);

        // ---- WRITE PWM ----
        // analogWrite(motorpin, Vmotor);
        analogWrite(motorpin, 255);

        // If no encoder pulses for 0.5s → motor stopped
        if (currtime - pulseT > 500000) {
            measspeed = 0;
            freq = 0;
        }

        // Smooth value for Serial Plotter
        meanmeasspeed = 0.1 * measspeed + 0.9 * meanmeasspeed;

        // Plotting (three traces)
        Serial.print(0); Serial.print(",");
        Serial.print(1500); Serial.print(",");
        Serial.println(meanmeasspeed);
        Serial.print(Vmotor);
        Serial.print(freq)
    }
}
