#include <Arduino.h>

// ==========================
// Motor / encoder parameters
// ==========================
const float Kv       = 400.0f;        // RPM per Volt
const float T        = 0.15f;
const float Npulses  = 70.0f;         // pulses per rev
const float freqToRPM = 60.0f / Npulses;

// PI constants
const float wn   = 1.0f / T;
const float zeta = 1.0f;
const float wo   = 1.0f / T;

const float Kp = (2.0f * zeta * wn / wo - 1.0f) / Kv;
const float KI = (wn * wn) / (Kv * wo);

// ==========================
// Pins
// ==========================
const int encoderPin = 5;    // Hall sensor
const int motorPin   = 10;   // PWM
const int ledPin     = 13;   // status LED

// ==========================
// PWM config (analogWrite)
// ==========================
const int PWM_BITS = 8;         // 8-bit (0–255)
const int PWM_MAX  = 255;       // max for analogWrite
const int PWM_FREQ = 20000;     // 20 kHz motor PWM

// ==========================
// Measurement/control vars
// ==========================
volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseMicros = 0;

float setRPM = 1000.0f;
float measRPM = 0.0f;
float filtRPM = 0.0f;

float error = 0.0f;
float intError = 0.0f;

unsigned long lastMeasMicros = 0;
unsigned long lastLoopMicros = 0;

const unsigned long MEAS_PERIOD_US = 50000;     // 50 ms
const unsigned long STALL_TIMEOUT_US = 300000;  // 0.3 s stall detect

// ==========================
// Interrupt routine
// ==========================
void IRAM_ATTR encoderISR() {
    pulseCount++;
    lastPulseMicros = micros();
}

// ==========================
// Setup
// ==========================
void setup() {
    delay(1000);

    Serial.begin(115200);

    pinMode(encoderPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);

    // Set up analogWrite() behaviour
    analogWriteResolution(motorPin, PWM_BITS);
    analogWriteFrequency(motorPin, PWM_FREQ);

    analogWrite(motorPin, 0);

    lastMeasMicros = micros();
    lastLoopMicros = micros();

    Serial.println("Starting clean analogWrite Hall RPM PI controller...");
}

// ==========================
// Main loop
// ==========================
void loop() {
    unsigned long now = micros();
    float dt = (now - lastLoopMicros) * 1e-6f;
    lastLoopMicros = now;

    if (dt > 0.1f) dt = 0.1f;

    // ------------------------------
    // 1. RPM measurement (every 50ms)
    // ------------------------------
    if (now - lastMeasMicros >= MEAS_PERIOD_US) {
        lastMeasMicros += MEAS_PERIOD_US;

        static uint32_t lastCount = 0;
        uint32_t countNow = pulseCount;
        uint32_t pulses = countNow - lastCount;
        lastCount = countNow;

        float windowSec = MEAS_PERIOD_US * 1e-6f;
        float freq = pulses / windowSec;

        measRPM = freq * freqToRPM;

        if (now - lastPulseMicros > STALL_TIMEOUT_US)
            measRPM = 0.0f;

        // low-pass filter
        const float alpha = 0.2f;
        filtRPM = alpha * measRPM + (1.0f - alpha) * filtRPM;

        if (pulses > 0)
            digitalWrite(ledPin, !digitalRead(ledPin)); // pulse activity blink
    }

    // ------------------------------
    // 2. PI control
    // ------------------------------
    error = setRPM - filtRPM;

    intError += KI * error * dt;
    if (intError > 1.0f) intError = 1.0f;
    if (intError < 0.0f) intError = 0.0f;

    float u = Kp * error + intError;
    if (u > 1.0f) u = 1.0f;
    if (u < 0.0f) u = 0.0f;

    int pwmVal = (int)(u * PWM_MAX);
    analogWrite(motorPin, pwmVal);

    // ------------------------------
    // 3. Serial plotter output
    // ------------------------------
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 10000) {
        lastPrint = now;
        Serial.print(setRPM);
        Serial.print(",");
        Serial.print(measRPM);
        Serial.print(",");
        Serial.print(filtRPM);
        Serial.print(",");
        Serial.println(pwmVal);
    }
}
