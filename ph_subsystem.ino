// pH subsystem 
#include <Arduino.h>

const int phPin = A0;
const int acidPumpPin = 5;
const int alkaliPumpPin = 10;
const int pwmFrequency = 5000;

const float phSetpoint = 5.0;
const float Kp = 100.0;
const float Ki = 2.0;
const float Kd = 0.0;

const float phDeadband = 0.1;
const float integralMax = 500.0;
const float outputMax = 5000.0;
const float outputMin = 100.0;
const int minDutyCycle = 150;
const int maxDutyCycle = 255;

const unsigned long measureInterval = 2000;
const unsigned long pumpCooldown = 1000;

float integral = 0.0;
float previousError = 0.0;
unsigned long lastMeasurement = 0;
unsigned long lastPumpAction = 0;
float currentPh = 5.0;
float currentVoltage = 0.0;
bool firstRun = true;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  
  analogWriteFrequency(acidPumpPin,pwmFrequency);
  analogWriteFrequency(alkaliPumpPin,pwmFrequency);
  
  
  // Set pins as outputs
  pinMode(acidPumpPin, OUTPUT);
  pinMode(alkaliPumpPin, OUTPUT);

  Serial.print("Setpoint: ");
  Serial.println(phSetpoint, 2);
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print(", Ki:");
  Serial.println(Ki);
  Serial.print(", Kd");
  Serial.println(Kd);
  Serial.print("Deadband: +-");
  Serial.println(phDeadband, 2);

  delay(2000);
}

// Read average voltage from pH probe
float getAverageVoltage(int pin, int samples = 40) {
  long total = 0;
  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delay(5);
  }

  float avgADC = total / samples;
  return avgADC * (3.3 / 4095.0); // Convert ADC to voltage
}

// Calculate pH from voltage
float calculatePh(float V) {
  const float V4 = 0.9915;
  const float V7 = 1.1735;
  const float V10 = 1.3525;

  const float gradient47 = (7.0f - 4.0f) / (V7 - V4);
  const float intercept47 = 7.0f - gradient47 * V7;

  const float gradient710 = (10.0f - 7.0f) / (V10 - V7);
  const float intercept710 = 10.0f - gradient710 * V10;

  if (V <= V7) {
    return gradient47 * V + intercept47;
  } else {
    return gradient710 * V + intercept710;
  }
}

// Start function
void startPump(int pin, int maxDuty = 255, int step = 5, int delayMs = 50) {
  for (int duty = 0; duty <= maxDuty; duty += step) {
    analogWrite(pin, duty);
    delay(delayMs);
  }
}

// Stop function
void stopPump(int pin, int currentDuty, int step = 5, int delayMs = 50) {
  for (int duty = currentDuty; duty >= 0; duty -= step) {
    analogWrite(pin, duty);
    delay(delayMs);
  }
}

// Running pump function
void activatePump(int pin, unsigned long duration, int dutyCycle = 255) {
  startPump(pin, dutyCycle);
  delay(duration);
  stopPump(pin, dutyCycle);
}

void pidControl(float ph, float dt) {
  float error = phSetpoint - ph;

  if (abs(error) < phDeadband) {
    Serial.println("Status: pH within deadband");
    integral = 0.0;
    previousError = error;
    return;
  }

  if (millis() - lastPumpAction < pumpCooldown) {
    Serial.println("Status: Cooldown active");
    return;
  }

  float P = Kp * error;

  integral += error * dt;
  integral = constrain(integral, -integralMax, integralMax);
  float I = Ki * integral;

  float D = 0.0;
  if (!firstRun) {
    float derivative = (error - previousError) / dt;
    D = Kd * derivative;
  } else {
    firstRun = false;
  }

  float output = P + I + D;

  if (abs(output) < outputMin) {
    Serial.println("Status: Output below minimum threshold");
    previousError = error;
    return;
  }

  unsigned long pumpTime = constrain(abs(output), outputMin, outputMax);
  float outputRatio = abs(output) / outputMax;
  int dutyCycle = constrain(minDutyCycle + outputRatio * (maxDutyCycle - minDutyCycle), minDutyCycle, maxDutyCycle);

  if (error > 0) {
    // pH too low, activate alkali pump
    Serial.println("pH low, activating alkali pump");
    Serial.print(" Duration: ");
    Serial.print(pumpTime);
    Serial.print(" ms, Duty: ");
    Serial.println(dutyCycle);
    activatePump(alkaliPumpPin, pumpTime, dutyCycle);
  } else {
    // pH too high, activate acid pump
    Serial.println("pH high, activating acid pump");
    Serial.print(" Duration: ");
    Serial.print(pumpTime);
    Serial.print(" ms, Duty: ");
    Serial.println(dutyCycle);
    activatePump(acidPumpPin, pumpTime, dutyCycle);
  }

  lastPumpAction = millis();
  previousError = error;
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastMeasurement >= measureInterval) {
    float dt = (currentTime - lastMeasurement) / 1000.0;
    lastMeasurement = currentTime;

    // Read voltage & calculate pH from probe
    currentVoltage = getAverageVoltage(phPin);
    currentPh = calculatePh(currentVoltage);

    Serial.print("Voltage: ");
    Serial.print(currentVoltage, 3);
    Serial.println(" V");
    Serial.print("pH: ");
    Serial.print(currentPh, 2);
    Serial.print(" (Setpoint: ");
    Serial.print(phSetpoint, 2);
    Serial.println(")");

    pidControl(currentPh, dt);
  }
}