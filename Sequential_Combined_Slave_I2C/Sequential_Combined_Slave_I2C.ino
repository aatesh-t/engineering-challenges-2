#include <Wire.h>
#include <Arduino.h>

// ========================================================
// pH control variables and constants
const int phPin = A0;
const int acidPumpPin = 5;
const int alkaliPumpPin = 10;
const int pwmFrequency = 20000;

float phSetpoint = 5.0;
const float Kp = 100.0;
const float Ki = 2.0;
const float Kd = 0.0;

const float phDeadband = 0.1;
const float integralMax = 500.0;
const float outputMax = 8000.0;
const float outputMin = 1500.0;
const int minDutyCycle = 250;
const int maxDutyCycle = 255;

const unsigned long measureInterval = 4000;
const unsigned long pumpCooldown = 1000;

float integral = 0.0;
float previousError = 0.0;
unsigned long lastMeasurement = 0;
unsigned long lastPumpAction = 0;
float currentPh = 5.0;
float currentVoltage = 0.0;
bool firstRun = true;

// non-blocking pump control variables
bool pumpActive = false;
int activePumpPin = 0;
unsigned long pumpStartTime = 0;
unsigned long pumpDuration = 0;
int targetDutyCycle = 0;
int currentDutyCycle = 0;

enum PumpPhase { RAMP_UP, HOLD, RAMP_DOWN, IDLE };
PumpPhase pumpPhase = IDLE;
// end of pH control variables
// ========================================================

// ========================================================
// temperature control variables and constants
const int analogPin = A7;
const float Vcc = 3.3;
const float Rref = 10000.0;
unsigned long tempPre = 0;  // Renamed to avoid conflict with pH timing
const long tempInterval = 1000;
const int pwmPin = 12;
float MeasTemp = 0.0;
float hysteresis = 0.5;
float SetpointTemp = 35.0; 

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
// end of temperature control variables
// ========================================================

// ========================================================
// motor control variables and constants
const float Kv       = 400.0f;        // RPM per Volt
const float T        = 0.15f;
const float Npulses  = 70.0f;         // pulses per rev
const float freqToRPM = 60.0f / Npulses;

// PI constants
const float wn   = 1.0f / T;
const float zeta = 1.0f;
const float wo   = 1.0f / T;
const float Kp_motor = (2.0f * zeta * wn / wo - 1.0f) / Kv;  
const float KI_motor = (wn * wn) / (Kv * wo);                

// Pins
const int encoderPin = 2;    // Hall sensor
const int motorPin   = 11;   // PWM
const int ledPin     = 13;   // status LED

// PWM config
const int PWM_BITS = 8;         // 8-bit (0–255)
const int PWM_MAX  = 255;       // max for analogWrite
const int PWM_FREQ = 20000;     // 20 kHz motor PWM

// Measurement/control vars
volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseMicros = 0;

float setRPM = 1000.0f;  
float measRPM = 0.0f;
float filtRPM = 0.0f;

float error_motor = 0.0f;       
float intError = 0.0f;

unsigned long lastMeasMicros = 0;
unsigned long lastLoopMicros = 0;

const unsigned long MEAS_PERIOD_US = 50000;     // 50 ms
const unsigned long STALL_TIMEOUT_US = 300000;  // 0.3 s stall detect

// end of motor control varaibles
// ========================================================

// motor control interrupt routine
void IRAM_ATTR encoderISR() {
    pulseCount++;
    lastPulseMicros = micros();
}

#define SLAVE_ADDR 0x28
 
typedef struct __attribute__((packed)) {
  float temp_setpoint;
  float pH_setpoint;
  float stir_setpoint;
} SetpointPacket;
 
typedef struct __attribute__((packed)) {
  float temperature;
  float pH;
  float rpm;
} SensorPacket;
 
SetpointPacket setpoints;
SensorPacket sensors;
SensorPacket sensors_safe;

// ========================================================
// pH control functions
// Read average voltage from pH probe
float getAverageVoltage(int pin, int samples = 10) {
  long total = 0;
  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delayMicroseconds(100);
  }

  float avgADC = total / samples;
  return avgADC * (3.3 / 4095.0);
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

  if (V <= V7)
    return gradient47 * V + intercept47;
  else
    return gradient710 * V + intercept710;
}

void updatePumpControl() {
  if (!pumpActive) return;
  
  unsigned long elapsed = millis() - pumpStartTime;
  
  // Phase timings (milliseconds)
  const int RAMP_STEP = 5;
  const int RAMP_DELAY = 50;
  const int NUM_STEPS = (targetDutyCycle + RAMP_STEP - 1) / RAMP_STEP;
  const unsigned long RAMP_UP_TIME = NUM_STEPS * RAMP_DELAY;
  const unsigned long RAMP_DOWN_START = pumpDuration - RAMP_UP_TIME;
  
  switch (pumpPhase) {
    case RAMP_UP:
      if (elapsed < RAMP_UP_TIME) {
        int step = elapsed / RAMP_DELAY;
        currentDutyCycle = min(step * RAMP_STEP, targetDutyCycle);
        analogWrite(activePumpPin, currentDutyCycle);
      } else {
        currentDutyCycle = targetDutyCycle;
        analogWrite(activePumpPin, currentDutyCycle);
        pumpPhase = HOLD;
      }
      break;
      
    case HOLD:
      if (elapsed >= RAMP_DOWN_START) {
        pumpPhase = RAMP_DOWN;
      }
      break;
      
    case RAMP_DOWN:
      if (elapsed < pumpDuration) {
        unsigned long rampDownElapsed = elapsed - RAMP_DOWN_START;
        int step = rampDownElapsed / RAMP_DELAY;
        currentDutyCycle = max(targetDutyCycle - (step * RAMP_STEP), 0);
        analogWrite(activePumpPin, currentDutyCycle);
      } else {
        // Pump cycle complete
        analogWrite(activePumpPin, 0);
        pumpActive = false;
        pumpPhase = IDLE;
        Serial.println("Pump cycle complete");
      }
      break;
      
    case IDLE:
      break;
  }
}

void startPumpNonBlocking(int pin, unsigned long duration, int dutyCycle) {
  if (pumpActive) {
    // Serial.println("Pump already running, request ignored");
    return;
  }
  
  pumpActive = true;
  activePumpPin = pin;
  pumpStartTime = millis();
  pumpDuration = duration;
  targetDutyCycle = dutyCycle;
  currentDutyCycle = 0;
  pumpPhase = RAMP_UP;
  
  Serial.print("Starting pump on pin ");
  Serial.print(pin);
  Serial.print(" for ");
  Serial.print(duration);
  Serial.println("ms");
}

// PID FUNCTION 
void pidControl(float ph, float dt) {
  // Don't trigger new pump if one is already active
  if (pumpActive) {
    // Serial.println("Status: Pump active, waiting for completion");
    return;
  }
  
  float error = phSetpoint - ph;

  // Deadband
  if (abs(error) < phDeadband) {
    // Serial.println("Status: pH within deadband");
    integral = 0.0;
    previousError = error;
    return;
  }

  // Cooldown
  if (millis() - lastPumpAction < pumpCooldown) {
    Serial.println("Status: Cooldown active");
    return;
  }

  // PID (only used for direction + duty scaling)
  float P = Kp * error;
  integral += error * dt;
  integral = constrain(integral, -integralMax, integralMax);
  float I = Ki * integral;

  float D = 0.0;
  if (!firstRun) {
    D = Kd * (error - previousError) / dt;
  } else {
    firstRun = false;
  }

  float output = P + I + D;

  // Pump time based on pH error
  float kTime = 3000;
  unsigned long pumpTime = abs(error) * kTime;
  pumpTime = constrain(pumpTime, 1500, 6000);

  // Duty cycle proportional to PID magnitude
  float outputRatio = abs(output) / outputMax;
  int dutyCycle = constrain(minDutyCycle + outputRatio * (maxDutyCycle - minDutyCycle),
                            minDutyCycle, maxDutyCycle);

  // Start pump (non-blocking)
  if (error > 0) {
    Serial.println("pH low → adding alkali");
    Serial.print("Pump time = "); Serial.print(pumpTime);
    Serial.print(" ms, Duty = "); Serial.println(dutyCycle);
    startPumpNonBlocking(alkaliPumpPin, pumpTime, dutyCycle);
  } else {
    Serial.println("pH high → adding acid");
    Serial.print("Pump time = "); Serial.print(pumpTime);
    Serial.print(" ms, Duty = "); Serial.println(dutyCycle);
    startPumpNonBlocking(acidPumpPin, pumpTime, dutyCycle);
  }

  lastPumpAction = millis();
  previousError = error;
}
// end of pH control functions
// ========================================================

// ========================================================
// temperature control functions
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
// end of temperature control functions
// ========================================================
 
void receiveSetpoints(int count) {
  if (count == sizeof(SetpointPacket)) {
    Wire.readBytes((char*)&setpoints, sizeof(SetpointPacket));

    Serial.println("=== SETPOINTS RECEIVED ===");
    Serial.println(setpoints.temp_setpoint);
    Serial.println(setpoints.pH_setpoint);
    Serial.println(setpoints.stir_setpoint);

    // Update pH setpoint
    phSetpoint = setpoints.pH_setpoint;
    Serial.print("pH setpoint updated to: ");
    Serial.println(phSetpoint, 2);

    // Update temperature setpoint
    SetpointTemp = setpoints.temp_setpoint;
    Serial.print("Temperature setpoint updated to: ");
    Serial.print(SetpointTemp, 2);
    Serial.println(" °C");

    // Update motor RPM setpoint
    setRPM = setpoints.stir_setpoint;
    Serial.print("Motor RPM setpoint updated to: ");
    Serial.print(setRPM, 1);
    Serial.println(" RPM");
  }
}
 
void sendSensors() {
  // fill the sensor struct
  /*
  sensors.temperature = MeasTemp;
  sensors.pH = currentPh;
  sensors.rpm = filtRPM;
  Wire.write((uint8_t*)&sensors, sizeof(SensorPacket));
  */

  // using the safe copy that is updated atomically in loop()
  Wire.write((uint8_t*)&sensors_safe, sizeof(SensorPacket));
}
 
void setup() {
  Serial.begin(115200);

  // ========================================================
  // pH control setup
  analogReadResolution(12);

  analogWriteFrequency(acidPumpPin, pwmFrequency);
  analogWriteFrequency(alkaliPumpPin, pwmFrequency);

  pinMode(acidPumpPin, OUTPUT);
  pinMode(alkaliPumpPin, OUTPUT);

  Serial.print("Setpoint: ");
  Serial.println(phSetpoint, 2);
  Serial.println(Kp);
  Serial.println(Ki);
  Serial.println(Kd);
  Serial.print("Deadband: +-");
  Serial.println(phDeadband, 2);
  // end of pH control setup
  // ========================================================

  // ========================================================
  // temperature control setup
  pinMode(pwmPin, OUTPUT);
  
  Serial.print("Temperature Setpoint: ");
  Serial.print(SetpointTemp, 2);
  Serial.println(" °C");
  Serial.print("Hysteresis: ");
  Serial.print(hysteresis, 2);
  Serial.println(" °C");
  // end of temperature control setup
  // ========================================================

  // ========================================================
  // motor control setup
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);
  
  // Set up analogWrite() behaviour
  analogWriteResolution(motorPin, PWM_BITS);
  analogWriteFrequency(motorPin, PWM_FREQ);

  analogWrite(motorPin, 0);
  
  lastMeasMicros = micros();
  lastLoopMicros = micros();
  
  Serial.print("Motor RPM Setpoint: ");
  Serial.print(setRPM, 1);
  Serial.println(" RPM");
  Serial.println("Motor controller initialised");
  // end of motor control setup
  // ========================================================

  // ========================================================
  // I2C slave setup
  Wire.begin((uint8_t)SLAVE_ADDR);
  Wire.onReceive(receiveSetpoints);
  Wire.onRequest(sendSensors);

  Serial.println("Nano I2C slave ready");
  // end of slave setup
  // ========================================================
}

// ========================================================
// HARDWARE QUIETING FUNCTIONS
// ========================================================

// All OFF helper
inline void allPumpsOff() {
  digitalWrite(acidPumpPin, LOW);
  digitalWrite(alkaliPumpPin, LOW);
}

// Quiet system for pH reading/control
void quietForPH() {
  // Turn off heater
  analogWrite(pwmPin, 0);

  // Turn off motor
  analogWrite(motorPin, 0);

  // Ensure other pump is OFF
  // The active pump (if pumpActive) should not be stopped
  if (!pumpActive) allPumpsOff();
}

// Quiet system for temperature reading/control
void quietForTemp() {
  // Turn off motor
  analogWrite(motorPin, 0);

  // Turn off pumps unless they are actively running
  if (!pumpActive) allPumpsOff();
}

// Quiet system for motor control
void quietForMotor() {
  // Heater OFF
  analogWrite(pwmPin, 0);

  // Pumps OFF unless ramping
  if (!pumpActive) allPumpsOff();
}

 
void loop() {

  // pH control loop
  quietForPH();
  updatePumpControl();

  unsigned long currentTime = millis();

  if (currentTime - lastMeasurement >= measureInterval) {
    float dt = (currentTime - lastMeasurement) / 1000.0;
    lastMeasurement = currentTime;

    currentVoltage = getAverageVoltage(phPin);
    currentPh = calculatePh(currentVoltage);

    /*
    Serial.print("Voltage: ");
    Serial.print(currentVoltage, 3);
    Serial.println(" V");
    Serial.print("pH: ");
    Serial.print(currentPh, 2);
    Serial.print(" (Setpoint: ");
    Serial.print(phSetpoint, 2);
    Serial.println(")");
    */

    pidControl(currentPh, dt);
  }

  // temperature control loop
  quietForTemp();
  unsigned long tempNow = millis();
  
  if (tempNow - tempPre >= tempInterval) {
    tempPre = tempNow;
    
    // Read ADC
    float vadc = analogRead(analogPin);
    float Vout = (vadc / 4095.0) * Vcc;
    
    // Avoid division by zero if Vout == Vcc
    if (Vout >= Vcc * 0.999) {
      Serial.println("Error: Vout too high");
    } else {
      // Convert to resistance
      float Rx = Rref * (Vout / (Vcc - Vout));
      
      // Convert to temperature
      MeasTemp = resistanceToTemp(Rx);
      
      // Print all information
      /*
      Serial.print("Vout = "); Serial.print(Vout, 3);
      Serial.print(" V, R = "); Serial.print(Rx);
      Serial.print(" ohms, Temp = "); 
      Serial.println(MeasTemp, 2);
      */
      
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

  // motor control loop
  quietForMotor();
  unsigned long motorNow = micros();
  float dt_motor = (motorNow - lastLoopMicros) * 1e-6f;
  lastLoopMicros = motorNow;
  
  if (dt_motor > 0.1f) dt_motor = 0.1f;
  
  // RPM measurement (every 50ms)
  if (motorNow - lastMeasMicros >= MEAS_PERIOD_US) {
    lastMeasMicros += MEAS_PERIOD_US;
    
    static uint32_t lastCount = 0;
    uint32_t countNow = pulseCount;
    uint32_t pulses = countNow - lastCount;
    lastCount = countNow;
    
    float windowSec = MEAS_PERIOD_US * 1e-6f;
    float freq = pulses / windowSec;
    measRPM = freq * freqToRPM;
    
    if (motorNow - lastPulseMicros > STALL_TIMEOUT_US)
      measRPM = 0.0f;
    
    // low-pass filter
    const float alpha = 0.2f;
    filtRPM = alpha * measRPM + (1.0f - alpha) * filtRPM;
    
    if (pulses > 0)
      digitalWrite(ledPin, !digitalRead(ledPin)); // pulse activity blink
  }
  
  // PI control
  error_motor = setRPM - filtRPM;
  intError += KI_motor * error_motor * dt_motor;
  
  if (intError > 1.0f) intError = 1.0f;
  if (intError < 0.0f) intError = 0.0f;
  
  float u = Kp_motor * error_motor + intError;
  
  if (u > 1.0f) u = 1.0f;
  if (u < 0.0f) u = 0.0f;
  
  int pwmVal = (int)(u * PWM_MAX);
  analogWrite(motorPin, pwmVal);
  
  // Serial plotter output 
  static unsigned long lastPrint = 0;
  if (motorNow - lastPrint >= 1000000) {
    lastPrint = motorNow;
    Serial.print("Motor - Set: ");
    Serial.print(setRPM);
    Serial.print(" RPM, Meas: ");
    Serial.print(measRPM);
    Serial.print(", Filt: ");
    Serial.print(filtRPM);
    Serial.print(", PWM: ");
    Serial.println(pwmVal);
  }
  // end of motor control loop


  // update safe sensor copy for I2C

  noInterrupts();
  sensors_safe.temperature = MeasTemp;
  sensors_safe.pH = currentPh;
  sensors_safe.rpm = filtRPM;
  interrupts();

  delay(1);
}