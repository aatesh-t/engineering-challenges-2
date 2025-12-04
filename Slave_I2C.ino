#include <Wire.h>
 
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
 
void receiveSetpoints(int count) {

  if (count == sizeof(SetpointPacket)) {

    Wire.readBytes((char*)&setpoints, sizeof(SetpointPacket));
 
    Serial.println("=== SETPOINTS RECEIVED ===");

    Serial.println(setpoints.temp_setpoint);

    Serial.println(setpoints.pH_setpoint);

    Serial.println(setpoints.stir_setpoint);

  }

}
 
void sendSensors() {

  // fill the sensor struct

  sensors.temperature = 25.0;

  sensors.pH = 7.0;

  sensors.rpm = 180;
 
  Wire.write((uint8_t*)&sensors, sizeof(SensorPacket));

}
 
void setup() {

  Serial.begin(115200);
 
  Wire.begin((uint8_t)SLAVE_ADDR);

  Wire.onReceive(receiveSetpoints);

  Wire.onRequest(sendSensors);
 
  Serial.println("Nano I2C slave ready");

}
 
void loop() {

  delay(100);

}
