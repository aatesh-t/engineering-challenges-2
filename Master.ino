#define USE_EDUROAM 1
#define WIFI_USER "zcabtha@ucl.ac.uk"
#define WIFI_PASS //add password
 
// MQTT
#include <WiFi.h>
#include "esp_eap_client.h"
#include <PubSubClient.h>
#include <Wire.h>
 
//I2C ADDRESS 
#define SLAVE_ADDR 0x28
 
//DATA PACKETS 
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
 
//MQTT CLIENT
WiFiClient espClient;
PubSubClient client(espClient);
 
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
 
//CONNECT TO EDUROAM 
void wifi_connect() {
  Serial.println("Connecting to eduroam...");
 
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
 
#if USE_EDUROAM
  esp_eap_client_set_identity((uint8_t*)WIFI_USER, strlen(WIFI_USER));
  esp_eap_client_set_username((uint8_t*)WIFI_USER, strlen(WIFI_USER));
  esp_eap_client_set_password((uint8_t*)WIFI_PASS, strlen(WIFI_PASS));
  esp_wifi_sta_enterprise_enable();
  WiFi.begin("eduroam");
#else
  WiFi.begin(WIFI_SSID, WIFI_PASS);
#endif
 
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(800);
  }
 
  Serial.println("\n✓ Connected to eduroam");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}
 
//MQTT CALLBACK
void callback(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (int i = 0; i < len; i++) msg += (char)payload[i];
 
  Serial.print("Setpoints received from NodeRED: ");
  Serial.println(msg);
 
  float t, p, s;
  if (sscanf(msg.c_str(), "%f,%f,%f", &t, &p, &s) == 3) {
    setpoints.temp_setpoint = t;
    setpoints.pH_setpoint   = p;
    setpoints.stir_setpoint = s;
 
    //SEND STRUCT OVER I2C
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write((uint8_t*)&setpoints, sizeof(SetpointPacket));
    Wire.endTransmission();
 
    Serial.println("✓ Setpoints sent to Nano via I2C");
  }
}
 
//MQTT RECONNECT LOOP
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32-Bioreactor")) {
      Serial.println("✓ MQTT connected");
      client.subscribe("bioreactor/setpoints");
    } else {
      Serial.println("MQTT failed, retrying...");
      delay(3000);
    }
  }
}
 
//READ I2C SENSOR PACKET FROM NANO
bool readSensors() {
  Wire.requestFrom(SLAVE_ADDR, sizeof(SensorPacket));
 
  uint8_t* p = (uint8_t*)&sensors;
  int i = 0;
 
  while (Wire.available() && i < sizeof(SensorPacket)) {
    p[i++] = Wire.read();
  }
 
  return (i == sizeof(SensorPacket));
}
 
//SETUP
void setup() {
  Serial.begin(115200);
 
  Wire.begin(21, 22);  // SDA = 21, SCL = 22
 
  wifi_connect();
 
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
 
  Serial.println("ESP32 Master Ready.");
}
 
//LOOP
void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();
 
  if (readSensors()) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%.2f,%.2f,%.1f",
             sensors.temperature, sensors.pH, sensors.rpm);
 
    client.publish("bioreactor/data", buf);
    Serial.print("Published to MQTT: ");
    Serial.println(buf);
  }
 
  delay(500);
}