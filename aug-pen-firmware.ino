#include <Wire.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

#define X     0
#define Y     1
#define Z     2
#define ALPHA X
#define BETA  Y
#define GAMMA Z

/* 
  ESP8266 PIN MAP
  ===============

  GND     -->     GND
  GPIO2   -->     MPU6050_SCL
  GPIO0   -->     MPU6050_SDA   (programming mode: LOW before startup)
  RX      -->     
  TX      -->     
  CH_EN   -->     VCC
  RESET   -->     
  VCC     -->     VCC

*/

char buff[100];

const int MPU = 0x68; // I2C Address of MPU-6050 (default)

// WiFi Connection Configuration
const char *wifi_ssid = "Honor 8C";
const char *wifi_password = "16june01";

// MQTT Connection Configuration
const char *mqtt_server = "192.168.43.20";
const char *mqtt_username = "user";
const char *mqtt_password = "iL0v3MoonGaYoung";

PubSubClient mqtt(WiFiClient());

float ACCEL[] = { 0, 0, 0 };
float GYRO[] = { 0, 0, 0 };

void initMPU6050();
void initWiFi();
void initMQTT();
void reconnectMQTT();
void updateSamples();
void debugSamples();
void publishSamples();

void setup() {
  Serial.begin(115200);

  initMPU6050();
  initWiFi();
  initMQTT();

  Serial.println("AUG PEN MK I : boot success.");
  delay(2000);
}

void loop() {
  updateSamples();
  debugSamples();

  mqtt.connected()
      ? mqtt.loop()
      : reconnect();

  publishSamples();

  delay(50);
}

void initMPU6050() {
  Wire.begin(0, 2);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGMT_1 Register
  Wire.write(0);    // Wake the MPU-6050.
  Wire.endTransmission(true);
}

void initWiFi() {
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("Connected. What's my IP? ");
  Serial.println(WiFi.localIP());
}

void initMQTT() {
  mqtt.setServer(mqtt_server, 1883);
  reconnectMQTT();
}

void reconnectMQTT() {
  Serial.print("MQTT Reconnecting");

  while (!mqtt.connected()) {
    if (!mqtt.connect("AUG_PEN_MK_I")) {
      Serial.print("Trying again in 3 seconds...");
      delay(3000);
    }
  }

  Serial.println("Connected to MQTT Server.");
}

void updateSamples() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // ACCEL_XOUT_H Register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // Request 14 bytes
  ACCEL[X]    = ((int) (Wire.read() << 8) | Wire.read()) / 4096.0;       
  ACCEL[Y]    = ((int) (Wire.read() << 8) | Wire.read()) / 4096.0;
  ACCEL[Z]    = ((int) (Wire.read() << 8) | Wire.read()) / 4096.0;  
  GYRO[ALPHA] = ((int) (Wire.read() << 8) | Wire.read()) / 32.8;  
  GYRO[BETA]  = ((int) (Wire.read() << 8) | Wire.read()) / 32.8;  
  GYRO[GAMMA] = ((int) (Wire.read() << 8) | Wire.read()) / 32.8; 
}

void debugSamples() {
  Serial.print("ACX=");   Serial.print(ACCEL[X]);
  Serial.print("\tACY="); Serial.print(ACCEL[Y]);
  Serial.print("\tACZ");  Serial.print(ACCEL[Z]);

  Serial.print("\tGYX="); Serial.print(GYRO[X]);
  Serial.print("\tGYY="); Serial.print(GYRO[Y]);
  Serial.print("\tGYZ="); Serial.print(GYRO[Z]);

  Serial.println();
}

char* floatToString(const float value) {
  dtostrf(value, 4, 2, buff);
  return buff;
}

void publishSamples() {
  mqtt.publish("ACX/", floatToString(ACCEL[X]));
  mqtt.publish("ACY/", floatToString(ACCEL[Y]));
  mqtt.publish("ACZ/", floatToString(ACCEL[Z]));
  mqtt.publish("GYX/", floatToString(GYRO[X]));
  mqtt.publish("GYY/", floatToString(GYRO[Y]));
  mqtt.publish("GYZ/", floatToString(GYRO[Z]));
}
