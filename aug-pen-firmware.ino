#include <ESP8266WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define X     0
#define Y     1
#define Z     2

/* 
  SCHEMATIC

  ESP8266     MPU-6050    FLASH
 --------------------------------
  GND         --------GND--------
  GPIO2       SCL         _GPIO2
  GPIO0       SDA         _GPIO1
  RX          INT         _RX
  TX          --------_TX--------
  CH_EN       --------VCC--------
  RST         --------RST--------
  VCC         --------VCC--------
*/

char buff[100]; // A globally shared 100 byte buffer space.

const int MPU = 0x68; // I2C Address of MPU-6050 (default).

// WiFi Connection Configuration
const char *wifi_ssid = "Honor 8C";     // SSID of the WiFi AP to be connected.
const char *wifi_password = "16june01"; // Password of the WiFi AP.

// MQTT Connection Configuration
const char *mqtt_server = "192.168.43.20";  // MQTT Broker Server Address.
const char *mqtt_username = "user"; // MQTT Credentials - Username.
const char *mqtt_password = "iL0v3MoonGaYoung"; // MQTT Credentials - Password.

WiFiClient wifi;  // WiFi Client for MQTT Client.
PubSubClient mqtt(wifi);  // MQTT PubSub Client.

float ACCEL[3] = {0};  // Current raw acceleration samples from MPU-6050.
float GYRO[3]  = {0};  // Current raw gyroscope samples from MPU-6050.

float VELOCITY[3] = {0};
float ANGULAR_VELOCITY[3] = {0};

float POSITION[3]     = {0};  // Current position, double integrated from ACCEL.
float ORIENTATION[3]  = {0};  // Current orientation, double integrated from GYRO.

void initMPU6050();
void initWiFi();
void initMQTT();
bool reconnectMQTT();
inline unsigned long square(long x) { return x * x; }
inline float squaref(float x) { return x * x; }
void updateSamples();
void debugSamples();
void publishSamples();

void setup() {
  Serial.begin(115200);

  initMPU6050();
  initWiFi();
  initMQTT();

  Serial.println(F("AUG PEN MK I : boot success."));
  delay(2000);
}

void loop() {
  updateSamples();
  debugSamples();

  mqtt.connected()
      ? mqtt.loop()
      : reconnectMQTT();

  publishSamples();

  delay(200);
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
    Serial.print('.');
  }

  Serial.print(F("Connected. What's my IP? "));
  Serial.println(WiFi.localIP());
}

void initMQTT() {
  mqtt.setServer(mqtt_server, 1883);
  reconnectMQTT();
}

bool reconnectMQTT() {
  Serial.print(F("MQTT Reconnecting"));

  while (!mqtt.connected()) {
    if (!mqtt.connect("AUG_PEN_MK_I", mqtt_username, mqtt_password)) {
      Serial.println(F("MQTT connection unsuccessfull."));
      Serial.println(mqtt.state());
      Serial.println(F("Trying again in 3 seconds..."));
      delay(3000);
    } else {
      Serial.println(F("MQTT connection successfull."));
    }
  }

  Serial.println(F("Connected to MQTT Server."));

  return mqtt.connected();
}

float t0 = 0;

void updateSamples() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // ACCEL_XOUT_H Register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // Request 14 bytes

  for (int i = 0; i < 3; ++i)
    ACCEL[i] = (Wire.read() << 8) | Wire.read();

  int temp = (Wire.read() << 8) | Wire.read();

  for (int i = 0; i < 3; ++i)
    GYRO[i] = (Wire.read() << 8) | Wire.read();

  float delta = (millis() / 1000) - t0;
  float delta_square = squaref(delta);

  

  t0 += delta;
}

void debugSamples() {
  Serial.print(F("\tpX="));  Serial.print(POSITION[X]);
  Serial.print(F("\tpY="));  Serial.print(POSITION[Y]);
  Serial.print(F("\tpZ="));   Serial.print(POSITION[Z]);

  Serial.print(F("\toX="));  Serial.print(ORIENTATION[X]);
  Serial.print(F("\toY="));  Serial.print(ORIENTATION[Y]);
  Serial.print(F("\toZ="));  Serial.print(ORIENTATION[Z]);

  Serial.println();
}

char* floatToString(const float value) {
  dtostrf(value, 4, 2, buff);
  return buff;
}

void publishSamples() {
  mqtt.publish("position/X", floatToString(POSITION[X]));
  mqtt.publish("position/Y", floatToString(POSITION[Y]));
  mqtt.publish("position/Z", floatToString(POSITION[Z]));

  mqtt.publish("orientation/X", floatToString(ORIENTATION[X]));
  mqtt.publish("orientation/Y", floatToString(ORIENTATION[Y]));
  mqtt.publish("orientation/Z", floatToString(ORIENTATION[Z]));
}
