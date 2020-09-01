/*
  Author: RITHVIK NISHAD
  License: GNU General Public v2.0 (see LICENSE)
  TODO: use micros() instead of millis() to support sampling rate above 1 KHz.
*/

// #define RELEASE       // Comment this to compile without including development and diagnostics code for faster performance.
#define USE_WIFI_MQTT // Comment this line to disable WiFi and MQTT. (Allows compatability for use w/ non wifi boards for development and testing)

#include <Wire.h>

#ifdef USE_WIFI_MQTT
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#endif

/*
  AFS_SEL   FULL SCALE RANGE    LSB SENSITIVITY
  ---------------------------------------------
    0             ±2g            16384 LSB/g
    1             ±4g            8192  LSB/g
    2             ±8g            4096  LSB/g
    3             ±16g           2048  LSB/g
*/
#define AFS_SEL 0
#define ACCEL_SENSITIVITY (16384.0 / pow(2, AFS_SEL)) // LSB/g

/*
  FS_SEL    FULL SCALE RANGE    LSB SENSITIVITY
  ---------------------------------------------
    0         ± 250 °/s           131  LSB/°/s
    1         ± 500 °/s           65.5 LSB/°/s
    2         ± 1000 °/s          32.8 LSB/°/s 
    3         ± 2000 °/s          16.4 LSB/°/s
*/
#define FS_SEL 0
#define GYRO_SENSITIVITY  (131.0 / pow(2, FS_SEL)) // LSB/°/s

/* 
  SCHEMATIC

  FLASH     ESP8266     MPU-6050    ATMEGA328P
  --------------------------------------------
  VCC       VCC         VCC         3v3
  _RST      RST
  _CH_EN    CH_EN       
  _TX       TX
  _RX       -RX(-)      (INT)       (2)
  -GPIO0    GPIO0       SDA         A4
  _GPIO2    GPIO2       SCL         A5
  GND       GND         GND         GND
*/

#ifndef RELEASE
#define SERIAL_ENABLED       // Comment to disable Serial communication. INFO and DEBUGGING will also be disabled.
#define POST_BOOT_DELAY 2000 // Explicit Post Boot Delay in milli-seconds.
#endif

#ifdef SERIAL_ENABLED
#define INFO_ENABLED  // Comment to disable INFO messages.
#define DEBUG_ENABLED // Comment to disable DEBUG messages.
#define SERIAL_PARAM(name, value) \
  Serial.print(F(name));          \
  Serial.print(value);
#endif

#ifdef INFO_ENABLED
#define INFO_PARAM(name, value) SERIAL_PARAM(name, value)
#define INFO(x) Serial.println(F(x)); // Use INFO("...") to Serial LOG a compile-time constant string.
#else
#define INFO_PARAM(name, val) ;
#define INFO(x) ;
#endif

#ifdef DEBUG_ENABLED
#define DEBUG(param)   \
  Serial.print(param); \
  Serial.print('\t');
#define DEBUG_PARAM(name, value) SERIAL_PARAM(name, value)
#define END_DEBUG Serial.println();
#else
#define DEBUG(param) ;
#define DEBUG_PARAM(name, value) ;
#define END_DEBUG ;
#endif

char buff[100];       // A globally shared 100 byte buffer space.

const int MPU = 0x68; // I2C Address of MPU-6050 (default).

#ifdef USE_WIFI_MQTT
// WiFi Connection Configuration
const char *wifi_ssid = "Honor 8C";     // SSID of the WiFi AP to be connected.
const char *wifi_password = "16june01"; // Password of the WiFi AP.

// MQTT Connection Configuration
const char *mqtt_server = "192.168.43.20";  // MQTT Broker Server Address.
const char *mqtt_username = "user"; // MQTT Credentials - Username.
const char *mqtt_password = "iL0v3MoonGaYoung"; // MQTT Credentials - Password.

WiFiClient wifi;  // WiFi Client for MQTT Client.
PubSubClient mqtt(wifi);  // MQTT PubSub Client.
#endif

#define X     0
#define Y     1
#define Z     2
#define ROLL  X
#define YAW   Y
#define PITCH Z

float ACCEL[3] = {0};  // Current raw acceleration samples from MPU-6050.
float GYRO[3]  = {0};  // Current raw gyroscope samples from MPU-6050.

float VELOCITY[3] = {0}; // Current velocity, integrated from ACCEL.

float POSITION[3]     = {0};  // Current position, integrated from VELOCITY.
float ORIENTATION[3]  = {0};  // Current orientation, integrated from GYRO.

#define DEBUG_VECTOR(nameX, nameY, nameZ, vect) \
  DEBUG_PARAM(nameX, vect[X]);                  \
  DEBUG_PARAM(nameY, vect[Y]);                  \
  DEBUG_PARAM(nameZ, vect[Z]);

#define DEBUG_SAMPLES                             \
  DEBUG_VECTOR("AX=", "AY=", "AZ=", ACCEL);       \
  DEBUG_VECTOR("VX=", "VY=", "VZ=", VELOCITY);    \
  DEBUG_VECTOR("PX=", "PY=", "PZ=", POSITION);    \
  DEBUG_VECTOR("GX=", "GY=", "GZ=", GYRO);        \
  DEBUG_VECTOR("OX=", "OY=", "OZ=", ORIENTATION); \
  END_DEBUG;

void initMPU6050(); // Initialises MPU-6050.

#ifdef USE_WIFI_MQTT
void initWiFi();                            // Initialises WiFi connectivity.
void initMQTT();                            // Initialises MQTT Client.
bool reconnectMQTT(bool firstTime = false); // Reconnects MQTT Client w/ broker.
void publishSamples();                      // Publish the samples to respective MQTT topics.
#endif

inline unsigned long square(long x) { return x * x; }
inline float squaref(float x) { return x * x; }

void updateSamples(); // Get and update the samples

void setup() {
#ifdef SERIAL_ENABLED
  Serial.begin(115200);
#endif

  initMPU6050();
  
#ifdef USE_WIFI_MQTT
  initWiFi();
  initMQTT();
#endif

#ifdef USE_WIFI_MQTT
  INFO("AUG PEN MK I : boot_result=0x0, success.");
#else
  INFO("AUG PEN MK I : boot_result=0x1, success.");
  INFO("WiFI and MQTT is disabled. Reason: 'USE_WIFI_MQTT' is not defined.");
#endif

#ifdef POST_BOOT_DELAY
  delay(POST_BOOT_DELAY);
#endif
}

void loop() {
  updateSamples();
  DEBUG_SAMPLES;

#ifdef USE_WIFI_MQTT
  mqtt.connected()
      ? mqtt.loop()
      : reconnectMQTT();

  publishSamples();
#endif

  delay(100);
}

void initMPU6050() {
  INFO("Connecting to MPU-6050...");
  Wire.begin(0, 2);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGMT_1 Register
  Wire.write(0);    // Wake the MPU-6050.
  Wire.endTransmission(true);
  INFO("MPU-6050 Connected. Status: Unknown");
}

#ifdef USE_WIFI_MQTT
void initWiFi() {
  INFO("Connecting to WiFi AP...");
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#ifdef INFO_ENABLED
    Serial.print('.');
#endif
  }
  INFO_PARAM("WiFi Connected. What's my IP? ", WiFi.localIP());
}

void initMQTT() {
  INFO("Connecting to MQTT Broker Server...");
  mqtt.setServer(mqtt_server, 1883);
  reconnectMQTT(true);
}

bool reconnectMQTT(bool firstTime) {
  if (!firstTime)
    INFO("MQTT Reconnecting...");

  while (!mqtt.connected()) {
    if (!mqtt.connect("AUG_PEN_MK_I", mqtt_username, mqtt_password)) {
      INFO_PARAM("MQTT connection unsuccessfull. Status: ", mqtt.state());
      INFO("Trying again in 3 seconds...");
      delay(3000);
    }
  }
  INFO_PARAM("MQTT Connected. Status: ", mqtt.state());
  return mqtt.connected();
}
#endif

// stores the last sampling time, for measuring delta time between last two samples.
uint32_t t0 = 0;
uint32_t t1 = 0;

void updateSamples() {
  // request 14 bytes of data from register 0x3B (ACCEL_XOUT_H).
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  
  // Read all ACCEL registers (2 * 3 bytes) and store in m/s^2.
  for (int i = 0; i < 3; ++i)
    ACCEL[i] = (((Wire.read() << 8) | Wire.read()) / ACCEL_SENSITIVITY) * 9.80655;

  // Read temperature register (2 bytes) and store as degree celsius.
  float temp = (((int)(Wire.read() << 8) | Wire.read()) / 340.0) + 36.53;

  // Read all GRYO regsters (2 * 3 bytes) and store in deg/s.
  for (int i = 0; i < 3; ++i)
    GYRO[i] = ((Wire.read() << 8) | Wire.read()) / GYRO_SENSITIVITY;

  // move previous sampling time.
  t0 = t1;

  // fetch current sampling time.
  t1 = millis();

  // check overflow ?? skip this sampling.
  if (t1 < t0)
    return;

  // get sampling time delta in seconds.
  float delta = (t1 - t0) / 1000;

  // compute velocity, position and orientation.
  for (int i = 0; i < 3; ++i) {
    VELOCITY[i] += ACCEL[i] * delta;
    
    POSITION[i] += VELOCITY[i] * delta;
    ORIENTATION[i] += GYRO[i] * delta;
  }
}

char* floatToString(const float value) {
  dtostrf(value, 4, 2, buff);
  return buff;
}

#ifdef USE_WIFI_MQTT
void publishSamples() {
  mqtt.publish("position/X", floatToString(POSITION[X]));
  mqtt.publish("position/Y", floatToString(POSITION[Y]));
  mqtt.publish("position/Z", floatToString(POSITION[Z]));

  mqtt.publish("orientation/X", floatToString(ORIENTATION[X]));
  mqtt.publish("orientation/Y", floatToString(ORIENTATION[Y]));
  mqtt.publish("orientation/Z", floatToString(ORIENTATION[Z]));
}
#endif
