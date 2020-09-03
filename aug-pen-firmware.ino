/*
  Author: RITHVIK NISHAD
  License: GNU General Public v2.0 (see LICENSE)
*/

// #define RELEASE         // Comment this to compile without including development and diagnostics code for faster performance.
// #define USE_WIFI_MQTT   // Comment this line to disable WiFi and MQTT. (Allows compatability for use w/ non wifi boards for development and testing).
#define TEAPOT_ENABLED  // DOES NOT WORK IN RELEASE MODE. DISABLE RELEASE. Comment this line to disable support for Teapot.

#include <MPU6050_6Axis_MotionApps20.h> // Library to interact w/ MPU6050's DMP engine to offload computation and reduce drift over time in integrated samplings.
// The above library defines: _MPU6050_6AXIS_MOTIONAPPS20_H_ as header guard. Can be used to detect which API is being used for development.

#include <Wire.h>

#ifdef USE_WIFI_MQTT
  #include <ESP8266WiFi.h>
  #include <PubSubClient.h>
#endif

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

#ifdef ESP8266
  #define MPU_INTERRUPT_PIN 3
#else
  #define MPU_INTERRUPT_PIN 2
#endif;

#define MPU6050_I2C_ADDRESS 0x68

#ifndef RELEASE
  #define SERIAL_ENABLED 115200       // Comment to disable Serial communication. INFO and DEBUGGING will also be disabled. Also defines the baud rate.
  
  #define INFO_ENABLED    // Comment to disable INFO messages. (disabling allows working w/ Serial Plotter)
  #define ASSERTS_ENABLED // Comment to disable ASSERT messages.
  #define DEBUG_ENABLED   // Comment to disable DEBUG messages.
  
  #define POST_BOOT_DELAY 2000 // Explicit Post Boot Delay in milli-seconds.
#endif

#define GET_VARIABLE_NAME(var) (#var)

#ifdef SERIAL_ENABLED
  #define SERIAL_OUT_CONST(msg) Serial.print(F(msg));
  #define SERIAL_OUT(var) Serial.print(var);
  #define SERIAL_OUT_LN Serial.println();
#else
  #define SERIAL_OUT_CONST(msg)
  #define SERIAL_OUT(var)
  #define SERIAL_OUT_LN
#endif

#define SERIAL_OUT_PARAM_MANUAL(name, value) SERIAL_OUT_CONST(name) SERIAL_OUT_CONST(": ") SERIAL_OUT(value)
#define SERIAL_OUT_PARAM_AUTO(param) SERIAL_OUT_PARAM_MANUAL(GET_VARIABLE_NAME(param), param);

#define GET_SERIAL_OUT_PARAM_MACRO(_1, _2, NAME, ...) NAME
#define SERIAL_OUT_PARAM(...) GET_SERIAL_OUT_PARAM_MACRO(__VA_ARGS__, SERIAL_OUT_PARAM_MANUAL, SERIAL_OUT_PARAM_MANUAL) (__VA_ARGS__)

#ifdef INFO_ENABLED
  #define INFO(x) SERIAL_OUT_CONST(x) // Use INFO("...") to Serial LOG a compile-time constant string.
  #define INFO_PARAM(...) SERIAL_OUT_PARAM(...)
#endif

#ifdef DEBUG_ENABLED
  #define DEBUG(...) SERIAL_OUT_PARAM(...)
  #define END_DEBUG SERIAL_OUT_LN
#endif

#ifdef ASSERTS_ENABLED
  bool result;

  #define BLOCK_ON_ASSERT_FAIL true // Set to false to disable blocking on ASSERT fail.
  #define ASSERT(x, msg)                     \
    if (!(result = x))                       \
    {                                        \
      SERIAL_OUT_CONST(msg)                  \
      SERIAL_OUT_PARAM("ASSERT_FAILED", x)   \
      while (BLOCK_ON_ASSERT_FAIL) ;         \
    }
#endif

MPU6050 mpu = MPU6050(MPU6050_I2C_ADDRESS);

uint8_t mpuIntStatus;       // Current interrupt status byte.
uint16_t packetSize;        // Expected DMP packet size (42 btes default)
uint16_t fifoCount;         // count of all bytes currently in FIFO.
uint8_t fifoBuffer[64];     // FIFO storage buffer

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; } // Interrupt Service Routine when DMP data is ready.

Quaternion q;        // [w, x, y, z] MPU6050 DMP quaternion container.
VectorInt16 aa;      // [x, y, z] ACCEL_RAW.
VectorInt16 aaReal;  // [x, y, z] Gravity free ACCEL.
VectorInt16 aaWorld; // [x, y, z] World frame ACCEL.
VectorFloat gravity;  // [x, y, z] Gravity vector.
// float euler[3];      // [psi, theta, phi] Euler angles.
float ypr[3];        // [yaw, pitch, roll] angles.

#if defined(TEAPOT_ENABLED) && defined(SERIAL_ENABLED)
  uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};
#endif

#ifdef USE_WIFI_MQTT
  char buff[50]; 
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

#define DEBUG_VECTOR(v) SERIAL_OUT_CONST(GET_VARIABLE_NAME(v)) SERIAL_OUT_CONST(" = [") SERIAL_OUT(v.x) SERIAL_OUT_CONST(", ") SERIAL_OUT(v.y) SERIAL_OUT_CONST(", ") SERIAL_OUT(v.z) SERIAL_OUT_CONST("]\t")
#define DEBUG_ARRAY(a) SERIAL_OUT_CONST(GET_VARIABLE_NAME(a)) SERIAL_OUT_CONST(" = [") SERIAL_OUT(a[0]) SERIAL_OUT_CONST(", ") SERIAL_OUT(a[1]) SERIAL_OUT_CONST(", ") SERIAL_OUT(a[2]) SERIAL_OUT_CONST("]\t")

#define DEBUG_SAMPLES DEBUG_VECTOR(aaWorld) DEBUG_ARRAY(ypr) SERIAL_OUT_LN

void initMPU6050(); // Initialises MPU-6050.

#ifdef USE_WIFI_MQTT
  void initWiFi();                            // Initialises WiFi connectivity.
  void initMQTT();                            // Initialises MQTT Client.
  bool reconnectMQTT(bool firstTime = false); // Reconnects MQTT Client w/ broker.
  void publishSamples();                      // Publish the samples to respective MQTT topics.
#endif

void updateSamples(); // Get and update the samples

void setup() {
  #ifdef SERIAL_ENABLED
    Serial.begin(SERIAL_ENABLED);
  #endif

  initMPU6050();
  
  #ifdef USE_WIFI_MQTT
    initWiFi();
    initMQTT();

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
}

void initMPU6050() {
  INFO("Connecting to MPU-6050...");
  #ifdef ESP8266
    Wire.begin(0, 2);
  #else
    Wire.begin();
  #endif

  Wire.setClock(400000);

  mpu.initialize();
  ASSERT(mpu.testConnection(), "MPU-6050 connection error !")

  pinMode(MPU_INTERRUPT_PIN, INPUT);

  INFO("Initializing Digital Motion Processing Engine...");
  ASSERT(mpu.dmpInitialize() == 0, "DMP Initialization failed.") //  { 0: success, !0: fail }

  INFO("Setting custom offsets...");
  // mpu.setXGyroOffset(220);
  // mpu.setYGyroOffset(76);
  // mpu.setZGyroOffset(-85);
  // mpu.setZAccelOffset(1788);

  INFO("Calibrating Accelerometer and Gyroscope...");
  mpu.CalibrateAccel(12);
  mpu.CalibrateGyro(12);

  INFO("Enabling DMP Engine...");
  mpu.setDMPEnabled(true);

  INFO("Attaching Interrupt Service Routine...");
  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  packetSize = mpu.dmpGetFIFOPacketSize();

  INFO("DMP Engine is ready.");
}

#ifdef USE_WIFI_MQTT

  void initWiFi() {
    INFO("Connecting to WiFi AP...");
    WiFi.begin(wifi_ssid, wifi_password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
  #ifdef INFO_ENABLED
      SERIAL_OUT_CONST('.');
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

void updateSamples() {
  while(!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if (fifoCount < packetSize) {
    
  } else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    INFO("FIFO overflow! Resetting...");
    mpu.resetFIFO();
  } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);

    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    #ifdef TEAPOT_ENABLED

      #define mapBufferToTeapot(tp, fb) teapotPacket[tp] = fifoBuffer[fb];

      mapBufferToTeapot(2, 0);
      mapBufferToTeapot(3, 1);
      mapBufferToTeapot(4, 4);
      mapBufferToTeapot(5, 5);
      mapBufferToTeapot(6, 8);
      mapBufferToTeapot(7, 9);
      mapBufferToTeapot(8, 12);
      mapBufferToTeapot(9, 13);

      Serial.write(teapotPacket, 14); // flushes teapotPacket to Serial.

      teapotPacket[11]++; // teapot packetCount (overflow loop at 0xFF is intended).

    #endif
  }
}
  

#ifdef USE_WIFI_MQTT

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

#endif
