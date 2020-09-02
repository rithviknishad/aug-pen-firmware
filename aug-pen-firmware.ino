/*
  Author: RITHVIK NISHAD
  License: GNU General Public v2.0 (see LICENSE)
*/

// #define RELEASE         // Comment this to compile without including development and diagnostics code for faster performance.
// #define USE_WIFI_MQTT   // Comment this line to disable WiFi and MQTT. (Allows compatability for use w/ non wifi boards for development and testing)
// #define TEAPOT_ENABLED  // Comment this line to disable support for Teapot.

#include <MPU6050_6Axis_MotionApps20.h> // Library to interact w/ MPU6050's DMP engine to offload computation and reduce drift over time in integrated samplings.
// The above library defines: _MPU6050_6AXIS_MOTIONAPPS20_H_ as header guard. Can be used to detect which API is being used for development.

#include <Wire.h>

#ifdef USE_WIFI_MQTT
  #include <ESP8266WiFi.h>
  #include <PubSubClient.h>
#endif

#ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_

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
#ifdef _MPU6050_6AXIS_MOTIONAPPS20_H_
  #ifdef ESP8266
    #define MPU_INTERRUPT_PIN 3
  #else
    #define MPU_INTERRUPT_PIN 2
  #endif;
#endif

#define MPU6050_I2C_ADDRESS 0x68

#ifndef RELEASE
  #define SERIAL_ENABLED       // Comment to disable Serial communication. INFO and DEBUGGING will also be disabled.
  #define POST_BOOT_DELAY 2000 // Explicit Post Boot Delay in milli-seconds.
#endif

#ifdef SERIAL_ENABLED
  #define INFO_ENABLED    // Comment to disable INFO messages. (disabling allows working w/ Serial Plotter)
  #define ASSERTS_ENABLED // Comment to disable ASSERT messages.
  #define DEBUG_ENABLED   // Comment to disable DEBUG messages.
  #define SERIAL_PARAM(name, value)                                                                        \
    Serial.print('\t');                                                                                    \
    Serial.print(F(name));                                                                                 \
    Serial.print(value);
#endif

#ifdef INFO_ENABLED
  #define INFO_PARAM(name, value) SERIAL_PARAM(name, value)
  #define INFO(x) Serial.println(F(x)); // Use INFO("...") to Serial LOG a compile-time constant string.
#else
  #define INFO_PARAM(name, val) ;
  #define INFO(x) ;
#endif

#ifdef ASSERTS_ENABLED
bool result;
#define ASSERT(x, msg)                  \
  if (!(result = x))                    \
  {                                     \
    Serial.println(F(msg));             \
    Serial.print(F("ASSERT_RESULT: ")); \
    Serial.print(result);               \
  }
#else
  #define ASSERT(x, msg) ;
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

#ifdef _MPU6050_6AXIS_MOTIONAPPS20_H_
  MPU6050 mpu = MPU6050(MPU6050_I2C_ADDRESS);

  uint8_t dmpReady = false;   // DMP ready status.
  uint8_t mpuIntStatus;       // Current interrupt status byte.
  uint8_t devStatus;          // Status after each operation. [0 = success, !0 = error]
  uint16_t packetSize;        // Expected DMP packet size (42 btes default)
  uint16_t fifoCount;         // count of all bytes currently in FIFO.
  uint8_t fifoBuffer[64];     // FIFO storage buffer

  volatile bool mpuInterrupt = false;
  void dmpDataReady() { mpuInterrupt = true; }

  Quaternion q;        // [w, x, y, z] MPU6050 DMP quaternion container.
  VectorInt16 aa;      // [x, y, z] ACCEL_RAW.
  VectorInt16 aaReal;  // [x, y, z] Gravity free ACCEL.
  VectorInt16 aaWorld; // [x, y, z] World frame ACCEL.
  VectorFloat gravity;  // [x, y, z] Gravity vector.
  float euler[3];      // [psi, theta, phi] Euler angles.
  float ypr[3];        // [yaw, pitch, roll] angles.

#else

  const int MPU = MPU6050_I2C_ADDRESS; // I2C Address of MPU-6050 (default).
  
  char buff[100];                      // A globally shared 100 byte buffer space.
  
  inline unsigned long square(long x) { return x * x; }
  inline float squaref(float x) { return x * x; }

#endif

#ifdef TEAPOT_ENABLED
  #define SERIAL_ENABLED
  uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};
#endif

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

// float ACCEL[3] = {0};  // Current raw acceleration samples from MPU-6050.
// float GYRO[3]  = {0};  // Current raw gyroscope samples from MPU-6050.

// float VELOCITY[3] = {0}; // Current velocity, integrated from ACCEL.

// float POSITION[3]     = {0};  // Current position, integrated from VELOCITY.
// float ORIENTATION[3]  = {0};  // Current orientation, integrated from GYRO.

#define DEBUG_VECTOR_ARRAY(nameX, nameY, nameZ, vect) \
  DEBUG_PARAM(nameX, vect[X]);                        \
  DEBUG_PARAM(nameY, vect[Y]);                        \
  DEBUG_PARAM(nameZ, vect[Z]);

#define DEBUG_VECTOR(nameX, nameY, nameZ, vect) \
  DEBUG_PARAM(nameX, vect.x);                        \
  DEBUG_PARAM(nameY, vect.y);                        \
  DEBUG_PARAM(nameZ, vect.z);

#define DEBUG_SAMPLES                           \
  DEBUG_VECTOR("AX=", "AY=", "AZ=", aa);        \
  DEBUG_VECTOR_ARRAY("GX=", "GY=", "GZ=", ypr); \
  END_DEBUG;
  // DEBUG_VECTOR("VX=", "VY=", "VZ=", VELOCITY);    \
  // DEBUG_VECTOR("PX=", "PY=", "PZ=", POSITION);    \
  // DEBUG_VECTOR("OX=", "OY=", "OZ=", ORIENTATION); \

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

  if (!dmpReady)
    return; // skip loop if dmp failed to load.

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

  #ifdef _MPU6050_6AXIS_MOTIONAPPS20_H_
    Wire.setClock(400000);

    mpu.initialize();
    ASSERT(mpu.testConnection(), "MPU-6050 connection error !")

    pinMode(MPU_INTERRUPT_PIN, INPUT);

    INFO("Initializing Digital Motion Processing Engine...");
    devStatus = mpu.dmpInitialize();
    ASSERT(devStatus == 0, "DMP Initialization failed.")

    INFO("Setting custom offsets...");
    // mpu.setXGyroOffset(220);
    // mpu.setYGyroOffset(76);
    // mpu.setZGyroOffset(-85);
    // mpu.setZAccelOffset(1788);

    INFO("Calibrating Accelerometer and Gyroscope...");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    INFO("Enabling DMP Engine...");
    mpu.setDMPEnabled(true);

    INFO("Attaching Interrupt Service Routine...");
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    packetSize = mpu.dmpGetFIFOPacketSize();

    INFO("DMP Engine is ready.");
    dmpReady = true;

  #else
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); // PWR_MGMT_1 Register
    Wire.write(0);    // Wake the MPU-6050.
    Wire.endTransmission(true);
    
    INFO("MPU-6050 Connected. Status: Unknown");
  #endif  
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

#ifdef _MPU6050_6AXIS_MOTIONAPPS20_H_

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
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aa, &q);

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

#else
  // stores the last sampling time, for measuring delta time between last two samples.
  uint32_t sampling_time[2] = {0};

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
    sampling_time[0] = sampling_time[1];

    // fetch current sampling time.
    sampling_time[1] = millis();

    // check overflow ?? skip this sampling.
    if (sampling_time[1] < sampling_time[0])
      return;

    // get sampling time delta in seconds.
    float delta = (sampling_time[1] - sampling_time[0]) / 1000;

    // compute velocity, position and orientation.
    for (int i = 0; i < 3; ++i) {
      VELOCITY[i] += ACCEL[i] * delta;
      
      POSITION[i] += VELOCITY[i] * delta;
      ORIENTATION[i] += GYRO[i] * delta;
    }
  }

#endif

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
