/*
  Authors: 
    * RITHVIK NISHAD        https://github.com/rithviknishad
    * KARTHIK P AJITHKUMAR  https://github.com/karthikpaji

  License: GNU General Public v2.0 (see LICENSE)
    
    * Permissions:            * Limitations:            * Conditions:
      * Commercial use.         * No liability.           * License & copyright notice.
      * Modifications.          * No warranty.            * State changes.
      * Distribution.                                     * Disclose source.
      * Private use.                                      * Same license.

  Repository: https://github.com/rithviknishad/standalone-3d-position-sensor

  College course project: Standalone 3D Position Sensor.
  Course: Measurements and Instrumentation.
*/


// #define WIFI_MQTT_ENABLED                             // Enables WiFi & MQTT functionalities. (Comment to disable)

#include <Wire.h>

#include <MPU6050_6Axis_MotionApps20.h>                 // Enables computation using MPU6050's DMP engine. (Comment to disable)

#if defined(WIFI_MQTT_ENABLED)
  #include <ESP8266WiFi.h>
  #include <PubSubClient.h>
#endif

/* 
                      IO MAP                    

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
#define MPU6050_I2C_ADDRESS 0x68                        // I2C address of MPU-6050 device.

#ifdef ESP8266
  #define MPU_INTERRUPT_PIN 3
#else
  #define MPU_INTERRUPT_PIN 2
#endif;


#if defined(_MPU6050_6AXIS_MOTIONAPPS20_H_)
  #define DMP_ENGINE_ENABLED
  MPU6050 mpu = MPU6050(MPU6050_I2C_ADDRESS);
#endif


/* 
           ACCELEROMETER SENSITIVITY          
   (refer MPU-6050 datasheet for more details)
    

  AFS_SEL   FULL SCALE RANGE    LSB SENSITIVITY
  ---------------------------------------------
    0             ±2g            16384 LSB/g
    1             ±4g            8192  LSB/g
    2             ±8g            4096  LSB/g
    3             ±16g           2048  LSB/g
*/
#define AFS_SEL 0

#if defined(DMP_ENGINE_ENABLED)
  #define AFS_SEL 0                                     // Overrides custom sensitivity if DMP_ENGINE_ENABLED.
#endif

const int ACCEL_SENSITIVITY = 16384 / pow(2, AFS_SEL);  // Sensitivity of the Accelerometer. (LSB/g)

#define ONLY_X_AXIS             1                       // If selected in AXES, computation will be performed only for (x).
#define ONLY_XY_AXIS            2                       // If selected in AXES, computation will be performed only for (x, y).
#define ONLY_XYZ_AXIS           3                       // If selected in AXES, computation will be performed only for (x, y, z).

#define AXES                    ONLY_XY_AXIS            // Vector dimension. (Setting is not used if DMP_ENGINE_ENABLED is defined)

#if defined(DMP_ENGINE_ENABLED)
  #define AXES                  ONLY_XYZ_AXIS           // if DMP_ENGINE_ENABLED, overrides AXES to ONLY_XYZ_AXIS.
#endif

#define CALIBRATION_ENABLED                             // Enables gravity compensation by calibrating at start-up. (Comment to disable)

#if defined(DMP_ENGINE_ENABLED)
  #undef CALIBRATION_ENABLED                            // Overrides and purges CALIBRATION_ENABLED, if DMP_ENGINE_ENABLED is defined.
#endif

#if defined(CALIBRATION_ENABLED)
  void calibrate();
  VectorInt16 RAW_ACCEL_OFFSET = VectorInt16();         // RAW Acceleration offset. (calibration)
  #define CALIBRATION_SAMPLES   1000                    // No. of samples to be taken while calibrating.
#else
  void calibrate() {}
#endif


#define OP_RATE               1000                      // Processing and output rate. (packets per second)
#define OP_DELTA_MICROS       1e+6 / OP_RATE            // Output frame interval. (microseconds)
#define OP_DELTA_SECONDS      1.0 / OP_RATE             // Output frame interval. (seconds)

#define SERIAL_ENABLED        2000000                   // Serial Buad Rate. (Higher, faster; max: 2000000)
                                                        // Enables serial activity. (Comment to disable)
                                                        // Disabling will also disable Info & Plot functionalities.

#define RUNTIME_ASSERTS_ENABLED                         // Enables assertion during runtime.
#define BLOCKING_ASSERT       false                     // Blocks execution if assert(...) failed. (Set to false to disable)

#define INFO_ENABLED                                    // Enables INFO & DEBUG messages. (Comment to disable)

#define PLOT_ENABLED                                    // Enables support for plot. (Comment to disable)
                                                        // Serial out in CSV format.

#define PLOT_BY_ASCII         1                         // Plot data is sent in ASCII.
                                                        // Useful for plotting and reading serial simultaneously.
                                                        // Tradeoff: Slow performance, Frequently truncates buffer.

#define PLOT_BY_BINARY        0                         // Plot data is sent in binary.
                                                        // Faster performance, since AVR's 64-byte serial buffer shall be truncated less frequently,
                                                        // and also because no translations have to be performed on the plot data.
                                                        // Tradeoff: Non-human readable via terminal.

#define PLOT_MODE             PLOT_BY_BINARY            // Sets the plot mode to be used.

#define MPU_STARTUP_DELAY     0                         // Explicit post MPU power-up delay. (milliseconds)
                                                        // For compensating MPU startup accuracy ramp. (Comment to disable)
                                                        // Overriden and disabled if DMP_ENGINE_ENABLED as DMP engine has in-built calibration program.

#define POST_BOOT_DELAY       1000                      // Explicit post boot delay. (milliseconds) 
                                                        // Allows to see runtime configurations before clrscr is invoked.

#if defined(DMP_ENGINE_ENABLED)
  #define MPU_STARTUP_DELAY   0                         // Overrides and sets MPU_STARTUP_DELAY = 0, if DMP_ENGINE_ENABLED is defined.
#endif

#define ACCEL_SUPPRESSION     0                         // WORLD_ACCEL suppression bound value.
                                                        // Values b/w +/- ACCEL_SUPPRESSION are considered to be 0 to eliminate small magnituded noise contributing to drift.

#define COMPUTE_VELOCITY                                // Enables velocity computation. (Comment to disable)
#define COMPUTE_POSITION                                // Enables position computation. (Comment to disable)


#if defined(COMPUTE_POSITION)
  VectorFloat POSITION = VectorFloat();                 // Stores the current estimated position vector. (m)
  #define COMPUTE_VELOCITY                              // Force enables velocity computation, as position estimation requires velocity data.
#endif

#if defined(COMPUTE_VELOCITY)
  VectorFloat VELOCITY = VectorFloat();                 // Stores the current estimated velocity vector. (m/s)
#endif

VectorInt16 RAW_ACCEL = VectorInt16();                  // RAW Acceleration sample space.

uint32_t lastOPTime = millis();                         // micros() value from last succesful output. (micro-seconds)
uint32_t lastSampleTime = 0;

int i;  // A globally shared iterator, to reduce dynamic mem usage in synchronous execution.
        // Make sure deep function invocations do not end up in a nested usage resulting in conflicts.
        // Use for top-level functions.


#if defined(DMP_ENGINE_ENABLED)
  uint8_t mpuIntStatus;                                 // Current interrupt status byte.
  uint16_t packetSize;                                  // Expected DMP packet size. (default: 42 bytes)
  uint16_t fifoCount;                                   // Count of all bytes currently in FIFO.
  uint8_t fifoBuffer[64];                               // FIFO storage buffer.

  volatile bool mpuInterrupt = false;
  void dmpDataReady() { mpuInterrupt = true; }          // Interrupt Service Routine on DMP data ready.

  Quaternion q;                                         // MPU6050 DMP quaternion container.
  float ypr[3];                                         // Orientation angles (yaw, pitch, roll).
  float euler[3];                                       // Euler angles (psi, theta, phi).
  VectorFloat gravityVector;                            // Gravity vector.
  VectorInt16 REAL_ACCEL;                               // Gravity free ACCEL.
  VectorInt16 WORLD_ACCEL;                              // Gravity + orientation corrected ACCEL / World frame ACCEL.
  VectorFloat ACCEL;                                    // ACCEL in m/s^2
#endif


#if defined(SERIAL_ENABLED)
  #define SOC(msg) Serial.print(F(msg));                // Serial writes a compile time constant value w/o loading it to RAM.
  #define SO(var) Serial.print(var);                    // Prints `var` to serial.
  #define SLN Serial.println();                         // Prints a new line to serial.
#else
  #define SOC(msg)
  #define SO(var)
  #define SLN
#endif


#if defined(RUNTIME_ASSERTS_ENABLED)
  bool result;                                          // Holds the result of the current assertion.

  #define ASSERT(x, msg)                            \
    if (!(result = x))                              \
    {                                               \
      SOC(msg)                                      \
      SOC("  Return Code: ")                        \
      SO(x)                                         \
      if (BLOCKING_ASSERT) {                        \
        SOC("\r\n Program terminated.")             \
      }                                             \
      while (BLOCKING_ASSERT) ;                     \
    }

#else 
  #define ASSERT(x, msg) 
#endif


#if defined(INFO_ENABLED)
  #define _INFO(x) SOC(x)
  #define INFO(x) _INFO(x) SLN
  #define INFO_PARAM(name, value) SOC(name) SOC(": ") SO(value) SLN
  #define INFO_VAR(x) SO(x)
#else
  #define _INFO(x)
  #define INFO(x)
  #define INFO_PARAM(name, value)
  #define INFO_VAR(x)
#endif

#if defined(PLOT_ENABLED)

  #define SW(b)   Serial.write(b);                      // Serial writes binary data.

  #define SOF     SW(0x00) SW(0x00) SW(0x0C) SW(0x74)   // Start of frame byte.
  #define DELIM   SO(",")                               // Channel Delimiter.
  #define EOF     SW('\r') SW('\n')                     // End of frame byte.

  #define START_PLOT    SOF
  #define CHANNEL_NEXT  DELIM
  #define END_PLOT      SLN

  #if (PLOT_MODE == PLOT_BY_BINARY)
    void plot(float v) {
      Serial.write((char *)&v, sizeof(float));
    }
  #elif (PLOT_MODE == PLOT_BY_ASCII)
    #define plot(val) SO(val) DELIM
  #endif

#endif

#if defined(WIFI_MQTT_ENABLED)                          // WiFi & MQTT related configurations
  char buff[64]; 
  
  const char *wifi_ssid = "Honor 8C";                   // WiFi AP SSID.
  const char *wifi_password = "16june01";               // WiFi AP security key.
  
  // TODO: rewrite wifi_password w/ dummy value on git push using actions.
  // TODO: or, use compile time define to define password. throw compile-error if not defined.
  // TODO: or, embed a serial com program which acquires ssid and key, and store in EEPROM. (program launch on startup w/ timeout)

  const char *mqtt_server = "192.168.43.20";            // MQTT broker address.
  const char *mqtt_username = "user";                   // Secure MQTT username.
  const char *mqtt_password = "iL0v3MoonGaYoung";       // Secure MQTT password.

  WiFiClient wifi;                                      // WiFi client instance.
  PubSubClient mqtt(wifi);                              // MQTT client instance.

  void initWiFi();                                      // Initialize WiFi connectivity.
  void initMQTT();                                      // Initialize MQTT client.
  bool reconnectMQTT();                                 // Attempts MQTT client reconnect w/ broker.
#endif

void clrscr();                                          // Clears terminal serial screen.
                                                        //  - Tested w/ Putty terminal.
                                                        //  - Does not interfere w/ Serial Plot functionality.

void resetSampling();                                   // Resets variables associated w/ sampling.

void initMPU6050();                                     // Initializes & boots MPU-6050 device.

void sample();                                          // Retrieves raw samples at the highest possible rate. (Does not process the sample)
void publish();                                         // Publishes the samples to serial and MQTT topics if enabled.



void setup() {
  #if defined(SERIAL_ENABLED)
    Serial.begin(SERIAL_ENABLED);                       // Initializes Serial communication.
    clrscr();                                           // Clears terminal screen after flushing existing serial out buffer.
  #endif

  SLN SLN
  INFO("Standalone 3D Position Sensor");
  INFO("version: 0.3-alpha");
  INFO("Licensed under the GNU General Public License v2.0");
  SLN SLN

  INFO("booting...")
  
  initMPU6050();

  #if defined(WIFI_MQTT_ENABLED)
    INFO("'WIFI_MQTT_ENABLED': true");

    initWiFi();
    initMQTT();
  #else
    INFO("'WIFI_MQTT_ENABLED': false");
  #endif

  INFO("Boot complete.");
  
  calibrate();

  #if defined(POST_BOOT_DELAY)
    delay(POST_BOOT_DELAY);
  #endif

  clrscr();

  resetSampling();
}


void loop() {
  sample();
  publish();
}



void clrscr() {
  Serial.write(27);     // ESC command.
  Serial.print(F("[2J"));  // CLRSCR command.
  Serial.write(27);     // ESC command.
  Serial.print(F("[H"));   // HOME command.
}

void resetSampling() {
  VELOCITY.x = 0;
  VELOCITY.y = 0;
  VELOCITY.z = 0;

  POSITION.x = 0;
  POSITION.y = 0;
  POSITION.z = 0;

  lastSampleTime = micros();
}

#if defined(DMP_ENGINE_ENABLED)
  void initDMPEngine() {
    mpu.initialize();
    ASSERT(mpu.testConnection(), "MPU-6050 connection error!");

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
#endif

void initMPU6050() {
  INFO("Connecting to MPU-6050...");

  #if defined(ESP8266)                                  // Initialize TwoWire / I2C bus.
    Wire.begin(0, 2);
  #else
    Wire.begin();
  #endif

  Wire.setClock(400000);                                // Set I2C bus speed: 400 KHz

  #if defined(DMP_ENGINE_ENABLED)
    initDMPEngine();
  #else
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);        // on device: MPU6050
    Wire.write(0x6B);                                   // register: PWR_MGT_1
    Wire.write(0);                                      // RESET -> Wake
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_I2C_ADDRESS);        // on device: MPU6050
    Wire.write(0x1C);                                   // register: AFS_SEL
    Wire.write(AFS_SEL << 3);                           // write: selected AFS_SEL
    Wire.endTransmission(true);
  #endif

  #if defined(MPU_STARTUP_DELAY)
    delay(MPU_STARTUP_DELAY);                           // perform MPU_STARTUP_DELAY if defined.
  #endif
}

#if !defined(DMP_ENGINE_ENABLED)
  void requestAcceleration() {
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);        // on device: MPU6050
    Wire.write(0x3B);                                   // register: ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_I2C_ADDRESS, 6, true);     // request register[0:6].
  }
#endif

#if defined(WIFI_MQTT_ENABLED)
  void initWiFi() {
    INFO("Connecting to WiFi AP...");
    WiFi.begin(wifi_ssid, wifi_password);               // Initializes WiFi to connect to specified SSID & password.

    while (WiFi.status() != WL_CONNECTED) {             // Waits for connection success. (Blocking code)
      delay(500);
      _INFO('.');
    }

    INFO_PARAM("WiFi Connected. What's my IP? ", WiFi.localIP());
  }

  void initMQTT() {
    INFO("Connecting to MQTT Broker Server...");
    mqtt.setServer(mqtt_server, 1883);
    reconnectMQTT(true);
  }

  bool reconnectMQTT() {
    while (!mqtt.connected()) {
      if (!mqtt.connect("SAPOS3", mqtt_username, mqtt_password)) {
        INFO_PARAM("MQTT connection unsuccessfull. Status: ", mqtt.state());
        INFO("Trying again in 3 seconds...");
        delay(3000);
      }
    }
    INFO_PARAM("MQTT Connected. Status: ", mqtt.state());
    return mqtt.connected();
  }

  char *floatToString(const float value); { dtostrf(value, 4, 2, buff); return buff; }
#endif

#if defined(CALIBRATION_ENABLED)
  void calibrate() {
    INFO("Calbirating...");
    
    uint32_t acc[AXES] = {0};

    for (samples = 0; samples < CALIBRATION_SAMPLES; ++samples)
    {
      requestAcceleration();
      for (i = 0; i < AXES; ++i)
        acc[i] += (uint32_t)((Wire.read() << 8) | Wire.read());
    }

    _INFO("Calibration OK. RAW_ACCEL_OFFSET = [")
    for (i = 0; i < AXES; ++i) {
      RAW_ACCEL_OFFSET[i] = acc[i] / CALIBRATION_SAMPLES;
      
      INFO_VAR(RAW_ACCEL_OFFSET[i]);
      _INFO(", ");
    }
    INFO("].");
  }
#endif

#if defined(DMP_ENGINE_ENABLED)
  void sample() {
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
      mpu.dmpGetAccel(&RAW_ACCEL, fifoBuffer);

      mpu.dmpGetGravity(&gravityVector, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravityVector);

      mpu.dmpGetLinearAccel(&REAL_ACCEL, &RAW_ACCEL, &gravityVector);
      mpu.dmpGetLinearAccelInWorld(&WORLD_ACCEL, &REAL_ACCEL, &q);

      // if (-ACCEL_SUPPRESSION < WORLD_ACCEL.x && WORLD_ACCEL.x < ACCEL_SUPPRESSION) WORLD_ACCEL.x = 0;
      // if (-ACCEL_SUPPRESSION < WORLD_ACCEL.y && WORLD_ACCEL.y < ACCEL_SUPPRESSION) WORLD_ACCEL.y = 0;
      // if (-ACCEL_SUPPRESSION < WORLD_ACCEL.z && WORLD_ACCEL.z < ACCEL_SUPPRESSION) WORLD_ACCEL.z = 0;

      ACCEL.x = WORLD_ACCEL.x * 9.80665 / ACCEL_SENSITIVITY;
      ACCEL.y = WORLD_ACCEL.y * 9.80665 / ACCEL_SENSITIVITY;
      ACCEL.z = WORLD_ACCEL.z * 9.80665 / ACCEL_SENSITIVITY;

      uint32_t current_micros = micros();

      float delta = (current_micros - lastSampleTime) / 1e+6;

      if (lastSampleTime == 0) {
        lastSampleTime = current_micros;
        return;
      }

      VELOCITY.x += ACCEL.x * delta;      
      VELOCITY.y += ACCEL.y * delta;
      VELOCITY.z += ACCEL.z * delta;      

      POSITION.x += VELOCITY.x * delta;
      POSITION.y += VELOCITY.y * delta;
      POSITION.z += VELOCITY.z * delta;

      lastSampleTime = current_micros;

    }
  }
#else
  void sample() {
    requestAcceleration();

    for (i = 0; i < AXES; ++i) {
      #if defined(CALIBRATION_ENABLED)
          RAW_ACCEL[i] += ((Wire.read() << 8) | Wire.read()) - RAW_ACCEL_OFFSET[i];
      #else
          RAW_ACCEL[i] += (Wire.read() << 8) | Wire.read();
      #endif
    }

    ++samples;
  }
#endif

#if defined(DMP_ENGINE_ENABLED)
  void __spfPublish() {
    START_PLOT

    plot(ACCEL.x * 100);
    plot(ACCEL.y * 100);
    // plot(ACCEL.z * 100);

    plot(VELOCITY.x * 100);
    plot(VELOCITY.y * 100);
    // plot(VELOCITY.z * 100);

    plot(POSITION.x * 100);
    plot(POSITION.y * 100);
    // plot(POSITION.z * 100);
  }
#else
  void __spfPublish() {
    for (i = 0; i < AXES; ++i) {
      accel[i] = RAW_ACCEL[i] * 9.80665 / (samples * ACCEL_SENSITIVITY);

      if (-ACCEL_SUPPRESSION < accel[i] && accel[i] < ACCEL_SUPPRESSION)
        accel[i] = 0.0;

      #if defined(COMPUTE_VELOCITY)
        VELOCITY[i] += accel[i] * OP_DELTA_SECONDS;
      #endif

      #if defined(COMPUTE_POSITION)
        POSITION[i] += VELOCITY[i] * OP_DELTA_SECONDS;
      #endif
    }

    #if defined(PLOT_ENABLED)
      PLOT_ON
      plot(accel);
      plot(VELOCITY);
      plot(POSITION);
      PLOT_OFF
    #endif
  }
#endif

void publish() {
  uint32_t current_micros = micros();

  if (current_micros - lastOPTime < OP_DELTA_MICROS) return;  // Continues execution only if it's time to publish data (as defined by OP_RATE)

  if (Serial.available() > 0) {
    while(Serial.available()) {
      if (Serial.read() == 'r')
        resetSampling();
    }
  }

  __spfPublish();

  #if defined(WIFI_MQTT_ENABLED)
    mqtt.connected() 
      ? mqtt.loop()
      : reconnectMQTT();
  
    mqtt.publish("position/X", floatToString(POSITION.x));
    mqtt.publish("position/Y", floatToString(POSITION.y));
    mqtt.publish("position/Z", floatToString(POSITION.z));
  #endif

  lastOPTime = current_micros;
}
