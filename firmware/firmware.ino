/*
  Author: RITHVIK NISHAD & KARTHIK P AJITHKUMAR
  License: GNU General Public v2.0 (see LICENSE)

  Made for college project: Standalone 3D Position Sensor.
  As part of: EEE2004 Measurements and Instrumentation course.
*/

// #define WIFI_MQTT_ENABLED   // Comment this line to disable WiFi and MQTT. (Allows compatability for use w/ non wifi boards for development and testing).

#include <Wire.h>

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


/* 
           SENSITIVITY OF ACCELEROMETER
    (refer MPU6050 datasheet for more details)
    

  AFS_SEL   FULL SCALE RANGE    LSB SENSITIVITY
  ---------------------------------------------
    0             ±2g            16384 LSB/g
    1             ±4g            8192  LSB/g
    2             ±8g            4096  LSB/g
    3             ±16g           2048  LSB/g
*/
#define AFS_SEL 3

const int ACCEL_SENSITIVITY = 16384 / pow(2, AFS_SEL);  // Sensitivity of the Accelerometer. (LSB/g)

#define ONLY_X_AXIS             1                       // If selected in AXES, computation will be performed only for (x).
#define ONLY_XY_AXIS            2                       // If selected in AXES, computation will be performed only for (x, y).
#define ONLY_XYZ_AXIS           3                       // If selected in AXES, computation will be performed only for (x, y, z).

#define AXES                    ONLY_XYZ_AXIS           // Vector dimension.

#define CALIBRATION_ENABLED                             // Enables calibration for gravity compensation at start-up.

#if defined(CALIBRATION_ENABLED)
  void calibrate();
  int16_t RAW_ACCEL_OFFSET[AXES] = {0};                 // RAW Acceleration offset. (calibration)

  #define CALIBRATION_SAMPLES   1000                    // No. of samples to be taken while calibrating.
  
#else
  void calibrate() {}
#endif


#define OP_RATE               100                       // Processing and output rate. (packets per second)
#define OP_DELTA_MICROS       1e+6 / OP_RATE            // Output frame interval. (microseconds)
#define OP_DELTA_SECONDS      1.0 / OP_RATE             // Output frame interval. (seconds)

#define SERIAL_ENABLED        115200                    // Serial Buad Rate. (Higher, faster; max: 2000000)
                                                        // Enables serial activity. (Comment to disable)
                                                        // Disabling will also disable Info & Plot functionalities.

#define INFO_ENABLED                                    // Enables INFO & DEBUG messages. (Comment to disable)

#define PLOT_ENABLED                                    // Enables support for plot. (Comment to disable)
                                                        // Serial out in CSV format.

#define PLOT_BY_ASCII         1                         // Plot data is sent in ASCII.
                                                        // Useful for plotting and reading serial simultaneously.

#define PLOT_BY_BINARY        0                         // Plot data is sent in binary.
                                                        // Faster performance, since 64-byte buffer is truncated less frequently,
                                                        // and also because no translations are performed on the data.

#define PLOT_MODE             PLOT_BY_ASCII             // Sets the plot mode to be used.

#define POST_BOOT_DELAY       1000                      // Explicit post boot delay. (milliseconds)
#define MPU_STARTUP_DELAY     0                         // Explicit post MPU power-up delay. (milliseconds)
                                                        // (for compensating MPU startup accuracy ramp) (Comment to disable)

#define COMPUTE_VELOCITY                                // Enables velocity computation. (Comment to disable)
#define COMPUTE_POSITION                                // Enables position computation. (Comment to disable)


#if defined(COMPUTE_POSITION)
  float _displacement[AXES] = {0.0};                    // Stores the current estimated position vector. (m)
  #define POSITION _displacement
  #define COMPUTE_VELOCITY                              // Force enables velocity computation, as position estimation requires velocity data.
#endif

#if defined(COMPUTE_VELOCITY)
  float _velocity[AXES] = {0.0};                        // Stores the current estimated velocity vector. (m/s)
  #define VELOCITY _velocity
#endif

int32_t RAW_ACCEL[AXES] = {0};                          // RAW Acceleration sample space.
uint32_t samples = 0;                                   // No of samples in RAW_ACCEL sample space.

uint32_t lastOPTime = millis();                         // micros() value from last succesful output. (micro-seconds)


int i;  // A globally shared iterator, to reduce dynamic mem usage in synchronous execution.
        // Make sure deep function invocations do not end up in a nested usage resulting in conflicts.
        // Use for top-level functions.


#if defined(SERIAL_ENABLED)
  #define SOC(msg) Serial.print(F(msg));                // Serial writes a compile time constant value w/o loading it to RAM.
  #define SO(var) Serial.print(var);                    // Prints `var` to serial.
  #define SLN Serial.println();                         // Prints a new line to serial.
#else
  #define SOC(msg)
  #define SO(var)
  #define SLN
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
  #define __comma   SOC(",")
  #define __plot(a) SO(a) __comma

  #define PLOT_ON   SO(millis()) __comma

  template <class T>
  void plot3(const T* vector) {
    for (i = 0; i < AXES; ++i) {
      __plot(vector[i]);
    }
  }

  #define PLOT_OFF      SLN
#endif

#if defined(WIFI_MQTT_ENABLED)                          // WiFi & MQTT related configurations
  char buff[50]; 
  
  const char *wifi_ssid = "Honor 8C";                   // WiFi AP SSID.
  const char *wifi_password = "16june01";               // WiFi AP security key.

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
void requestAcceleration();                             // Requests MPU-6050 to send 6 bytes of RAW_ACCEL. Use Wire.read() to read each byte after invocation.

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

  INFO("S3DPS: Warming up...")
  
  initMPU6050();

  #if defined(WIFI_MQTT_ENABLED)
    INFO("'WIFI_MQTT_ENABLED': true");

    initWiFi();
    initMQTT();
  #else
    INFO("'WIFI_MQTT_ENABLED': false");
  #endif

  INFO("S3DPS: boot() -> success.");
  
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
  Serial.print("[2J");  // CLRSCR command.
  Serial.write(27);     // ESC command.
  Serial.print("[H");   // HOME command.
}

void resetSampling() {
  lastOPTime = micros();
  samples = 0;
  for (i = 0; i < AXES; ++i)
    RAW_ACCEL[i] = 0;
}

void initMPU6050() {
  INFO("Connecting to MPU-6050...");

#if defined(ESP8266)                                    // Initialize TwoWire / I2C bus.
  Wire.begin(0, 2);
#else
  Wire.begin();
#endif

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);          // on device: MPU6050
  Wire.write(0x6B);                                     // register: PWR_MGT_1
  Wire.write(0);                                        // RESET -> Wake
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);          // on device: MPU6050
  Wire.write(0x1C);                                     // register: AFS_SEL
  Wire.write(AFS_SEL << 3);                             // write: selected AFS_SEL
  Wire.endTransmission(true);

  #if defined(MPU_STARTUP_DELAY)
  delay(MPU_STARTUP_DELAY);                             // perform MPU_STARTUP_DELAY if defined.
#endif
}

void requestAcceleration() {
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);          // on device: MPU6050
  Wire.write(0x3B);                                     // register: ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_I2C_ADDRESS, 6, true);       // request register[0:6].
}

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

  char* floatToString(const float value) { dtostrf(value, 4, 2, buff); return buff; }
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

float accel[AXES] = {0.0};
void publish() {
  if (micros() - lastOPTime < OP_DELTA_MICROS) return;  // Continues execution only if it's time to publish data (as defined by OP_RATE)

  for (i = 0; i < AXES; ++i) {
    accel[i] = RAW_ACCEL[i] * 9.80665 / (samples * ACCEL_SENSITIVITY);

    #if defined(COMPUTE_VELOCITY)
      VELOCITY[i] += accel[i] * OP_DELTA_SECONDS;
    #endif

    #if defined(COMPUTE_POSITION)
      POSITION[i] += VELOCITY[i] * OP_DELTA_SECONDS;
    #endif
  }

  #if defined(PLOT_ENABLED)
    PLOT_ON
    plot3(accel);
    plot3(VELOCITY);
    plot3(POSITION);
    PLOT_OFF
  #endif

  resetSampling();

  #if defined(WIFI_MQTT_ENABLED)
    mqtt.connected() 
      ? mqtt.loop()
      : reconnectMQTT();
  
    // mqtt.publish("position/X", floatToString(POSITION[1]));
    // mqtt.publish("position/Y", floatToString(POSITION[2]));
    // mqtt.publish("position/Z", floatToString(POSITION[3]));
  #endif
}
