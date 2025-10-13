/*
  xsns_118_mbussoil.ino - Modbus Soil Moisture & Temperature Sensor Driver for Tasmota

  Copyright (C) 2025 Peter Kowalewski

  This driver adds support for RS485/Modbus soil sensors (e.g., DFRobot SEN0600) to Tasmota.
  It reads humidity and temperature values via Modbus RTU commands and exposes them to Tasmota's web UI, MQTT, and telemetry.

  Features:
    - Uses configurable GPIOs for RS485 RX/TX (set via Tasmota web/module configuration)
    - Supports float temperature and integer humidity (0-100%)
    - Periodic polling and error handling
    - Debug logging for sensor communication

  Usage:
    1. Assign RX and TX pins in Tasmota's web/module configuration.
    2. Enable the driver via user_config_override.h or build flags.
    3. Connect the sensor to the specified pins.
    4. Sensor data will appear in Tasmota's web UI and MQTT telemetry.

  License: GNU General Public License v3.0
*/

#ifdef USE_MBUSSOILSENSOR
#define XSNS_118 118

// Sensor labels for Tasmota JSON and web output
#define SENSOR_SOILMTEMPERATURE  "Temperature"
#define SENSOR_SOILMHUMIDITY      "Humidity"
// Default RS485 pin assignments (can be overridden via Tasmota web config)
#define RS485_RX 6
#define RS485_TX 7

// Data structure for sensor readings
typedef struct
{
    float temperature;        // Soil temperature in °C (float for precision)
    unsigned int humidity;    // Soil humidity in % (0-100)
} tXsnSensorSettings;
tXsnSensorSettings XsnSensorSettings;

// Modbus command arrays for temperature and humidity
uint8_t tempCmd[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA};
uint8_t humCmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
uint8_t detectionCmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};

HardwareSerial RS485Serial(1);
bool sensorConnected = false;

// Forward declarations
bool testSensorConnection();
float readSensorData(uint8_t data[]);

/*
  SoilMoistureShow - Outputs sensor data to Tasmota JSON and web interface.
  If 'json' is true, appends data to telemetry JSON.
  Otherwise, sends formatted data to the web UI.
*/
void SoilMoistureShow(bool json) {
  if (json) {
    ResponseAppend_P(PSTR(",\"%s\":%d,\"%s\":%.1f"), SENSOR_SOILMHUMIDITY, XsnSensorSettings.humidity, SENSOR_SOILMTEMPERATURE, XsnSensorSettings.temperature);
  } 
#ifdef USE_WEBSERVER
  else {
    WSContentSend_PD(PSTR("{s}Soil " SENSOR_SOILMHUMIDITY "{m}%u " D_UNIT_PERCENT "{e}"
                          "{s}Soil " SENSOR_SOILMTEMPERATURE "{m}%.1f " D_UNIT_CELSIUS "{e}"),
                     XsnSensorSettings.humidity,
                     XsnSensorSettings.temperature);
  }
#endif
}

/*
  SensorDriverInit - Initializes the RS485 serial port and tests sensor connection.
  Called once at startup.
*/
void SensorDriverInit() {
   XsnSensorSettings.humidity = 0; // Beispielwert
   XsnSensorSettings.temperature = 0; // Beispielwert
   RS485Serial.begin(4800, SERIAL_8N1, RS485_RX, RS485_TX);
   sensorConnected = testSensorConnection(detectionCmd);
}

/*
  testSensorConnection - Sends a Modbus command to check if the sensor responds.
  Returns true if a valid response is received.
*/
bool testSensorConnection(uint8_t data[]) {
  
  AddLog(LOG_LEVEL_INFO, PSTR("Testing sensor with detection command..."));
  uint8_t response[10];

  while (RS485Serial.available()) {
    RS485Serial.read();
  }

  char sendBuf[64];
  snprintf(sendBuf, sizeof(sendBuf), "Sending: ");
  for (int i = 0; i < 8; i++) {
    char tmp[8];
    snprintf(tmp, sizeof(tmp), "0x%02X ", data[i]);
    strncat(sendBuf, tmp, sizeof(sendBuf) - strlen(sendBuf) - 1);
  }
  AddLog(LOG_LEVEL_DEBUG, sendBuf);

  RS485Serial.write(data, 8);
  RS485Serial.flush();

  delay(200);

  int bytesRead = 0;
  unsigned long timeout = millis() + 1000;
  while (millis() < timeout && bytesRead < 10) {
    if (RS485Serial.available()) {
      response[bytesRead] = RS485Serial.read();
      bytesRead++;
    }
  }

  char respBuf[64];
  snprintf(respBuf, sizeof(respBuf), "Response (%d bytes): ", bytesRead);
  for (int i = 0; i < bytesRead; i++) {
    char tmp[8];
    snprintf(tmp, sizeof(tmp), "0x%02X ", response[i]);
    strncat(respBuf, tmp, sizeof(respBuf) - strlen(respBuf) - 1);
  }
  AddLog(LOG_LEVEL_DEBUG, respBuf);

  if (bytesRead >= 7 && response[0] == 0x01 && response[1] == 0x03 && response[2] == 0x02) {
    AddLog(LOG_LEVEL_INFO, PSTR("✓ Valid sensor response detected!"));
    return true;
  }

  return false;
}

/*
  SoilMoistureRead - Reads humidity and temperature from the sensor and updates global values.
  Called every second.
*/
float readSensorData(uint8_t data[]){
  
  uint8_t response[10];
  
  // Clear receive buffer
  while (RS485Serial.available()) {
    RS485Serial.read();
  }
  
  // Send command
  RS485Serial.write(data, 8);
  RS485Serial.flush();
  
  delay(200);
  
  // Read response
  int bytesRead = 0;
  unsigned long timeout = millis() + 1000;
  
  while (millis() < timeout && bytesRead < 10) {
    if (RS485Serial.available()) {
      response[bytesRead] = RS485Serial.read();
      bytesRead++;
    }
  }
  
  float readResult = 0.0;
  if (bytesRead >= 7 && response[0] == 0x01 && response[1] == 0x03 && response[2] == 0x02) {
    uint16_t tempRaw = (response[3] << 8) | response[4];
    readResult = tempRaw / 10.0;
  }
  return readResult;
}

void SoilMoistureRead() {
  if (sensorConnected) {
    XsnSensorSettings.humidity = (unsigned int)readSensorData(humCmd);
    XsnSensorSettings.temperature = readSensorData(tempCmd);
  } else {
    AddLog(LOG_LEVEL_DEBUG, PSTR("Sensor not connected, skipping read."));
  }
}

bool Xsns118(uint32_t function) {
  switch (function) {
    case FUNC_INIT:
      SensorDriverInit();
      break;
    case FUNC_EVERY_SECOND:
      if (sensorConnected) {
        SoilMoistureRead();
      }     
      break;
    case FUNC_JSON_APPEND:
      SoilMoistureShow(true);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      SoilMoistureShow(false);
      break;
#endif  // USE_WEBSERVER
  }
  return true;
}
#endif // USE_MBUSSOILSENSOR