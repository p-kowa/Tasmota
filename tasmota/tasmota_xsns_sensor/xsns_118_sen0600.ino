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
#if !defined(ESP32)
#error "This driver is only supported on ESP32 platforms."
#endif

#ifdef USE_SEN0600
#define XSNS_118 118

// Sensor labels for Tasmota JSON and web output
#define SENSORNAME "SEN0600_"
#define SOILMTEMPERATURE "Temperature"
#define SOILMHUMIDITY "Humidity"
#define SOILMCONDUCTIVITY "Conductivity"
// Default RS485 pin assignments (can be overridden via Tasmota web config)

#define TEMP_CMD_TEMPLATE 0x03, 0x00, 0x01, 0x00, 0x01
#define HUM_CMD_TEMPLATE 0x03, 0x00, 0x00, 0x00, 0x01
#define CONDUCTIVITY_CMD_TEMPLATE 0x03, 0x00, 0x02, 0x00, 0x01
#define DETECTION_CMD_TEMPLATE 0x03, 0x00, 0x00, 0x00, 0x01
#define GETADDRESS_CMD_TEMPLATE 0x03, 0x07, 0xD0, 0x00, 0x01
#define SETADDRESS_CMD_TEMPLATE 0x06, 0x07, 0xD0, 0x00
#define MAX_SENSORS 8

// Define command arrays as constants
uint8_t kTemplateTempRequest[5] = {TEMP_CMD_TEMPLATE};
uint8_t kTemplateHumRequest[5] = {HUM_CMD_TEMPLATE};
uint8_t kTemplateConductivityRequest[5] = {CONDUCTIVITY_CMD_TEMPLATE};
uint8_t kTemplateDetectRequest[5] = {DETECTION_CMD_TEMPLATE};

unsigned int currentSensor = 0;

uint8_t rxPin; // RX pin from Tasmota config
uint8_t txPin; // TX pin from Tasmota config


// Data structure for sensor readings
typedef struct
{
  float temperature;     // Soil temperature in °C (float for precision)
  uint16_t humidity;     // Soil humidity in % (0-100)
  uint16_t conductivity; // Soil conductivity in µS/cm
  uint8_t tempCommand[8];
  uint8_t humCommand[8];
  uint8_t conductivityCommand[8];
} tXsnSensorData;
tXsnSensorData XsnSensorData[MAX_SENSORS];

// Data structure for sensor settings
typedef struct
{
  uint8_t sensorAddresses[MAX_SENSORS]; // Array of sensor addresses
  uint8_t numSensors;
  uint32_t crc32;
} tXsnSensorSettings;
tXsnSensorSettings XsnSensorSettings;

HardwareSerial RS485Serial(1);
bool sensorConnected = false;

// Modbus CRC16 calculation
uint16_t ModbusCRC16(const uint8_t *buf, uint8_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++)
  {
    crc ^= buf[pos];
    for (uint8_t i = 0; i < 8; i++)
    {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void Xsns150SettingsLoad(bool erase)
{
  // *** Start init default values in case file is not found ***
  memset(&XsnSensorSettings, 0x00, sizeof(XsnSensorSettings));
  // Init any other parameter in struct
  XsnSensorSettings.numSensors = 1;
  if (XsnSensorSettings.numSensors > MAX_SENSORS)
  {
    XsnSensorSettings.numSensors = MAX_SENSORS;
  }
  XsnSensorSettings.sensorAddresses[0] = 0x01; // Default sensor address if sensor read fails

#ifndef USE_UFILESYS
  AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: XSNS118 Use defaults as file system not enabled"));
#else
  // Try to load file /.snsset115
  char filename[20];
  // Use for drivers:
  snprintf_P(filename, sizeof(filename), PSTR(TASM_FILE_SENSOR), XSNS_118);
  if (erase)
  {
    TfsDeleteFile(filename); // Use defaults
  }
  else if (TfsLoadFile(filename, (uint8_t *)&XsnSensorSettings, sizeof(tXsnSensorSettings)))
  {
    AddLog(LOG_LEVEL_INFO, PSTR("CFG: XSNS118 loaded from file"));
  }
  else
  {
    // File system not ready: No flash space reserved for file system
    AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: XSNS118 Use defaults as file system not ready or file not found"));
  }
#endif // USE_UFILESYS
}

// Save settings to filesystem if changed
void Xsns150SettingsSave()
{
#ifdef USE_UFILESYS
  // Called from FUNC_SAVE_SETTINGS every SaveData second and at restart
  uint32_t crc32 = GetCfgCrc32((uint8_t *)&XsnSensorSettings + 4, sizeof(tXsnSensorSettings) - 4); // Skip crc32
  if (crc32 != XsnSensorSettings.crc32)
  {
    // Try to save file /.drvset087
    XsnSensorSettings.crc32 = crc32;

    char filename[20];
    // Use for drivers:
    snprintf_P(filename, sizeof(filename), PSTR(TASM_FILE_SENSOR), XSNS_118);
    if (TfsSaveFile(filename, (const uint8_t *)&XsnSensorSettings, sizeof(tXsnSensorSettings)))
    {
      EnsureDefaultSensorAddress();
      AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: XSNS118 saved to file"));
    }
    else
    {
      EnsureDefaultSensorAddress();
      AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: XSNS118 failed to save settings to file, will retry later"));
    }
  }
#endif // USE_UFILESYS
}

void SetCommand(uint8_t sensorAddress, uint8_t requestTemplate[], uint8_t address[])
{
  // Build command: address + template
  address[0] = sensorAddress;
  memcpy(&address[1], requestTemplate, 5); // template is 5 bytes
  // Calculate CRC for first 6 bytes
  uint16_t crc = ModbusCRC16(address, 6);
  address[6] = crc & 0xFF;        // CRC low byte
  address[7] = (crc >> 8) & 0xFF; // CRC high byte
  AddLogBuffer(LOG_LEVEL_DEBUG, address, 8);
}

// Create command arrays for each sensor based on its address
void Xsns150CreateCommands()
{
  // Create command arrays for each sensor based on its address
  for (uint8_t i = 0; i < XsnSensorSettings.numSensors; i++)
  {
    uint8_t address = XsnSensorSettings.sensorAddresses[i];
    SetCommand(address, kTemplateTempRequest, XsnSensorData[i].tempCommand);
    SetCommand(address, kTemplateHumRequest, XsnSensorData[i].humCommand);
    SetCommand(address, kTemplateConductivityRequest, XsnSensorData[i].conductivityCommand);
  }
}

bool SensorSerialPinTest(){
  if(PinUsed(GPIO_RXD) && PinUsed(GPIO_TXD)){
    rxPin = Pin(GPIO_RXD);
    txPin = Pin(GPIO_TXD);
    AddLog(LOG_LEVEL_INFO, PSTR("XSNS118: Using RX pin %u and TX pin %u"), rxPin, txPin);
    return true;
  }
  return false;
}

/*
  SensorDriverInit - Initializes the RS485 serial port and tests sensor connection.
  Called once at startup.
*/
void SensorDriverInit()
{
  if (!SensorSerialPinTest()) {
    AddLog(LOG_LEVEL_ERROR, PSTR("XSNS118: RX or TX pin not configured! Please set them in Module Configuration."));
    return; // Pins not configured, abort initialization
  }
  AddLog(LOG_LEVEL_INFO, PSTR("Initializing Modbus Soil Sensor Driver"));
  Xsns150SettingsLoad(false);
  Xsns150CreateCommands();
  Xsns150SettingsSave();
  // Initialize sensor data to safe defaults
  for (size_t i = 0; i < XsnSensorSettings.numSensors; i++)
  {
    XsnSensorData[i].humidity = 0;
    XsnSensorData[i].temperature = 0;
    XsnSensorData[i].conductivity = 0;
  }

  AddLog(LOG_LEVEL_DEBUG, PSTR("Starting RS485 serial on RX pin %u, TX pin %u"), rxPin, txPin);
  RS485Serial.begin(4800, SERIAL_8N1, rxPin, txPin);
  sensorConnected = testSensorConnection();
}

/*
  testSensorConnection - Sends a Modbus command to check if the sensor responds.
  Returns true if a valid response is received.
*/
bool testSensorConnection()
{
  bool sensorFound = false;
  AddLog(LOG_LEVEL_INFO, PSTR("Nummber of sensors: %u"), XsnSensorSettings.numSensors);
  for (size_t i = 0; i < XsnSensorSettings.numSensors;)
  {
    uint8_t request[8];
    memcpy(request, XsnSensorData[i].humCommand, 8);
    AddLog(LOG_LEVEL_INFO, PSTR("Testing sensor %u with detection humidity command..."), i+1);
    uint8_t response[10];

    while (RS485Serial.available())
    {
      RS485Serial.read();
    }

    AddLogBuffer(LOG_LEVEL_DEBUG, request, 8);

    RS485Serial.write(request, 8);
    RS485Serial.flush();

    delay(200);

    int bytesRead = 0;
    unsigned long timeout = millis() + 1000;
    while (millis() < timeout && bytesRead < 10)
    {
      if (RS485Serial.available())
      {
        response[bytesRead] = RS485Serial.read();
        bytesRead++;
      }
    }

    AddLogBuffer(LOG_LEVEL_DEBUG, response, bytesRead);

    if (bytesRead >= 7 && response[0] == request[0] && response[1] == 0x03 && response[2] == 0x02)
    {
      AddLog(LOG_LEVEL_INFO, PSTR("✓ Valid sensor response detected!"));
      sensorFound = true;
      i++; // Only increment if sensor is valid
    }
    else
    {
      AddLog(LOG_LEVEL_INFO, PSTR("✗ Invalid sensor response. Removing address %u from list."), request[0]);
      // Remove this address from the list and shift others down
      for (uint8_t j = i; j < XsnSensorSettings.numSensors - 1; j++)
      {
        XsnSensorSettings.sensorAddresses[j] = XsnSensorSettings.sensorAddresses[j + 1];
        memcpy(&XsnSensorData[j], &XsnSensorData[j + 1], sizeof(tXsnSensorData));
      }
      XsnSensorSettings.numSensors--;
      // Do not increment i, as we want to check the new sensor at this index
    }
  }
  Xsns150SettingsSave(); // Save updated sensor list
  return sensorFound;
}
/*
  SoilMoistureRead - Reads humidity and temperature from the sensor and updates global values.
  Called every second.
*/
float readSensorData(uint8_t data[], uint8_t sensorAddress)
{

  uint8_t response[10];
  int bytesRead = 0;
  // Clear receive buffer
  readSerial(data, response, bytesRead);

  float readResult = 0.0;
  if (bytesRead >= 7 && response[0] == sensorAddress && response[1] == 0x03 && response[2] == 0x02)
  {
    uint16_t tempRaw = (response[3] << 8) | response[4];
    readResult = tempRaw / 10.0;
  }
  return readResult;
}

void readSerial(uint8_t data[], uint8_t answer[], int &bytesRead)
{
  while (RS485Serial.available())
  {
    RS485Serial.read();
  }

  // Send command
  RS485Serial.write(data, 8);
  RS485Serial.flush();

  delay(200);

  // Read response
  unsigned long timeout = millis() + 1000;

  while (millis() < timeout && bytesRead < 10)
  {
    if (RS485Serial.available())
    {
      answer[bytesRead] = RS485Serial.read();
      bytesRead++;
    }
  }
}

void SoilMoistureRead()
{
  if (sensorConnected)
  {
    XsnSensorData[currentSensor].temperature = readSensorData(XsnSensorData[currentSensor].tempCommand, XsnSensorSettings.sensorAddresses[currentSensor]);
    XsnSensorData[currentSensor].humidity = (unsigned int)readSensorData(XsnSensorData[currentSensor].humCommand, XsnSensorSettings.sensorAddresses[currentSensor]);
    XsnSensorData[currentSensor].conductivity = (unsigned int)readSensorData(XsnSensorData[currentSensor].conductivityCommand, XsnSensorSettings.sensorAddresses[currentSensor]);
    AddLog(LOG_LEVEL_DEBUG, PSTR("Sensor %u - Temp: %.1f °C, Humidity: %u %%, Conductivity: %u µS/cm"), currentSensor, XsnSensorData[currentSensor].temperature, XsnSensorData[currentSensor].humidity, XsnSensorData[currentSensor].conductivity);
  }
  else
  {
    AddLog(LOG_LEVEL_DEBUG, PSTR("Sensor not connected, skipping read."));
  }
  currentSensor++;
  if (currentSensor >= XsnSensorSettings.numSensors)
  {
    currentSensor = 0;
  }
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

const char ModbusSoilCommands[] PROGMEM = "ModbusSoil_|" // Prefix
                                          "GetAddress|SetAddress|AddSensor|DeleteSensor|Help";

void (*const ModbusSoilCommand[])(void) PROGMEM = {
    &CmndModbusSoilGetAddress,
    &CmndModbusSoilSetAddress,
    &CmndModbusSoilAddSensor,
    &CmndModbusSoilDeleteSensor,
    &CmndModbusSoilHelp};

void CmndModbusSoilHelp(void)
{
  Response_P(PSTR("Available commands: ModbusSoil_GetAddress, ModbusSoil_SetAddress, ModbusSoil_AddSensor, ModbusSoil_DeleteSensor, ModbusSoil_Help"));
}

void CmndModbusSoilGetAddress(void)
{
  uint8_t request[8];
  uint8_t response[10];
  int bytesRead = 0;
  uint8_t getAddressCmd[5] = {GETADDRESS_CMD_TEMPLATE};
  SetCommand(0xFF, getAddressCmd, request);
  AddLogBuffer(LOG_LEVEL_DEBUG, request, 8);
  readSerial(request, response, bytesRead);
  AddLogBuffer(LOG_LEVEL_DEBUG, response, bytesRead);
  if (bytesRead > 0)
  {
    uint8_t detectedAddress = response[0];
    Response_P(PSTR("Detected sensor address: %u"), detectedAddress);
  }
  else
  {
    Response_P(PSTR("No response received."));
  }
}

void CmndModbusSoilSetAddress(void)
{
  uint8_t request[8], response[10];
  int bytesRead = 0;

  // Detect current address
  uint8_t getAddressCmd[5] = {GETADDRESS_CMD_TEMPLATE};
  SetCommand(0xFF, getAddressCmd, request);
  readSerial(request, response, bytesRead);

  if (bytesRead > 0)
  {
    uint8_t detectedAddress = response[0];
    ResponseCmndNumber(XdrvMailbox.payload);
    uint8_t newAddress = (uint8_t)XdrvMailbox.payload;

    if ((newAddress > 0) && (newAddress < 256) && (newAddress != detectedAddress))
    {
      // Build set address command template
      uint8_t setAddressCmd[5] = {SETADDRESS_CMD_TEMPLATE};
      setAddressCmd[4] = newAddress;

      SetCommand(detectedAddress, setAddressCmd, request);
      readSerial(request, response, bytesRead);
      AddLogBuffer(LOG_LEVEL_DEBUG, response, bytesRead);
      ResponseCmndNumber(XdrvMailbox.payload);
    }
    else
    {
      Response_P(PSTR("Invalid address. Please enter a number between 1 and 255, different from the current address (%d)."), detectedAddress);
    }
  }
  else
  {
    Response_P(PSTR("No detected address received."));
  }
}

void CmndModbusSoilAddSensor(void)
{
  if (XsnSensorSettings.numSensors < MAX_SENSORS)
  {
    ResponseCmndNumber(XdrvMailbox.payload);
    uint8_t newAddress = (uint8_t)XdrvMailbox.payload;
    if (newAddress == 0 || newAddress > 255 || newAddress == 157)
    {
      ConfiguredSensors();
      return;
    }
    // Check if address is already used
    bool addressExists = false;
    for (uint8_t i = 0; i < XsnSensorSettings.numSensors; i++)
    {
      if (XsnSensorSettings.sensorAddresses[i] == newAddress)
      {
        addressExists = true;
        break;
      }
    }
    if (!addressExists && newAddress > 0 && newAddress < 256)
    {
      XsnSensorSettings.sensorAddresses[XsnSensorSettings.numSensors] = newAddress;
      XsnSensorSettings.numSensors++;
      Xsns150CreateCommands();
      Xsns150SettingsSave();
      Response_P(PSTR("Added sensor with address %u. Total sensors: %u"), newAddress, XsnSensorSettings.numSensors);
    }
    else
    {
      Response_P(PSTR("Invalid or duplicate address. Please enter a number between 1 and 255 that is not already used."));
    }
    EnsureDefaultSensorAddress();
  }
  else
  {
    Response_P(PSTR("Maximum number of sensors (%u) reached."), MAX_SENSORS);
  }
}

void CmndModbusSoilDeleteSensor(void)
{
  if (XsnSensorSettings.numSensors == 0)
  {
    Response_P(PSTR("No sensors found, this shall not happen. At least one sensor shall be configured."));
    return;
  }
  ResponseCmndNumber(XdrvMailbox.payload);
  uint8_t delAddress = (uint8_t)XdrvMailbox.payload;
  if (delAddress == 0 || delAddress > 255 || delAddress == 157)
  {
    ConfiguredSensors();
    return;
  }
  bool addressFound = false;
  for (uint8_t i = 0; i < XsnSensorSettings.numSensors; i++)
  {
    if (XsnSensorSettings.sensorAddresses[i] == delAddress)
    {
      // Prevent deleting the last remaining address
      if (XsnSensorSettings.numSensors == 1)
      {
        Response_P(PSTR("Cannot delete the last sensor address. At least one sensor (default 0x01) must remain."));
        return;
      }
      // Prevent deleting default address 0x01 if it's the only one left
      addressFound = true;
      // Shift remaining addresses down
      for (uint8_t j = i; j < XsnSensorSettings.numSensors - 1; j++)
      {
        XsnSensorSettings.sensorAddresses[j] = XsnSensorSettings.sensorAddresses[j + 1];
      }
      XsnSensorSettings.numSensors--;
      Xsns150CreateCommands();
      Xsns150SettingsSave();
      Response_P(PSTR("Deleted sensor with address %u. Total sensors: %u"), delAddress, XsnSensorSettings.numSensors);
      break;
    }
  }
  // Ensure at least one default sensor address is set
  EnsureDefaultSensorAddress();
  if (!addressFound)
  {
    Response_P(PSTR("Address %u not found among configured sensors."), delAddress);
  }
}

/*************************************************************************************\
* Helper Functions
\*************************************************************************************/

// Ensure at least one default sensor address is set
void EnsureDefaultSensorAddress() {
  if (XsnSensorSettings.numSensors == 1) {
    XsnSensorSettings.sensorAddresses[0] = 0x01;
    for (uint8_t i = 1; i < MAX_SENSORS; i++) {
      XsnSensorSettings.sensorAddresses[i] = 0;
    }
  }
}

// Display configured sensors
void ConfiguredSensors()
{
  if (XsnSensorSettings.numSensors == 0)
  {
    Response_P(PSTR("No sensors configured."));
  }
  else
  {
    Response_P(PSTR("Configured sensors number: (%u): Addresses: "), XsnSensorSettings.numSensors);
    for (uint8_t i = 0; i < XsnSensorSettings.numSensors; i++)
    {
      ResponseAppend_P(PSTR(" %u"), XsnSensorSettings.sensorAddresses[i]);
    }
  }
}

/*********************************************************************************************\
 * Presentation
\*********************************************************************************************/

void SoilMoistureShow(bool json)
{
  for (uint8_t i = 0; i < XsnSensorSettings.numSensors; i++)
  {
    if (json)
    {
      ResponseAppend_P(PSTR(",\"%s%u_Humidity\":%d,\"%s%u_Temperature\":%.1f"),
                 SENSORNAME, i + 1, (int)XsnSensorData[i].humidity,
                 SENSORNAME, i + 1, XsnSensorData[i].temperature);
    }
#ifdef USE_WEBSERVER
    else
    {
      WSContentSend_PD(PSTR("{s}%s%u " SOILMHUMIDITY "{m}%u " D_UNIT_PERCENT "{e}"
                      "{s}%s%u " SOILMTEMPERATURE "{m}%.1f " D_UNIT_CELSIUS "{e}"),
                 SENSORNAME, i + 1, (int)XsnSensorData[i].humidity,
                 SENSORNAME, i + 1, XsnSensorData[i].temperature);
    }
#endif
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns118(uint32_t function)
{
  bool result = false;
  switch (function)
  {
  case FUNC_INIT:
    SensorDriverInit();
    break;
  case FUNC_EVERY_SECOND:
    if (sensorConnected)
    {
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
#endif // USE_WEBSERVER
  case FUNC_COMMAND:
    result = DecodeCommand(ModbusSoilCommands, ModbusSoilCommand);
    break;
  }
  return result;
}
#endif // USE_MBUSSOILSENSOR