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

// Modbus command templates
#define DETECTION_CMD_TEMPLATE 0x03, 0x00, 0x00, 0x00, 0x01
#define GETADDRESS_CMD_TEMPLATE 0x03, 0x07, 0xD0, 0x00, 0x01
#define SETADDRESS_CMD_TEMPLATE 0x06, 0x07, 0xD0, 0x00
#define MAX_SENSORS 8
#define MAX_REGISTER_NUMBER 5

unsigned int currentSensor = 0;

uint8_t rxPin; // RX pin from Tasmota config
uint8_t txPin; // TX pin from Tasmota config

// Data structure for sensor readings
typedef struct
{
  float temperature;     // Soil temperature in °C (float for precision)
  uint16_t humidity;     // Soil humidity in % (0-100)
  uint16_t conductivity; // Soil conductivity in µS/cm
  uint8_t sensorFound;  // Flag indicating if sensor data was successfully read
} tXsnSensorData;
tXsnSensorData XsnSensorData[MAX_SENSORS];

// Data structure for sensor settings
typedef struct
{
  uint8_t sensorAddresses[MAX_SENSORS]; // Array of sensor addresses
  uint8_t registerNumber;
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
  XsnSensorSettings.registerNumber = 2;
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
  testSensorConnection - Validates that all configured sensors respond to Modbus commands.
  Logs any non-responsive sensors but does NOT remove them from configuration.
  User can manually delete non-responsive sensors using ModbusSoil_DeleteSensor command.
  Called once during driver initialization (FUNC_INIT).
  Returns: true if at least one valid sensor was found, false if all sensors are unreachable.
*/
bool testSensorConnection()
{
  bool sensorFound = false;
  const uint8_t testQuantity = 1;
  
  AddLog(LOG_LEVEL_INFO, PSTR("XSNS118: Testing %u configured sensor(s)..."), XsnSensorSettings.numSensors);
  
  for (uint8_t i = 0; i < XsnSensorSettings.numSensors; i++)
  {
    uint8_t sensorAddress = XsnSensorSettings.sensorAddresses[i];
    
    // Build test command (Modbus function 0x03 to read 1 humidity register 0x0000)
    uint8_t request[8];
    request[0] = sensorAddress;
    uint8_t detectionTemplate[5] = {DETECTION_CMD_TEMPLATE};
    memcpy(&request[1], detectionTemplate, 5);
    
    // Calculate and append CRC
    uint16_t crc = ModbusCRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    
    AddLog(LOG_LEVEL_DEBUG, PSTR("XSNS118: Testing sensor %u (address: %u)..."), i + 1, sensorAddress);
    AddLogBuffer(LOG_LEVEL_DEBUG, request, 8);
    
    // Clear serial buffer before sending
    while (RS485Serial.available())
    {
      RS485Serial.read();
    }
    
    // Send test command
    RS485Serial.write(request, 8);
    RS485Serial.flush();
    
    delay(200);  // Wait for sensor response
    
    // Read response
    uint8_t response[10];
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
    
    // Validate response: [Address][Function 0x03][ByteCount][Data1][Data2][CRC_L][CRC_H]
    uint8_t expectedByteCount = testQuantity * 2;
    if (bytesRead >= 7 && response[0] == sensorAddress && response[1] == 0x03 && response[2] == expectedByteCount)
    {
      // Validate CRC
      uint16_t calcCRC = ModbusCRC16(response, bytesRead - 2);
      uint16_t recvCRC = (response[bytesRead-1] << 8) | response[bytesRead-2];
      
      if (calcCRC == recvCRC)
      {
        AddLog(LOG_LEVEL_INFO, PSTR("✓ Sensor %u (address: %u) responded successfully!"), i + 1, sensorAddress);
        XsnSensorData[i].sensorFound = 1;
      }
      else
      {
        AddLog(LOG_LEVEL_INFO, PSTR("⚠ Sensor %u (address: %u) - CRC validation failed. Use 'ModbusSoil_DeleteSensor %u' to remove."), i + 1, sensorAddress, sensorAddress);
        XsnSensorData[i].sensorFound = 0;
      }
    }
    else
    {
      AddLog(LOG_LEVEL_INFO, PSTR("⚠ Sensor %u (address: %u) did not respond. Use 'ModbusSoil_DeleteSensor %u' to remove."), i + 1, sensorAddress, sensorAddress);
      XsnSensorData[i].sensorFound = 0;
    }
  }
  
  for (uint8_t i = 0; i < XsnSensorSettings.numSensors; i++)
  {
    if (XsnSensorData[i].sensorFound)
    {
      sensorFound = true;
      break;
    }
  }

  if (sensorFound)
  {
    AddLog(LOG_LEVEL_INFO, PSTR("XSNS118: Sensor connection test completed. %u sensor(s) available."), XsnSensorSettings.numSensors);
  }
  else
  {
    AddLog(LOG_LEVEL_ERROR, PSTR("XSNS118: No sensors responded during connection test!"));
  }
  
  return sensorFound;
}
/*
  readSerial - Generic serial I/O for Modbus communication.
  Sends a request and reads the response with flexible sizing.
  Parameters:
    - request: Modbus request buffer
    - requestLen: Length of request (typically 8 bytes)
    - response: Buffer to store response
    - bytesRead: Reference variable to track bytes received
    - expectedLen: Expected response length (varies with register quantity)
  Returns: true if exact expectedLen bytes received, false on timeout/mismatch
*/
bool readSerial(uint8_t* request, uint8_t requestLen, uint8_t* response, int& bytesRead, int expectedLen)
{
  bytesRead = 0;
  while (RS485Serial.available()) RS485Serial.read();
  RS485Serial.write(request, requestLen);      // ← FLEXIBLE: any request length
  RS485Serial.flush();
  delay(100);                                   
  
  unsigned long timeout = millis() + 1000;
  while (millis() < timeout && bytesRead < expectedLen)  // ← FLEXIBLE: any response length
  {
    if (RS485Serial.available())
    {
      response[bytesRead] = RS485Serial.read();
      bytesRead++;
    }
  }
  return (bytesRead == expectedLen);  // ← Returns success/failure
}

/*
  ReadSensorRegisters - Read multiple registers in one Modbus call (function 0x03).
  Parameters:
    - sensorAddress: Target sensor slave ID (0x01-0xFF)
    - quantity: Number of registers to read (1-MAX_REGISTER_NUMBER=5)
    - registers: Output array of uint16_t values
  Returns: bool - True if response valid and CRC correct, false otherwise
  Process:
    1. Build Modbus function 0x03 request
    2. Call readSerial() with dynamic expected length
    3. Validate response header (address, function, byte count)
    4. Validate CRC on full response
    5. Extract and return register values
*/
bool ReadSensorRegisters(uint8_t sensorAddress, uint8_t quantity, uint16_t* registers)
{
  if (quantity == 0 || quantity > MAX_REGISTER_NUMBER) 
    return false;
  
  // Build Modbus request: Function 0x03 (Read Holding Registers)
  uint8_t request[8];
  request[0] = sensorAddress;
  request[1] = 0x03;           // Function code
  request[2] = 0x00;           // Start address high byte
  request[3] = 0x00;           // Start address low byte
  request[4] = 0x00;           // Quantity high byte
  request[5] = quantity;       // Quantity low byte
  
  uint16_t crc = ModbusCRC16(request, 6);
  request[6] = crc & 0xFF;
  request[7] = (crc >> 8) & 0xFF;
  
  // Send request and read response
  uint8_t response[11];  // Max: 1+1+1+(5*2)+2 = 11 bytes
  int bytesRead = 0;
  int expectedLen = 5 + quantity * 2;
  
  if (!readSerial(request, 8, response, bytesRead, expectedLen))
    return false;
  
  // Validate response header
  if (response[0] != sensorAddress) return false;
  if (response[1] != 0x03) return false;
  if (response[2] != quantity * 2) return false;
  
  // Validate CRC
  uint16_t calcCRC = ModbusCRC16(response, bytesRead - 2);
  uint16_t recvCRC = (response[bytesRead-1] << 8) | response[bytesRead-2];
  if (calcCRC != recvCRC) return false;
  
  // Extract registers
  for (uint8_t i = 0; i < quantity; i++)
  {
    int dataIdx = 3 + (i * 2);
    registers[i] = (response[dataIdx] << 8) | response[dataIdx + 1];
  }
  
  return true;
}

/*
  SoilMoistureRead - Poll the next sensor in round-robin sequence.
  Only reads sensors that passed the connection test.
  Skips non-responsive sensors but still cycles through them.
  Called from FUNC_EVERY_SECOND callback (~1Hz).
*/
void SoilMoistureRead()
{
  if (!sensorConnected)
    return;
  
  // Only attempt to read if this sensor responded during init test
  if (XsnSensorData[currentSensor].sensorFound)
  {
    uint16_t registers[MAX_REGISTER_NUMBER];
    
    if (ReadSensorRegisters(XsnSensorSettings.sensorAddresses[currentSensor], 
                            XsnSensorSettings.registerNumber, 
                            registers))
    {
      // Register 0 = Humidity * 10
      XsnSensorData[currentSensor].humidity = (registers[0] > 0) ? registers[0] / 10 : 0;
      
      // Register 1 = Temperature * 10
      if (XsnSensorSettings.registerNumber >= 2)
        XsnSensorData[currentSensor].temperature = (registers[1] > 0) ? registers[1] / 10.0 : 0;
      
      // Register 2 = Conductivity (if requested)
      if (XsnSensorSettings.registerNumber >= 3)
        XsnSensorData[currentSensor].conductivity = (registers[2] > 0) ? registers[2] : 0;
      
      AddLog(LOG_LEVEL_DEBUG, PSTR("Sensor %u - Temp: %.1f °C, Humidity: %u %%, Conductivity: %u µS/cm"), 
             currentSensor, XsnSensorData[currentSensor].temperature, 
             XsnSensorData[currentSensor].humidity, XsnSensorData[currentSensor].conductivity);
    }
    else
    {
      AddLog(LOG_LEVEL_DEBUG, PSTR("Sensor %u: read failed."), currentSensor);
    }
  }
  // else: sensor not found, skip it but still advance to next
  
  // Always move to next sensor for next second
  currentSensor++;
  if (currentSensor >= XsnSensorSettings.numSensors)
    currentSensor = 0;
}

/*************************************************************************************\
* Helper Functions
\*************************************************************************************/

void SetCommand(uint8_t sensorAddress, const uint8_t requestTemplate[], uint8_t address[])
{
  // Build command: address + template
  address[0] = sensorAddress;
  memcpy(&address[1], requestTemplate, 5); // template is 5 bytes
  // Calculate CRC for first 6 bytes
  uint16_t crc = ModbusCRC16(address, 6);
  address[6] = crc & 0xFF;        // CRC low byte
  address[7] = (crc >> 8) & 0xFF; // CRC high byte
}

void EnsureDefaultSensorAddress() 
{
  if (XsnSensorSettings.numSensors == 1) {
    XsnSensorSettings.sensorAddresses[0] = 0x01;
    for (uint8_t i = 1; i < MAX_SENSORS; i++) {
      XsnSensorSettings.sensorAddresses[i] = 0;
    }
  }
}

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
 * Commands
\*********************************************************************************************/

const char ModbusSoilCommands[] PROGMEM = "ModbusSoil_|" // Prefix
                                          "GetAddress|SetAddress|SetRegisters|AddSensor|DeleteSensor|Help";

void (*const ModbusSoilCommand[])(void) PROGMEM = {
    &CmndModbusSoilGetAddress,
    &CmndModbusSoilSetAddress,
    &CmndModbusSoilSetRegisters,
    &CmndModbusSoilAddSensor,
    &CmndModbusSoilDeleteSensor,
    &CmndModbusSoilHelp};

void CmndModbusSoilHelp(void)
{
  Response_P(PSTR("Available commands: ModbusSoil_GetAddress, ModbusSoil_SetAddress, ModbusSoil_SetRegisters, ModbusSoil_AddSensor, ModbusSoil_DeleteSensor, ModbusSoil_Help"));
}

void CmndModbusSoilGetAddress(void)
{
  uint8_t request[8];
  uint8_t response[10];
  int bytesRead = 0;
  uint8_t getAddressCmd[5] = {GETADDRESS_CMD_TEMPLATE};
  SetCommand(0xFF, getAddressCmd, request);
  AddLogBuffer(LOG_LEVEL_DEBUG, request, 8);
  readSerial(request, 8, response, bytesRead, 10);
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
  readSerial(request, 8, response, bytesRead, 10);

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
      readSerial(request, 8, response, bytesRead, 10);
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

void CmndModbusSoilSetRegisters(void)
{
  ResponseCmndNumber(XdrvMailbox.payload);
  uint8_t registerNumber = (uint8_t)XdrvMailbox.payload;
  if (registerNumber > 0 && registerNumber <= MAX_REGISTER_NUMBER)
  {
    XsnSensorSettings.registerNumber = registerNumber;
    Xsns150SettingsSave();
    Response_P(PSTR("Set register number to %u."), registerNumber);
  }
  else
  {
    Response_P(PSTR("Invalid register number. Please enter 1, 2, 3, 4, or 5."));
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
      addressFound = true;
      // Shift remaining addresses down
      for (uint8_t j = i; j < XsnSensorSettings.numSensors - 1; j++)
      {
        XsnSensorSettings.sensorAddresses[j] = XsnSensorSettings.sensorAddresses[j + 1];
      }
      XsnSensorSettings.numSensors--;
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