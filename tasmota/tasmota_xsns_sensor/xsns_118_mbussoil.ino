#ifdef USE_MBUSSOILSENSOR
#define XSNS_118 118
#define SENSOR_SOILMTEMPERATURE  "Temperature"
#define SENSOR_SOILMHUMIDITY      "Humidity"

typedef struct
{
    unsigned int temperature;
    unsigned int humidity;
} tXsnSensorSettings;
tXsnSensorSettings XsnSensorSettings;

bool SoilMoistureRead() {
  XsnSensorSettings.humidity = 45; // Beispielwert
  XsnSensorSettings.temperature = 22; // Beispielwert

  return true;
}

void SoilMoistureShow(bool json) {
  if (json) {
    ResponseAppend_P(PSTR(",\"%s\":%d,\"%s\":%d"), SENSOR_SOILMHUMIDITY, XsnSensorSettings.humidity, SENSOR_SOILMTEMPERATURE, XsnSensorSettings.temperature);
  } 
#ifdef USE_WEBSERVER
  else {
    AddLog(LOG_LEVEL_INFO, PSTR("SoilMoistureShow web display called"));
    WSContentSend_PD(PSTR("{s}Soil " SENSOR_SOILMHUMIDITY "{m}%u " D_UNIT_PERCENT "{e}"
                          "{s}Soil " SENSOR_SOILMTEMPERATURE "{m}%u " D_UNIT_CELSIUS "{e}"),
                     XsnSensorSettings.humidity,
                     XsnSensorSettings.temperature);
  }
#endif
}

void SensorDriverInit() {
  // UART initialisieren, ggf. RS485 DE/RE Pins setzen
}


bool Xsns118(uint32_t function) {
  switch (function) {
    case FUNC_INIT:
      SensorDriverInit();
      break;
    case FUNC_EVERY_SECOND:
      SoilMoistureRead();
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