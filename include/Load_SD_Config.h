/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Read the initial configuration file to activate or deactivate the different modules.
 * 
 */

#ifndef Load_SD_Config_h
#define Load_SD_Config_h

#include <Arduino.h>
#include <IniFile.h>
#include <Ethernet.h>
#include "Configuration.h"
#include "Genenal_functions.h"
#include "MQTT_Pub.h"
#include "DHT_Sensors.h"
#include "DO_Sensor.h"
#include "PH_Sensors.h"
#include "Lux_Sensors.h"
#include "ORP_Sensors.h"
#include "WP_Temp_Sensors.h"
#include "Current_Sensors.h"

//TODO: documentar métodos

bool SD_check_IniFile(IniFile *ini_file);

void SD_load_culture_ID(IniFile *ini, Culture_ID_st *culture_id);

void SD_load_MQTT_config(IniFile *ini, MQTT_Pub *&mqtt_pub, Culture_ID_st *culture_id);

void SD_load_Cnn_type(IniFile *ini, Internet_cnn_type &option);

void SD_load_Eth_config(IniFile *ini, uint8_t *mac);

void SD_load_DHT_sensors(IniFile *ini, DHT_Sensors *sensors);

void SD_load_DO_sensor(IniFile *ini, DO_Sensor *sensor);

void SD_load_pH_sensors(IniFile *ini, PH_Sensors *&sensors);

bool extract_str_params_Lux_sensor(char *str, Lux_Sensors::Lux_Sensor_model_t &model, uint8_t &addr, uint8_t &addr_pin);

void SD_load_Lux_sensors(IniFile *ini, Lux_Sensors *&sensors);

void SD_load_ORP_sensors(IniFile *ini, ORP_Sensors *&sensors);

void SD_load_WP_Temp_sensors(IniFile *ini, WP_Temp_Sensors *&sensors);

bool extract_str_params_Current_sensor(char *str, uint8_t &pin, Current_Sensors::Current_Model_t &model, uint16_t &var);

void SD_load_Current_sensors(IniFile *ini, Current_Sensors *&sensors);


#endif
