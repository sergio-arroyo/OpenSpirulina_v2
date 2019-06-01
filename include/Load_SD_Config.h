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
#include "OS_Actuators.h"


/**
 * Open Ini config file and check if it's in correct format
 * 
 * @param ini_file The full path where the file are stored on the SD card
 * @return true if the file contain the correct format, otherwise returns false 
 **/
bool SD_check_IniFile(IniFile *ini_file);

/**
 * Load the culture indentification
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param culture_id Structure where store the culture data
 **/
void SD_load_culture_ID(IniFile *ini, Culture_ID_st *culture_id);

/**
 * Load the MQTT informatio to send data to remote broker
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param mqtt_pub MQTT_Pub tructure that will contain the data of the MQTT broker
 * @param culture_id Structure where store the culture data
 **/
void SD_load_MQTT_config(IniFile *ini, MQTT_Pub *&mqtt_pub, Culture_ID_st *culture_id);

/**
 * Load the connection type to use to send data to remote server
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param option The variable where to store the connection type
 **/
void SD_load_Cnn_type(IniFile *ini, Internet_cnn_type &option);

/**
 * Load the Ethernet initial configuration
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param mac The array where to store the MAC address
 **/
void SD_load_Eth_config(IniFile *ini, uint8_t *mac);

/**
 * Load the DHT sensors initial configuration
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param sensors The DHT_Sensors object where to add the DHT sensors
 **/
void SD_load_DHT_sensors(IniFile *ini, DHT_Sensors *sensors);

/**
 * Load the DO sensor initial configuration
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param sensor The DO_Sensor object where to add the DO sensor
 **/
void SD_load_DO_sensor(IniFile *ini, DO_Sensor *sensor);

/**
 * Load the PH sensors initial configuration
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param sensors The PH_Sensors object where to add the PH sensor
 **/
void SD_load_pH_sensors(IniFile *ini, PH_Sensors *&sensors);

/**
 * Extract the specific configuration for a Lux sensor from a text string
 * 
 * @param str Initial string from which to obtain the data
 * @param model Lux_Sensor_model_t enumerate that identifies the lux model
 * @param addr The array where to store the MAC address
 * @param addr_pin The pin where the sensor has the ADDR port connected
 * @return True if the process execute correcty, otherwise returns false
 **/
bool extract_str_params_Lux_sensor(char *str, Lux_Sensors::Lux_Sensor_model_t &model, uint8_t &addr, uint8_t &addr_pin);

/**
 * Load the Lux sensors initial configuration
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param sensors The Lux_Sensors object where to add the Lux sensors
 **/
void SD_load_Lux_sensors(IniFile *ini, Lux_Sensors *&sensors);

/**
 * Load the ORP sensors initial configuration
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param sensors The ORP_Sensors object where to add the ORP sensors
 **/
void SD_load_ORP_sensors(IniFile *ini, ORP_Sensors *&sensors);

/**
 * Load the WaterProof temperature sensors initial configuration
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param sensors The WP_Temp_Sensors object where to add the WP temperature sensors
 **/
void SD_load_WP_Temp_sensors(IniFile *ini, WP_Temp_Sensors *&sensors);

/**
 * Extract the specific configuration for a Lux sensor from a text string
 * 
 * @param str Initial string from which to obtain the data
 * @param pin The pin where the sensor has connected
 * @param model Current_Model_t enumerate that identifies the current model:
 *      ACS712 -> For invasive sensor
 *      SCT013 -> For non invasive sensor with internal burden resistence
 * @param var Indicates the variation value:
 *       For ACS712 model, indicates the sensitivity (in mV/A)
 *       For SCT013 model, indicates the Ampere value of the clamp
 * @return True if the process execute correcty, otherwise returns false
 **/
bool extract_str_params_Current_sensor(char *str, uint8_t &pin, Current_Sensors::Current_Model_t &model, uint16_t &var);

/**
 * Load the current sensors initial configuration
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param sensors The Current_Sensors object where to add the current sensors
 **/
void SD_load_Current_sensors(IniFile *ini, Current_Sensors *&sensors);

/**
 * Extract the specific configuration for a actuator/device from a text string
 * 
 * @param str Initial string from which to obtain the data
 * @param dev_pin The digital where whe device/actuator is connected
 * @param dev_id The ID to indentify the device/actuator
 * @param init_value The initial value that will be assigned to de device (HIGH/LOW)
 * @return True if the process execute correcty, otherwise returns false
 **/
bool extract_params_Actuator(char *str, uint8_t &dev_pin, char *dev_id, uint8_t &ini_val);

/**
 * Load the WebServer and actuators/devices initial configuration
 * 
 * @param ini The object that contains the IniFile class from where load the data
 * @param eth_server The EthernetServer object where to initialize the WebServer
 * @param actuators The OS_Actuators object where to add the actuators adds to system
 **/
void SD_load_WebServerActuators(IniFile *ini, EthernetServer *&eth_server, OS_Actuators *&actuators);

#endif
