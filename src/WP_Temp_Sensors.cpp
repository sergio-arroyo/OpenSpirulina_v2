/**
 * OpenSpirulina http://www.openspirulina.com
 *
     * Autors: Sergio Arroyo (UOC)
 * 
 * Waterproof Temperature Sensors class used to control all DS18B20 sensors attached to the system
 * 
 */

#include "WP_Temp_Sensors.h"

WP_Temp_Sensors::WP_Temp_Sensors(uint8_t oneWire_pin) {
    oneWireObj = new OneWire(oneWire_pin);
    sensors_ds18 = new DallasTemperature(oneWireObj);
	n_pairs = 0;
    initialized = false;
    
    for (uint8_t i=0; i<WP_T_MAX_PAIRS_SENS; i++) {
        arr_s_results[i] = 0;
        arr_b_results[i] = 0;
    }
};

void WP_Temp_Sensors::begin() {
    sensors_ds18->begin();
    initialized = true;
}

WP_Temp_Sensors::WP_Sensor_error_t WP_Temp_Sensors::add_sensors_pair(const uint8_t* s_sensor, const uint8_t* b_sensor) {
    if (!sensors_ds18->isConnected(s_sensor)) return Surface_NotConn;     // If surface sensor not connected, return error
    if (!sensors_ds18->isConnected(b_sensor)) return Background_NotConn;  // If background sensor not connected, return error
    if (n_pairs >= WP_T_MAX_PAIRS_SENS) return Max_Reached;               // Controls that no more pairs of the allowed ones are inserted

    for (uint8_t i=0; i<8; i++) {                                         // Copy incomming data
        sensors_pairs[n_pairs].s_sensor[i] = s_sensor[i];
        sensors_pairs[n_pairs].b_sensor[i] = b_sensor[i];
    }
    n_pairs++;
    
    return No_Error;
}

void WP_Temp_Sensors::store_all_results() {
    sensors_ds18->requestTemperatures();                   //Sends command for all devices on the bus to perform a temperature conversion

    for (uint8_t i=0; i<n_pairs; i++) {
        arr_s_results[i] = sensors_ds18->getTempC(sensors_pairs[i].s_sensor);
        arr_b_results[i] = sensors_ds18->getTempC(sensors_pairs[i].b_sensor);
    }
}

const float WP_Temp_Sensors::get_result_pair(uint8_t n_pair, WP_Temp_sensor_t sensor) {
    if (n_pair >= n_pairs) return -127;                    // If pair is outbound, return a error value
    
    if (sensor == S_Surface)
        return arr_s_results[n_pair];
    
    if (sensor == S_Background)
        return arr_b_results[n_pair];
    
    if (sensor == S_Both)
        return (arr_s_results[n_pair] + arr_b_results[n_pair]) / 2;
    
    return -127;                                           // Otherwise return a error value
}

const float WP_Temp_Sensors::get_instant_pair(uint8_t n_pair, WP_Temp_sensor_t sensor, bool send_req) {
    if (n_pair >= n_pairs) return -127;                    //If the sensor pair ID is greater than the available one, 0 is returned

    if (send_req) sensors_ds18->requestTemperatures();     //Sends command for all devices on the bus to perform a temperature conversion
    
    if (sensor == S_Surface)
        return sensors_ds18->getTempC(sensors_pairs[n_pair].s_sensor);
    
    if (sensor == S_Background)
        return sensors_ds18->getTempC(sensors_pairs[n_pair].b_sensor);
    
    return (sensors_ds18->getTempC(sensors_pairs[n_pair].s_sensor) +
            sensors_ds18->getTempC(sensors_pairs[n_pair].b_sensor)) / 2;
}

const uint8_t WP_Temp_Sensors::get_n_pairs() {
    return n_pairs;
}

bool WP_Temp_Sensors::is_init() {
    return initialized;
}

void WP_Temp_Sensors::bulk_results(String* str, bool print_tag, char delim, bool reset) {
    if (reset) str->remove(0);                             // Indicates whether the string should be deleted before entering the new values
    
    for (uint8_t i=0; i<n_pairs; i++) {
        if (delim != '\0') str->concat(delim);
        if (print_tag) {
            str->concat("T");
            str += i + 1;
            str->concat("_s=");
        }
        str->concat(arr_s_results[i]);
        if (print_tag) {
            str->concat("T");
            str += i + 1;
            str->concat("_b=");
        }
        str->concat(arr_b_results[i]);
    }
}