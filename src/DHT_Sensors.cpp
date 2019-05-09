/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * DHT_Sensor class used to control all DHT sensors attached to the system
 * 
 */

#include "DHT_Sensors.h"


DHT_Sensors::DHT_Sensors() {
	n_sensors = 0;

    for (uint8_t i=0; i<DHT_MAX_SENSORS; i++) {
        arr_Temp[i] = 0;
        arr_Humd[i] = 0;
    }
};

bool DHT_Sensors::add_sensor(uint8_t pin, DHT_Dev_Model_t model) {
    if (n_sensors >= DHT_MAX_SENSORS) return false;
    
    arr_sensors[n_sensors] = new DHT();
    arr_sensors[n_sensors]->setup(pin);
    n_sensors++;
    
    return true;
}

/* Capture temperatures & humidities from all DHT sensors */
void DHT_Sensors::capture_all_sensors() {
	for (uint8_t i=0; i<n_sensors; i++) {
        arr_Temp[i] = arr_sensors[i]->getTemperature();    // Read Temperature
		arr_Humd[i] = arr_sensors[i]->getHumidity();       // Read Humidity
	}
}

const float DHT_Sensors::get_Temperature(uint8_t n_sensor) {
    if (n_sensor < n_sensors)
        return arr_sensors[n_sensor]->getTemperature();
    
    return 0;
}

const float DHT_Sensors::get_Humidity(uint8_t n_sensor) {
    if (n_sensor < n_sensors)
        return arr_sensors[n_sensor]->getHumidity();
    
    return 0;
}

const uint8_t DHT_Sensors::get_n_sensors() {
    return n_sensors;
}

void DHT_Sensors::bulk_results(String &str, bool reset, bool print_tag, bool print_value, char delim) {
    if (reset) str.remove(0);                              // Delete string before entering the new values
    if (str != "") str.concat(delim);                      // If string is not empty, add delimiter

    for (uint8_t i=0; i<n_sensors; i++) {
        if (i && delim != '\0') str.concat(delim);

        if (print_tag) {                                   // Ambient temperature
            str.concat(F("Amb"));
            str += i+1;
            str.concat(F("_t"));

            if (print_value) str.concat(F("="));
        }
        if (print_value) str.concat(arr_Temp[i]);

        str.concat(delim);
        if (print_tag) {                                   // Ambient humidity
            str.concat(F("Amb"));
            str += i+1;
            str.concat(F("_h"));

            if (print_value) str.concat(F("="));
        }
        if (print_value) str.concat(arr_Humd[i]);
    }
}
