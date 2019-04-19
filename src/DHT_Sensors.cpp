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
    if (n_sensors < DHT_MAX_SENSORS) {
        arr_sensors[n_sensors] = new DHT();
        arr_sensors[n_sensors]->setup(pin);
        n_sensors++;
        return true;
    } else
        return false;
}

/* Capture temperatures & humidities from all DHT sensors */
void DHT_Sensors::capture_all_DHT() {
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

void DHT_Sensors::bulk_Temperatures(String* str, char delim, bool reset) {
    //TODO: Arreglar el reseteo de la informacion previa
    //if (reset) ((String*)str) = "";                        // Indicates whether the string should be deleted before entering the new values
    
    for (uint8_t i=0; i<n_sensors; i++) {
        if (delim != '\0') str->concat(delim);
        str->concat("ta");
        str += i + 1;
        str->concat("=");
        str->concat(arr_Temp[i]);
    }
}

void DHT_Sensors::bulk_Humidities(String* str, char delim, bool reset) {
    //TODO: Arreglar el reseteo de la informacion previa
    //if (reset) str = "";                                   // Indicates whether the string should be deleted before entering the new values
    
    for (uint8_t i=0; i<n_sensors; i++) {
        if (delim != '\0') str->concat(delim);
        str->concat("ha");
        str += i + 1;
        str->concat("=");
        str->concat(arr_Humd[i]);
    }
}

const uint8_t DHT_Sensors::get_num_sensors() {
    return n_sensors;
}

