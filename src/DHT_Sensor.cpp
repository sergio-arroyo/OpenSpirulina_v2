/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * DHT_Sensor class used to control all DHT sensors attached to the system
 * 
 */

#include "DHT_Sensor.h"


DHT_Sensor::DHT_Sensor() {
	n_sensors = 0;
};

bool DHT_Sensor::add_sensor(uint8_t pin, DHT_Dev_Model_t model) {
    if (n_sensors < DHT_MAX_SENSORS) {
        arr_sensors[n_sensors] = new DHT();
        arr_sensors[n_sensors]->setup(pin);
        n_sensors++;
        return true;
    } else
        return false;
}

/* Capture temperatures & humidities from all DHT sensors */
void DHT_Sensor::capture_all_DHT() {
	for (uint8_t i=0; i<n_sensors; i++) {
        arr_Temp[i] = arr_sensors[i]->getTemperature();    // Read Temperature
		arr_Humd[i] = arr_sensors[i]->getHumidity();       // Read Humidity
	}
}

const float DHT_Sensor::get_Temperature(uint8_t n_sensor) {
    if (n_sensor < n_sensors)
        return arr_sensors[n_sensor]->getTemperature();
    
    return 0;
}

const float DHT_Sensor::get_Humidity(uint8_t n_sensor) {
    if (n_sensor < n_sensors)
        return arr_sensors[n_sensor]->getHumidity();
    
    return 0;
}

void DHT_Sensor::bulk_Temperatures(String* str, char delim, bool reset) {
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

void DHT_Sensor::bulk_Humidities(String* str, char delim, bool reset) {
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

const uint8_t DHT_Sensor::get_num_sensors() {
    return n_sensors;
}

