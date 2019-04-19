/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * PH_Sensor class used to control all PH sensors attached to the system
 * 
 */

#include "PH_Sensors.h"


PH_Sensors::PH_Sensors() {
	n_sensors = 0;
    n_samples = PH_SENS_N_SAMP_READ;
    
    for (uint8_t i=0; i<PH_MAX_NUM_SENSORS; i++)
        arr_results[i] = 0;
};

bool PH_Sensors::add_sensor(uint8_t pin) {
    if (n_sensors < PH_MAX_NUM_SENSORS) {
        pin_sensors[n_sensors++] = pin;
        return true;
    } else
        return false;
}

/* Capture temperatures & humidities from all DHT sensors */
void PH_Sensors::store_all_results() {
	for (uint8_t i=0; i<n_sensors; i++)
        arr_results[i] = get_sensor_value(i);                  //Read Temperature
}

/* Return pH value from a specific sensor */
const float PH_Sensors::get_sensor_value(uint8_t n_sensor) {
    if (n_sensor >= n_sensors) return 0;                       //If n_sensor is out of bounds for number of sensors attached, return 0

    int buffer[10];
    for (uint8_t i=0; i<n_samples; i++) {                      //Get 10 sample value from the sensor for smooth the value
		buffer[i] = analogRead(pin_sensors[n_sensor]);
		delay(10);
    }
	
    sort_result(buffer, n_samples);

    buffer[0] = 0;                                             //Reuse buffer[0] possition to calculate de final value
    for (uint8_t i=2; i<n_samples-2; i++)                      //Take the average value of 6 center sample
        buffer[0] += buffer[i];
	
    float pH_value = (float)buffer[0]*5.0/1024/(n_samples-1);  //Convert the analog into millivolt
    pH_value = 3.5 * pH_value;                                 //Convert the millivolt into pH value

    return pH_value;                                           //Return pH_value for that sensorPin
}

const uint8_t PH_Sensors::get_n_sensors() {
    return n_sensors;
}

void PH_Sensors::set_n_samples(const uint8_t _n_samples) {
    n_samples = _n_samples;
}

const uint8_t PH_Sensors::get_n_samples() {
    return n_samples;
}

void PH_Sensors::bulk_results(String* str, bool print_tag, char delim, bool reset) {
    //TODO: Arreglar el reseteo de la informacion previa
    //if (reset) ((String*)str) = "";                        // Indicates whether the string should be deleted before entering the new values
    
    for (uint8_t i=0; i<n_sensors; i++) {
        if (delim != '\0') str->concat(delim);
        if (print_tag) {
            str->concat("pH");
            str += i + 1;
            str->concat("=");
        }
        str->concat(arr_results[i]);
    }
}

void PH_Sensors::sort_result(int* arr, const uint8_t n_samples) {
    int tmp_val;

    for (uint8_t i=0; i < (n_samples-1); i++) {
        for (uint8_t j=0; j < (n_samples-(i+1)); j++) {
            if (arr[j] > arr[j+1]) {
                tmp_val = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = tmp_val;
            }
        }
    }
}
