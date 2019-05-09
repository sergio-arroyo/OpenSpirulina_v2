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

void PH_Sensors::capture_all_sensors() {
	for (uint8_t i=0; i<n_sensors; i++)
        arr_results[i] = get_sensor_value(i);              //Read Temperature
}

const float PH_Sensors::get_sensor_value(uint8_t n_sensor) {
    if (n_sensor >= n_sensors) return 0;                   // If n_sensor is out of bounds for number of sensors attached, return 0

    float read_v, min_v=0, max_v=0, total_v=0;
    for (uint8_t i=n_samples; i>0; i--) {
        read_v = analogRead(pin_sensors[n_sensor]);
        total_v += read_v;

        if (read_v < min_v) min_v = read_v;                // Update de min value
        if (read_v > max_v) max_v = read_v;                // Update de max value
        delay(10);
    }

    total_v = total_v - min_v - max_v;                     // Discards lower and higher value for the average
    total_v /= (n_samples-2);
    
    float pH_value = (float)total_v * 5.0 / 1024;          // Convert the analog into millivolt
    pH_value *= 3.5;                                       // Convert the millivolt into pH value

    return pH_value;                                       // Return pH_value for that sensorPin
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

void PH_Sensors::bulk_results(String &str, bool reset, bool print_tag, bool print_value, char delim) {
    if (reset) str.remove(0);                              // Delete string before entering the new values
    if (str != "") str.concat(delim);                      // If string is not empty, add delimiter
    
    for (uint8_t i=0; i<n_sensors; i++) {
        if (i && delim != '\0') str.concat(delim);
        if (print_tag) {
            str.concat("pH");
            str += i+1;

            if (print_value) str.concat("=");
        }
        if (print_value) str.concat(arr_results[i]);
    }
}
