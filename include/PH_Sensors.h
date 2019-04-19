/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * PH_Sensor class used to control all PH sensors attached to the system
 * 
 */
#ifndef PH_SENSOR_h
#define PH_SENSOR_h

#include <Arduino.h>
#include "Configuration.h"

class PH_Sensors {
private:
    uint8_t n_sensors;                                     // Number of sensors that are added
    uint8_t n_samples;                                     // Number of samples to obtain for each reading process
    uint8_t pin_sensors[PH_MAX_NUM_SENSORS];               // Array of pH sensors
    float arr_results[PH_MAX_NUM_SENSORS];                 // Array of read values

    void sort_result(int* arr, const uint8_t size);        // Sort the list of values obtained from lux sensor

public:
    PH_Sensors();
    
    bool add_sensor(uint8_t pin);
    void store_all_results();
    const float get_sensor_value(uint8_t n_sensor);
    const uint8_t get_n_sensors();

    void set_n_samples(uint8_t _n_samples);
    const uint8_t get_n_samples();

    void bulk_results(String* str, bool print_tag=true, char delim=',', bool reset=false);
};

#endif
