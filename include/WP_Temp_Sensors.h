/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Waterproof Temperature Sensors class used to control all DS18B20 sensors attached to the system
 * 
 */
#ifndef WP_TEMP_SENSORS_h
#define WP_TEMP_SENSORS_h

#include <Arduino.h>
#include <DallasTemperature.h>
#include "Configuration.h"


class WP_Temp_Sensors {
public:
    struct Sensor_pairs_t {
        uint8_t s_sensor[8];                                      // Defines the surface sensor address
        uint8_t b_sensor[8];                                      // Defines the background sensor address
    };

    WP_Temp_Sensors(uint8_t oneWire_pin);
    
    void begin();                                                 // Initialize bus
    uint8_t add_sensors_pair(const uint8_t* s_sensor, 
                             const uint8_t* b_sensor);            // Add pair sensor of sensors
    void store_all_results();                                     // Get values from all sensors and calculate the respective means
    
    const float get_result_pair(uint8_t n_pair, uint8_t n_sensor);
    const float get_result_pair_mean(uint8_t n_pair);
    const float get_instant_pair_mean(uint8_t n_pair,
                                      bool send_req = true);      // Get instant mean from specific sensor pair
    const uint8_t get_n_pairs();                                  // Get the number of pair sensors added on system
    bool is_init();

    void bulk_results(String* str, bool print_tag = true,
                      char delim = ',', bool reset = false);

private:
    OneWire* oneWireObj;                                          // One Wire control protocol
    DallasTemperature* sensors_ds18;                              // Control DS18 sensors
    uint8_t n_pairs;                                              // Number of pair sensors that are added
    bool initialized;                                             // Indicates whether the object has been initialized or not 

    Sensor_pairs_t sensors_pairs[WP_T_MAX_PAIRS_SENS];
    float arr_s_results[WP_T_MAX_PAIRS_SENS];                     // Array of read values from surface
    float arr_b_results[WP_T_MAX_PAIRS_SENS];                     // Array of read values from background
};

#endif
