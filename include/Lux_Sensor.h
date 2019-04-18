/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Lux_Sensor class used to control lux sensors attached to the system
 * 
 */
#ifndef Lux_Sensor_h
#define Lux_Sensor_h

#include <Arduino.h>
#include <BH1750.h>
#include "Configuration.h"

class Lux_Sensor {
private:
    bool initialized;
    uint8_t n_samples;                                          // Number of samples to obtain for each reading process
    BH1750* bh1750_dev;                                         // Pointer to BH1750 instance

    const float filter_result(float* list, uint8_t n_samples);  // Calculate the efective value
    void  sort_result(float* arr, const uint8_t size);          // Sort the list of values obtained from lux sensor

public:
    Lux_Sensor();

    bool begin(uint8_t addr, uint8_t addr_pin = 0);
    const float capture_lux();                                  // Return the readed values from sensor

    void set_n_samples(uint8_t _n_samples);
    uint8_t get_n_samples();
    bool is_init();
};

#endif
