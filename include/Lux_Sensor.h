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
public:
    /**
     * Constructor
     **/
    Lux_Sensor();

    /**
     * Initialize the lux module
     * 
     * @param addr  Indicates the address for BH1750 lux sensor 
     * @param addr_pin The pin that change the BH1750 address from 0x23 to 0x5C
     * @return returns true if module is initialize correctly, otherwise returns false 
     **/
    bool begin(uint8_t addr, uint8_t addr_pin = 0);

    /**
     * Return the readed values from sensor
     * 
     * @return The calculated instantaneous value of the read and filtered values
     **/
    const float capture_lux();

    /**
     * Update the number of samples to obtain in each reading
     * 
     * @param _n_samples Number of samples to obtain
     **/
    void set_n_samples(uint8_t _n_samples);

    /**
     * Get the number of samples to obtain in each reading
     * 
     * @return Number of samples to obtain
     **/
    uint8_t get_n_samples();

    /**
     * Indicates whether the module is started or not
     * 
     * @return Return true if the module is initialized, otherwise false
     **/
    bool is_init();

private:
    bool initialized;
    uint8_t n_samples;                                     // Number of samples to obtain for each reading process
    BH1750* bh1750_dev;                                    // Pointer to BH1750 instance

    const float capture_and_filter();                      // Calculate the efective value
};

#endif
