/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Lux_Sensor class used to control lux sensors attached to the system
 * 
 * The class supports sensors models BH1750 and MAX44009
 * 
 */
#ifndef Lux_Sensors_h
#define Lux_Sensors_h

#include <Arduino.h>
#include <BH1750.h>
#include <MAX44009.h>
#include "Configuration.h"

class Lux_Sensors {
public:
    enum Lux_Sensor_model_t : uint8_t {
        mod_UNDEFINED = 0,
        mod_BH1750,
        mod_MAX44009
    };
    
    /**
     * Constructor
     **/
    Lux_Sensors();

    /**
     * Initialize the lux module
     * 
     * @param addr  Indicates the address for BH1750 lux sensor 
     * @param addr_pin The pin that change the BH1750 address from 0x23 to 0x5C
     * @return returns true if module is initialize correctly, otherwise returns false 
     **/
    bool add_sensor(Lux_Sensor_model_t model, uint8_t addr, uint8_t addr_pin = 0);
    
    /**
     * Return the readed values from sensor
     * 
     * @return The calculated instantaneous value of the read and filtered values
     **/
    const uint16_t get_instant_lux(uint8_t n_sensor);

    /** 
     * Read all lux sensors and store the values to internal array
     * 
     **/
    void capture_all_sensors();

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
    const uint8_t get_n_samples();
    
    /**
     * Get the number of sensors added to the system
     * 
     * @return The number of sensors added to the system
     **/
    const uint8_t get_n_sensors();

    /**
     * Get the model of the sensor
     * 
     * @return The model of the sensor
     **/
    Lux_Sensors::Lux_Sensor_model_t get_model_sensors(uint8_t n_sensor);

    /**
     * Performs dump of all result values stored in the data array
     * 
     * @param str Pointer to String where the results are stored
     * @param reset Indicates whether the text string will be cleaned before entering data
     * @param print_tag Indicates whether the label of each sensor should be displayed
     * @param print_value Indicates whether the value of each sensor should be displayed
     * @param delim Character that indicates the separator of the fields shown
     **/
    void bulk_results(String &str, bool reset = true, bool print_tag = true,
                        bool print_value = true, char delim = ',');

private:
    uint8_t n_samples;                                     // Number of samples to obtain for each reading process

    struct Lux_data_st {
        Lux_Sensor_model_t model;
        void *sensor;
        uint16_t read_val = 0;
    } lux_sensors[4];                                      // structure to store the different sensors instances (BH1750 and MAX44009)

    uint8_t n_sensors_BH;
    uint8_t n_sensors_MAX;

};

#endif
