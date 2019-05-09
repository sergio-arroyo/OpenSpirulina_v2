/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * PH_Sensor class used to control all PH sensors attached to the system
 * 
 */
#ifndef PH_Sensors_h
#define PH_Sensors_h

#include <Arduino.h>
#include "Configuration.h"

class PH_Sensors {
public:
    /**
     * Constructor
     **/
    PH_Sensors();

    /**
     * Add new PH sensors to the system
     * 
     * @param pin Where the sensor is connected
     * @return Return true if sensor added correctly, otherwise retunrs false
     **/
    bool add_sensor(uint8_t pin);

    /** 
     * Read all PH sensors and store the values to internal array
     **/
    void capture_all_sensors();

    /**
     * Capture the instant PH value from specific sensor
     * 
     * @return The calculated instantaneous value of the read and filtered values
     **/
    const float get_sensor_value(uint8_t n_sensor);

    /**
     * Get the number of sensors added to the system
     * 
     * @return The number of sensors added to the system
     **/
    const uint8_t get_n_sensors();

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
    uint8_t n_sensors;                                     // Number of sensors that are added
    uint8_t n_samples;                                     // Number of samples to obtain for each reading process
    uint8_t pin_sensors[PH_MAX_NUM_SENSORS];               // Array of pH sensors
    float arr_results[PH_MAX_NUM_SENSORS];                 // Array of read values

};

#endif
