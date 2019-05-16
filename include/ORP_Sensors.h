/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * ORP_Sensor class used to control all ORP (Oxydo Reduction Potential) probe sensors attached to the system
 * 
 * Based on: https://www.atlas-scientific.com/_files/code/ORP-i2c.pdf
 * 
 */
#ifndef ORP_Sensors_h
#define ORP_Sensors_h

#include <Arduino.h>
#include "Wire.h"
#include "Configuration.h"


class ORP_Sensors {
public:
    /**
     * Constructor
     **/
    ORP_Sensors();
    
    /**
     * Add new ORP sensors to the system
     * 
     * @param addr The I2C address where the sensor is connected
     * @return Return true if sensor added correctly, otherwise retunrs false
     **/
    bool add_sensor(uint8_t addr);

    /** 
     * Read all current sensors and store the values to internal array
     * 
     **/
    void capture_all_sensors();

    /**
     * Get the instant value of the probe sensor
     * 
     * @param n_sensor Number of sensor added to the system (from 0 to N-1)
     * @return The instant value in millivolts (mV)
     **/
    const int16_t get_mV(uint8_t n_sensor);

    /**
     * Get the number of sensors added to the system
     * 
     * @return The number of sensors added to the system
     **/
    const uint8_t get_n_sensors();

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
    uint8_t n_sensors;
    
    uint8_t addr_sensors[ORP_MAX_SENSORS];                  // Array of ORP sensors
    int16_t val_sensors[ORP_MAX_SENSORS];                   // Array of read values (Range +/-2000mV)
};

#endif
