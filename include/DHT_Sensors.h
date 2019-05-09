/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * DHT_Sensor class used to control all DHT sensors attached to the system
 * 
 */
#ifndef DHT_Sensors_h
#define DHT_Sensors_h

#include <Arduino.h>
#include <DHT.h>
#include "Configuration.h"


class DHT_Sensors {
public:
    typedef enum {
        AUTO_DETECT,
        DHT11,
        DHT22,
        AM2302,  // Packaged DHT22
        RHT03    // Equivalent to DHT22
    } DHT_Dev_Model_t;
    
    /**
     * Constructor
     **/
    DHT_Sensors();
    
    /**
     * Add new DHT sensors to the system
     * 
     * @param pin Where the sensor is connected
     * @param model Indicates the type of sensor. Types options
     *          AUTO_DETECT, DHT11, DHT22, AM2302, RHT03
     * @return Return true if sensor added correctly, otherwise retunrs false
     **/
    bool add_sensor(uint8_t pin, DHT_Dev_Model_t model = AUTO_DETECT);

    /** 
     * Read all current sensors and store the values to internal array
     * 
     **/
    void capture_all_sensors();

    /**
     * Get the value of the temperature sensor stored in the array
     * 
     * @param n_sensor Number of sensor added to the system (from 0 to N-1)
     * @return The stored value in the array for specific temperature sensor
     **/
    const float get_Temperature(uint8_t n_sensor);

    /**
     * Get the value of the humidity sensor stored in the array
     * 
     * @param n_sensor Number of sensor added to the system (from 0 to N-1)
     * @return The stored value in the array for specific temperature sensor
     **/
    const float get_Humidity(uint8_t n_sensor);

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
    
    DHT* arr_sensors[DHT_MAX_SENSORS];                     // Array of DHT sensors
    float arr_Temp[DHT_MAX_SENSORS];                       // Array of read temperatures
    float arr_Humd[DHT_MAX_SENSORS];                       // Array of read humidities
};

#endif
