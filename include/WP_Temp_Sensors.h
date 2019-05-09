/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Waterproof Temperature Sensors class used to control all DS18B20 sensors attached to the system
 * 
 */
#ifndef WP_Temp_Sensors_h
#define WP_Temp_Sensors_h

#include <Arduino.h>
#include <DallasTemperature.h>
#include "Configuration.h"


class WP_Temp_Sensors {
public:
    struct Sensor_pairs_t {
        uint8_t s_sensor[8];                                      // Defines the surface sensor address
        uint8_t b_sensor[8];                                      // Defines the background sensor address
    };

    enum WP_Temp_sensor_t : uint8_t {
        S_Surface = 0,
        S_Background,
        S_Both
    };

    enum WP_Sensor_error_t : uint8_t {
        No_Error = 0,
        Surface_NotConn,
        Background_NotConn,
        Max_Reached
    };

    /**
     * Constructor
     **/
    WP_Temp_Sensors(uint8_t oneWire_pin);

    /**
     * Initialize the bus to control DS18 sensors
     **/
    void begin();

    /**
     * Add new WP temp. sensor to the system
     * 
     * @param s_sensor Address of the surface sensor to be inserted
     * @param b_sensor Address of the background sensor to be inserted
     * @return Returns:
     *             0 if sensor added correctly
     *             1 if the surface sensor not connected
     *             2 if the background sensor not connected
     *             3 no more pairs of the allowed ones can be inserted 
     **/
    WP_Sensor_error_t add_sensors_pair(const uint8_t* s_sensor, 
                                        const uint8_t* b_sensor);

    /** 
     * Read all temperature sensors and store the values to internal array
     **/
    void store_all_results();
    

    /* Return result from sensor pair. n_sensor=1: return surface value, n_sensor!=2: return background value */

    /**
     * Return result from sensor pair
     * 
     * @param n_pair Number of pair sensor to be consulted
     * @param sensor The sensor from which you want to obtain the read value:
     *             S_Surface: returns the surface sensor value
     *             S_Background: returns the background sensor value
     *             S_Both: returns de mean of both values
     * @return Get de value stored for the especific sensor
     **/
    const float get_result_pair(uint8_t n_pair, WP_Temp_sensor_t sensor);

    /**
     * Return the instant result read from sensor pair
     * 
     * @param n_pair Number of pair sensor to be consulted
     * @param sensor The sensor from which you want to obtain the read value:
     *             S_Surface: returns the surface sensor value
     *             S_Background: returns the background sensor value
     *             S_Both: returns de mean of both values
     * @return Get de value stored for the especific sensor
     **/
    const float get_instant_pair(uint8_t n_pair,
                                 WP_Temp_sensor_t sensor,
                                 bool send_req = true);

    /**
     * Get the number of sensors added to the system
     * 
     * @return The number of sensors added to the system
     **/
    const uint8_t get_n_pairs();

    /**
     * Indicates whether the module is started or not
     * 
     * @return Return true if the module is initialized, otherwise false
     **/
    bool is_init();

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
    OneWire* oneWireObj;                                          // One Wire control protocol
    DallasTemperature* sensors_ds18;                              // Control DS18 sensors
    uint8_t n_pairs;                                              // Number of pair sensors that are added
    bool initialized;                                             // Indicates whether the object has been initialized or not 

    Sensor_pairs_t sensors_pairs[WP_T_MAX_PAIRS_SENS];
    float arr_s_results[WP_T_MAX_PAIRS_SENS];                     // Array of read values from surface
    float arr_b_results[WP_T_MAX_PAIRS_SENS];                     // Array of read values from background
};

#endif
