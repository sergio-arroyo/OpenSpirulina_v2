/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * DHT_Sensor class used to control all DHT sensors attached to the system
 * 
 */
#ifndef DHT_SENSOR_h
#define DHT_SENSOR_h

#include <Arduino.h>
#include <DHT.h>
#include "Configuration.h"

typedef enum {
    AUTO_DETECT,
    DHT11,
    DHT22,
    AM2302,  // Packaged DHT22
    RHT03    // Equivalent to DHT22
  } DHT_Dev_Model_t;

class DHT_Sensors {
public:
    DHT_Sensors();
    
    bool add_sensor(uint8_t pin, DHT_Dev_Model_t model=AUTO_DETECT);
    void capture_all_DHT();
    const float get_Temperature(uint8_t n_sensor);
    const float get_Humidity(uint8_t n_sensor);
    void bulk_Temperatures(String *str, char delim=',', bool reset=false);
    void bulk_Humidities(String *str, char delim=',', bool reset=false);
    const uint8_t get_num_sensors();
    
private:
    uint8_t n_sensors;
    
    DHT* arr_sensors[DHT_MAX_SENSORS];                     // Array of DHT sensors
    float arr_Temp[DHT_MAX_SENSORS];                       // Array of read temperatures
    float arr_Humd[DHT_MAX_SENSORS];                       // Array of read humidities
};

#endif
