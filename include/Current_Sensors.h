/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Current_Sensors class used to control invasive & non-invasive current sensors attached to the system
 * 
 * Based on:
 *    DFRobot code: https://wiki.dfrobot.com/Gravity_Analog_AC_Current_Sensor__SKU_SEN0211_
 * 
 *    Naylamp Mechatronics code: https://naylampmechatronics.com/blog/48_tutorial-sensor-de-corriente-acs712.html
 * 
 */

#ifndef Current_Sensors_h
#define Current_Sensors_h

#include <Arduino.h>
#include "Configuration.h"


class Current_Sensors {
public:
    typedef enum {                                             // Defines the type of sensor
        ACS712 = 0,                                            // Invasive sensor
        SCT013                                                 // Non-invasive sensor
    } Current_Model_t;
    
    /**
     * Constructor
     **/
    Current_Sensors();

    /**
     * Add new current sensors to the system
     * 
     * @param pin Where the sensor is connected
     * @param model Indicates the type of sensor. ACS712 = invasive sensor; SCT013 = Non invasive sensor
     * @param var Indicates the variation value for the sensor 
     * @return Return true if sensor added correctly, otherwise retunrs false
     **/
    bool add_sensor(uint8_t pin, Current_Model_t model, uint16_t var);

    /**
     * Get the value of the sensor captured in the instant
     * 
     * @param n_sensor Number of sensor added to the system (from 0 to N-1)
     * @return The instant value readed 
     **/
    const float get_current_value(uint8_t n_sensor);

    /** 
     * Read all current sensors and store the values to internal array
     **/
    void capture_all_sensors();

    /**
     * Get the number of sensors added to the system
     * 
     * @return The number of sensors added to the system
     **/
    const uint8_t get_n_sensors();

    /**
     * Gets the reference voltage stored
     * 
     * @return Calculated reference voltage (in mV)
     **/
    const uint16_t get_volt_ref();

    /**
     * Sets a specific value for reference voltage
     * 
     * @param _v_ref The voltage reference (in mV)
     **/
    void set_volt_ref(uint16_t _v_ref);

private:
    struct Curr_sens_t {
        Current_Model_t model;
        uint8_t  pin;
        uint16_t var;
    } sensors[CURR_MAX_NUM_SENSORS];

    uint16_t v_ref;
    uint8_t n_sensors;
    float arr_current[CURR_SENS_DEF_NUM];                  // Array of read currents

    /** 
     * Obtain current value on specific invasive sensor ACS712
     * 
     * @param n_sensor Number of sensor added to the system (from 0 to N-1)
     * @return The instant value readed 
     **/
    const float get_current_ACS712(uint8_t n_sensor);

    /** 
     * Obtain current value on specific non-invasive sensor SCT013
     * 
     * @param n_sensor Number of sensor added to the system (from 0 to N-1)
     * @return The instant value readed 
     **/
    const float get_current_SCT013(uint8_t n_sensor);
    
    /**
     * Read the reference voltage calculated on the MCU where the system runs (in mV)
     * and saves it for use in internal calculations
     * 
     * @return Calculated reference voltage (in mV)
     **/
    uint16_t read_V_ref();
};

#endif
