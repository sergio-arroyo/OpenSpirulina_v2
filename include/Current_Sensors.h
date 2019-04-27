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
    
    /* Constructor */
    Current_Sensors();

    /* Add new current sensors to the system */
    bool add_sensor(uint8_t pin, Current_Model_t model, uint16_t var);

    /* Obtain current value on specific invasive sensor ACS712 */
    const float get_current_value(uint8_t n_sensor);

    /* Read all current sensors and store the values */
    void capture_all_currents();

    const uint8_t get_n_sensors();

    const uint16_t get_volt_ref();
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

    const float get_current_ACS712(uint8_t n_sensor);
    const float get_current_SCT013(uint8_t n_sensor);
    
    uint16_t read_V_ref();
};

#endif