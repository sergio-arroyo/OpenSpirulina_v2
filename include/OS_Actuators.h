/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * OS_Actuators class used to define de external modules (like reles relay, warm focus, etc.) 
 * and be able to act on these (activate or desactivate)
 * 
 */

#ifndef OS_Actuators_h
#define OS_Actuators_h

#include <Arduino.h>
#include "Configuration.h"

#define OS_ACTUATOR_OFF        0x00
#define OS_ACTUATOR_ON         0x01
#define OS_ACTUATOR_STATE_LOW  0x00
#define OS_ACTUATOR_STATE_HIGH 0x01
#define OS_ACTUATOR_NOT_DEF    0xFF

//TODO: documentar clase

class OS_Actuators {
public:
    bool add_device(const char *dev_id, uint8_t dev_pin, uint8_t init_value = LOW);

    bool change_state(const char *dev_id, uint8_t value = 0xFF);

    uint8_t check_device_state(const char *dev_id);

    uint8_t get_n_devices();

private:
    int8_t find_device_by_id(const char *dev_id);
    int8_t find_device_by_pin(uint8_t dev_pin);

    void id_to_lowerCase(char *id);

    struct OS_Actuatorcs_dev {
        char id[ACT_MAX_DEV_ID_LEN+1];
        uint8_t pin;
    } devices[ACT_MAX_NUM_DEVICES];

    uint8_t n_devices = 0;
};

#endif
