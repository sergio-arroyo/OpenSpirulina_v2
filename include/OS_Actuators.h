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


class OS_Actuators {
public:
    /**
     * Initialize the lux module
     * 
     * @param dev_id The ID to indentify the device/actuator
     * @param dev_pin The digital where whe device/actuator is connected
     * @param init_value The initial value that will be assigned to de device (HIGH/LOW)
     * @return true if device is added correcty to the system, otherwise returns false 
     **/
    bool add_device(const char *dev_id, uint8_t dev_pin, uint8_t init_value = LOW);

    /**
     * Change the state to a specific device/actuator
     * 
     * @param dev_id The ID that indentify the device/actuator
     * @param value The new value that will be assigned to de device
     *    LOW  - Assigns LOW value  (0x00)
     *    HIGH - Assigns HIGH value (0x01)
     *    0xFF - Switch de actual value
     * @return true if value has change correctly, otherwise returns false 
     **/
    bool change_state(const char *dev_id, uint8_t value = 0xFF);

    /**
     * Get the actual state from a specific device/actuator
     * 
     * @param dev_id The ID that indentify the device/actuator
     * @return The current state of the device, if it exist
     *    LOW  - The current value is LOW  (0x00)
     *    HIGH - The current value is HIGH (0x01)
     *    0xFF - The requested device not exist on the system
     **/
    uint8_t get_device_state_by_id(const char *dev_id);

    /**
     * Get the actual ID from a specific device/actuator
     * 
     * @param pos The specific position where stored the device/actuator
     * @return The ID that indentify the device/actuator
     **/
    const char *get_device_id(uint8_t pos) const;

    /**
     * Get the actual state from a specific device/actuator
     * 
     * @param pos The specific position where stored the device/actuator
     * @return The current state of the device, if it exist
     *    LOW  - The current value is LOW  (0x00)
     *    HIGH - The current value is HIGH (0x01)
     *    0xFF - The requested device not exist on the system
     **/
    const uint8_t get_device_state(uint8_t pos) const;

    /**
     * Get the number of devices/actuators added to the system
     * 
     * @return The number of devices/actuators added to the system
     **/
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
