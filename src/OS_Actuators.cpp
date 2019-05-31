/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * OS_Actuators class used to define de external modules (like reles relay, warm focus, etc.) 
 * and be able to act on these (activate or desactivate)
 * 
 */

#include "OS_Actuators.h"

bool OS_Actuators::add_device(const char *dev_id, uint8_t dev_pin, uint8_t init_value) {
    // Check is possible to add more devices
    if (n_devices >= ACT_MAX_NUM_DEVICES)
        return false;

    // Check if device is already added to the system
    if (find_device_by_id(dev_id) > -1 || find_device_by_pin(dev_pin) > -1)
        return false;
    
    devices[n_devices].pin = dev_pin;
    strncpy(devices[n_devices].id, dev_id, ACT_MAX_DEV_ID_LEN);
    id_to_lowerCase(devices[n_devices].id);                // Change to lower case ID
    
    // Initialize pin as PUTPUT
    pinMode(devices[n_devices].pin, OUTPUT);
    digitalWrite(devices[n_devices].pin, init_value);

    n_devices++;
    
    return true;
}

bool OS_Actuators::change_state(const char *dev_id, uint8_t value) {
    // find the position of the device
    int8_t pos = find_device_by_id(dev_id);

    if (pos == -1)
        return false;                                      // Return false if device not found
    
    if (value != 0xFF) {
        digitalWrite(devices[pos].pin, value);             // Apply the new value
    } else {
        // Switch actual value:
        //    true  (xor) true -> false
        //    false (xor) true -> true
        digitalWrite(devices[pos].pin,
            digitalRead(devices[pos].pin) ^ true);
    }

    return true;
}

uint8_t OS_Actuators::check_device_state(const char *dev_id) {
    // find the position of the device
    int8_t pos = find_device_by_id(dev_id);

    if (pos == -1)
        return OS_ACTUATOR_NOT_DEF;                        // return device not found

    return digitalRead(devices[pos].pin);                  // return the read value (HIGH=0x1 / LOW=0x0)
}

uint8_t OS_Actuators::get_n_devices() {
    return n_devices;
}

int8_t OS_Actuators::find_device_by_id(const char *dev_id) {
    int8_t pos = -1;
    char id_s[ACT_MAX_DEV_ID_LEN+1] = "";

    strncpy(id_s, dev_id, ACT_MAX_DEV_ID_LEN);
    id_to_lowerCase(id_s);                                 // change to lower case ID

    for (uint8_t i=0; i<n_devices; i++) {
        if (strcmp(devices[i].id, id_s) == 0) {
            pos = i;
            break;         // exit for
        }
    }

    return pos;
}

int8_t OS_Actuators::find_device_by_pin(uint8_t dev_pin) {
    int8_t pos = -1;

    for (uint8_t i=0; i<n_devices; i++) {
        if (devices[i].pin == dev_pin) {
            pos = i;
            break;         // exit for
        }
    }

    return pos;
}

void OS_Actuators::id_to_lowerCase(char *id) {
    uint8_t i = 0;

    while (id[i]) {
        id[i] = (char) tolower(id[i]);
        i++;
    }
}
