/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * ORP_Sensor class used to control all ORP (Oxydo Reduction Potential) probe sensors attached to the system
 * 
 */

#include "ORP_Sensors.h"


ORP_Sensors::ORP_Sensors() {
	n_sensors = 0;

    for (uint8_t i=0; i<ORP_MAX_SENSORS; i++)
        val_sensors[i] = 0;
};

bool ORP_Sensors::add_sensor(uint8_t addr) {
    if (n_sensors >= ORP_MAX_SENSORS) return false;
    
    addr_sensors[n_sensors++] = addr;
    
    return true;
}

/* Capture temperatures & humidities from all ORP sensors */
void ORP_Sensors::capture_all_sensors() {
	for (uint8_t i=0; i<n_sensors; i++)
        val_sensors[i] = get_mV(i);                        // Read millivolts signal
}

const int16_t ORP_Sensors::get_mV(uint8_t n_sensor) {
    if (n_sensor >= n_sensors) return -9999;

    Wire.beginTransmission(addr_sensors[n_sensor]);        // call the circuit by its ID number.
    Wire.write('r');                                       // transmit the command that was sent through the serial port.
    Wire.endTransmission();                                // end the I2C data transmission.
    delay(900);                                            // wait the correct amount of time for the circuit to complete its instruction.

    Wire.requestFrom((int)addr_sensors[n_sensor], 20, 1);  // call the circuit and request 20 bytes (this may be more than we need)
    uint8_t result_code = Wire.read();                     // the first byte is the response code, we read this separately.
    Wire.write("sleep");                                   // enter in sleep mode for low energy comsumption
    
    if (result_code != 1)
        return -9999;

    char ORP_data[10];
    uint8_t i = 0;
    while (Wire.available()) {                             // are there bytes to receive.
        ORP_data[i] = Wire.read();                         // load this byte into our array.
        if (ORP_data[i] == 0) {                            // if we see that we have been sent a null command.
            Wire.endTransmission();                        // end the I2C data transmission.
            break;                                         // exit the while loop.
        }
        i++;                                               // incur the counter for the array element.
    }

    return atoi(ORP_data);
}

const uint8_t ORP_Sensors::get_n_sensors() {
    return n_sensors;
}

void ORP_Sensors::bulk_results(String &str, bool reset, bool print_tag, bool print_value, char delim) {
    if (reset) str.remove(0);                              // Delete string before entering the new values
    if (str != "") str.concat(delim);                      // If string is not empty, add delimiter

    for (uint8_t i=0; i<n_sensors; i++) {
        if (i && delim != '\0') str.concat(delim);

        if (print_tag) {                                   // Ambient temperature
            str.concat(F("ORP"));
            str += i+1;

            if (print_value) str.concat(F("="));
        }
        if (print_value) str.concat(val_sensors[i]);
    }
}
