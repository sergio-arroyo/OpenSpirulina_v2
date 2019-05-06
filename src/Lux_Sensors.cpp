/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Lux_Sensor class used to control lux sensors attached to the system
 * 
 * The class supports sensors models BH1750 and MAX44009
 * 
 */

#include "Lux_Sensors.h"


Lux_Sensors::Lux_Sensors() {
    n_sensors_BH  = 0;
    n_sensors_MAX = 0;
    n_samples     = LUX_SENS_N_SAMP_READ;
}

bool Lux_Sensors::add_sensor(Lux_Sensors::Lux_Sensor_model_t model, uint8_t addr, uint8_t addr_pin) {
    if (model == mod_BH1750 && n_sensors_BH >= 2) return false;
    if (model == mod_MAX44009 && n_sensors_MAX >= 2) return false;

    if (addr_pin != 0) {
        pinMode(addr_pin, OUTPUT);                              // Sets the digital pin as output
        digitalWrite(addr_pin, HIGH);                           // Sets the digital pin ON
        delay(200);
    }

    uint8_t act_sens = get_n_sensors();
    switch (model) {
        case mod_BH1750:
            lux_sensors[act_sens].sensor = new BH1750(addr);    // Instanciate new BH1750 object

            // If not initialized exit and return false
            if ( !((BH1750*) lux_sensors[act_sens].sensor)->begin(BH1750::Mode::CONTINUOUS_HIGH_RES_MODE) )
                return false;
            
            lux_sensors[act_sens].model = mod_BH1750;
            n_sensors_BH++;
            break;
        
        case mod_MAX44009:
            lux_sensors[act_sens].sensor =  new MAX44009();
            
            // If not initialized exit and return false
            if ( ((MAX44009*) lux_sensors[act_sens].sensor)->begin() != 0)
                return false;
            
            lux_sensors[act_sens].model = mod_MAX44009;
            n_sensors_MAX++;
            break;
    }

    return true;
}

const uint16_t Lux_Sensors::get_instant_lux(uint8_t n_sensor) {
    if (n_sensor >= get_n_sensors()) return 0;

    if (lux_sensors[n_sensor].model == mod_BH1750)
        return (uint16_t) ((BH1750*) lux_sensors[n_sensor].sensor)->readLightLevel();

    if (lux_sensors[n_sensor].model == mod_MAX44009)
        return (uint16_t) ((MAX44009*) lux_sensors[n_sensor].sensor)->get_lux();

    return 0;
}

void Lux_Sensors::capture_all_sensors() {
    uint8_t act_sens = n_sensors_BH + n_sensors_MAX;

    for (uint8_t i=0; i<act_sens; i++) {
        lux_sensors[i].read_val = get_instant_lux(i);
    }
}

void Lux_Sensors::set_n_samples(uint8_t _n_samples) {
    n_samples = _n_samples;
}

const uint8_t Lux_Sensors::get_n_samples() {
    return n_samples;
}

const uint8_t Lux_Sensors::get_n_sensors() {
    return (n_sensors_BH + n_sensors_MAX);
}

Lux_Sensors::Lux_Sensor_model_t Lux_Sensors::get_model_sensors(uint8_t n_sensor) {
    if (n_sensor >= get_n_sensors())
        return mod_UNDEFINED;

    return lux_sensors[n_sensor].model;
}

void Lux_Sensors::bulk_results(String &str, bool reset, bool print_tag, char delim) {
    if (reset) str.remove(0);                              // Indicates whether the string should be deleted before entering the new values
    
    for (uint8_t i=0; i<get_n_sensors(); i++) {
        if (i && delim != '\0') str.concat(delim);
        if (print_tag) {
            str.concat("lux");
            str += i + 1;
            str.concat("=");
        }
        str.concat(lux_sensors[i].read_val);
    }
}
