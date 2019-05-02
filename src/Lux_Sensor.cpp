/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * General methods and functions related to the optical density sensor,
 * designed by OpenSpirulina
 * 
 */

#include "Lux_Sensor.h"


Lux_Sensor::Lux_Sensor() {
    n_samples   = LUX_SENS_N_SAMP_READ;
    initialized = false;
}

bool Lux_Sensor::begin(uint8_t addr, uint8_t addr_pin) {
    if (initialized) return true;                          // If the sensor already started, return true and not tray more
    
    if (addr_pin != 0) {
        pinMode(addr_pin, OUTPUT);                         // Sets the digital pin as output
        digitalWrite(addr_pin, HIGH);                      // Sets the digital pin ON
        delay(200);
    }

    bh1750_dev = new BH1750(addr);                         // Instanciate new BH1750 object
    
    // If not initialized exit and return false
    if (!bh1750_dev->begin(BH1750::Mode::CONTINUOUS_HIGH_RES_MODE_2))
        return false;                                      // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
    
    initialized = true;
    return true;
}

const float Lux_Sensor::capture_lux() {
    return capture_and_filter();
}

const float Lux_Sensor::capture_and_filter() {
    float read_v, min_v=0, max_v=0, total_v=0;

    for (uint8_t i=n_samples; i>0; i--) {
        read_v = bh1750_dev->readLightLevel();
        total_v += read_v;

        if (read_v < min_v) min_v = read_v;                          // Update de min value
        if (read_v > max_v) max_v = read_v;                          // Update de max value

        delay(10);
    }
    total_v = total_v - min_v - max_v;                               // Discards lower and higher value for the average
    
    return (total_v / (n_samples-2));
}

void Lux_Sensor::set_n_samples(const uint8_t _n_samples) {
    n_samples = _n_samples;
}

uint8_t Lux_Sensor::get_n_samples() {
    return n_samples;
}

bool Lux_Sensor::is_init() {
    return initialized;
}
