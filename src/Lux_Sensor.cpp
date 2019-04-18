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

/* Capture the lux value from sensor */
const float Lux_Sensor::capture_lux() {
    float iir[n_samples];

    for (uint8_t i=0; i<n_samples; i++) {
        iir[i] = bh1750_dev->readLightLevel();             // Read value from the lux sensor
    }
    return filter_result(iir, n_samples);
}

/* Sort a list of value items and calculate the efective value */
const float Lux_Sensor::filter_result(float* list, uint8_t n_samples) {
    sort_result(list, n_samples);
    
    // Calculate the final result
    float sum_values = 0;
    for (uint8_t r=1; r < (n_samples-1); r++)                        // Sums the values except the first and last elements
        sum_values += list[r];
    
    return (sum_values / (n_samples-2));
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

void Lux_Sensor::sort_result(float* arr, const uint8_t n_samples) {
    float tmp_val;

    for (int i=0; i < (n_samples-1); i++) {
        for (int j=0; j < (n_samples-(i+1)); j++) {
            if (arr[j] > arr[j+1]) {
                tmp_val = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = tmp_val;
            }
        }
    }
}
