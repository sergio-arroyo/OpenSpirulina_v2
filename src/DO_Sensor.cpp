/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * General methods and functions related to the optical density sensor,
 * designed by OpenSpirulina
 * 
 */

#include "DO_Sensor.h"


DO_Sensor::DO_Sensor() {
    n_samples    = DO_SENS_N_SAMP_READ;
    ms_reads     = DO_SENS_MS_READS;
    lux_results  = {0, };
    initialized  = false;
}

bool DO_Sensor::begin(uint8_t _addr, uint8_t _R_pin, uint8_t _G_pin, uint8_t _B_pin) {
    if (initialized) return true;                          // If the sensor already started, return true and not tray more

    R_pin = _R_pin;                                        // LEDs pin assign
    G_pin = _G_pin;
    B_pin = _B_pin;
    pinMode(R_pin, OUTPUT);                                // pin mode to output
    pinMode(G_pin, OUTPUT);
    pinMode(B_pin, OUTPUT);

    bh1750_dev = new BH1750(_addr);                        // Instanciate new BH1750 object

    // If not initialized exit and return false
    if (!bh1750_dev->begin(BH1750::Mode::CONTINUOUS_HIGH_RES_MODE_2))
        return false;                                      // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
    
    initialized = true;
    return true;
}

void DO_Sensor::capture_DO() {
    lux_results.preLux_value = capture_preLux();           // Get pre Lux value without any actived led
    lux_results.R_value = capture_Red_LED();               // Get the values for each LED color from the DO
    lux_results.G_value = capture_Green_LED();
    lux_results.B_value = capture_Blue_LED();
    lux_results.W_value = capture_White_LED();
}

const float DO_Sensor::capture_preLux() {
    return capture_and_filter();
}

const float DO_Sensor::capture_Red_LED() {
    digitalWrite(R_pin, HIGH);                             // Activate red LED
    delay(500);

    float result = capture_and_filter();
    digitalWrite(R_pin, LOW);                              // Deactivate red LED

    return result;
}

const float DO_Sensor::capture_Green_LED() {
    digitalWrite(G_pin, HIGH);                                       // Activate green LED
    delay(500);

    float result = capture_and_filter();
    digitalWrite(G_pin, LOW);                                        // Deactivate green LED

    return result;
}

const float DO_Sensor::capture_Blue_LED() {
    digitalWrite(B_pin, HIGH);                                       // Activate blue LED
    delay(500);

    float result = capture_and_filter();
    digitalWrite(B_pin, LOW);                                        // Deactivate blue LED

    return result;
}

const float DO_Sensor::capture_White_LED() {
    digitalWrite(R_pin, HIGH);                                       // Activate R,G,B LEDs
    digitalWrite(G_pin, HIGH);
    digitalWrite(B_pin, HIGH);
    delay(500);

    float result = capture_and_filter();
    digitalWrite(R_pin, LOW);                                        // Deactivate R,G,B LEDs
    digitalWrite(G_pin, LOW);
    digitalWrite(B_pin, LOW);

    return result;
}

const float DO_Sensor::get_instant_lux() {
    return bh1750_dev->readLightLevel();
}

const float DO_Sensor::get_preLux_value() {
    return lux_results.preLux_value;
}

const float DO_Sensor::get_Red_value() {
    return lux_results.R_value;
}

const float DO_Sensor::get_Green_value() {
    return lux_results.G_value;
}

const float DO_Sensor::get_Blue_value() {
    return lux_results.B_value;
}

const float DO_Sensor::get_White_value() {
    return lux_results.W_value;
}

const float DO_Sensor::capture_and_filter() {
    float read_v, min_v=0, max_v=0, total_v=0;

    for (uint8_t i=n_samples; i>0; i--) {
        read_v = bh1750_dev->readLightLevel();
        total_v += read_v;

        if (read_v < min_v) min_v = read_v;                          // Update de min value
        if (read_v > max_v) max_v = read_v;                          // Update de max value

        delay(ms_reads);
    }
    total_v = total_v - min_v - max_v;                               // Discards lower and higher value for the average

    return (total_v / (n_samples-2));
}

void DO_Sensor::set_n_samples(const uint8_t _n_samples) {
    n_samples = _n_samples;
}

uint8_t DO_Sensor::get_n_samples() {
    return n_samples;
}

void DO_Sensor::set_ms_reads(const uint16_t _ms_reads) {
    ms_reads = _ms_reads;
}

uint16_t DO_Sensor::get_ms_reads() {
    return ms_reads;
}

bool DO_Sensor::is_init() {
    return initialized;
}

void DO_Sensor::bulk_results(String &str, bool reset, bool print_tag, bool print_value, char delim) {
    if (reset) str.remove(0);                              // Delete string before entering the new values
    if (str != "") str.concat(delim);                      // If string is not empty, add delimiter
    
    if (print_tag) {                                       // preLux value
        str.concat(F("DO_pLux"));
        if (print_value) str.concat(F("="));
    }
    if (print_value) str.concat(lux_results.preLux_value);
    str.concat(delim);

    if (print_tag) {                                       // Red value
        str.concat(F("DO_R"));
        if (print_value) str.concat(F("="));
    }
    if (print_value) str.concat(lux_results.R_value);
    str.concat(delim);

    if (print_tag) {                                       // Green value
        str.concat(F("DO_G"));
        if (print_value) str.concat(F("="));
    }
    if (print_value) str.concat(lux_results.G_value);
    str.concat(delim);

    if (print_tag) {                                       // Blue value
        str.concat(F("DO_B"));
        if (print_value) str.concat(F("="));
    }
    if (print_value) str.concat(lux_results.B_value);
    str.concat(delim);
    
    if (print_tag) {                                       // White (RGB) value
        str.concat(F("DO_W"));
        if (print_value) str.concat(F("="));
    }
    if (print_value) str.concat(lux_results.W_value);
}
