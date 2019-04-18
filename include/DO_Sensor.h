/**
 * OpenSpirulina http://www.openspirulina.com
 * 
 * Autors: Sergio Arroyo (UOC)
 * 
 * General methods and functions related to the optical density sensor,
 * designed by OpenSpirulina
 * 
 */
#ifndef DO_Sensor_h
#define DO_Sensor_h

#include <Arduino.h>
#include <BH1750.h>
#include "Configuration.h"


class DO_Sensor {
private:
    bool initialized;
    BH1750* bh1750_dev;                                    // Pointer to BH1750 instance

    uint8_t R_pin;                                         // Pinout for Red, Green and Blue LED connected to DO
    uint8_t G_pin;                                         //
    uint8_t B_pin;                                         //
    uint8_t n_samples;                                     // Number of samples to obtain for each reading process
    uint16_t ms_reads;                                     // Milliseconds to wait in each reading cycle
    struct buff_RGB_t {                                    // Results of DO sensors [Red, Green, Blue, White]
		float R_value;
		float G_value;
		float B_value;
		float W_value;
    } RGBW_results;                        

    float filter_result(float* list, uint8_t n_samples);   // Calculate the efective value
    void  sort_result(float* arr, const uint8_t size);     // Sort the list of values obtained from lux sensor

public:
    DO_Sensor();

    bool  begin(uint8_t _addr, uint8_t _R_pin, uint8_t _G_pin, uint8_t _B_pin);
    void  capture_DO();
    const float capture_Red_LED();
    const float capture_Green_LED();
    const float capture_Blue_LED();
    const float capture_White_LED();

    const float readLightLevel();                          // Capture light value without any led activated
    const float get_Red_value();                           // Return the readed values from sensor for R, G, B & W leds
    const float get_Green_value();                         //
    const float get_Blue_value();                          //
    const float get_White_value();                         //

    void set_n_samples(uint8_t _n_samples);
    uint8_t get_n_samples();
    void set_ms_reads(uint16_t _ms_reads);
    uint16_t get_ms_reads();
    bool is_init();
};

#endif
