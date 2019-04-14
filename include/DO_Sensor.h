/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Fran Romero
 *         Sergio Arroyo (UOC)
 * 
 * General methods and functions related to the optical density sensor,
 * designed by OpenSpirulina
 * 
 */
#ifndef DO_SENSOR_h
#define DO_SENSOR_h

#include <BH1750.h>

class DO_Sensor: public BH1750 {
private:
    bool initialized;
    uint8_t R_pin;                                         // Pinout for Red, Green and Blue LED connected to DO
    uint8_t G_pin;                                         //
    uint8_t B_pin;                                         //
    uint8_t n_samples;                                     // Number of samples to obtain for each reading process
    uint16_t ms_wait;                                      // Milliseconds to wait in each reading cycle
    struct buffer_t {                                      // Results of DO sensors [Red, Green, Blue, White]
		float R_value;
		float G_value;
		float B_value;
		float W_value;
    } RGBW_results;                        

    float filter_result(float* list, uint8_t n_samples);   // Calculate the efective value
    void  sort_result(float* arr, const uint8_t size);     // Sort the list of values obtained from lux sensor

public:
    DO_Sensor(uint8_t _addr, uint8_t _R_pin, uint8_t _G_pin, uint8_t _B_pin,
              uint8_t _n_samples, uint16_t _ms_wait);

    bool  begin(Mode mode);
    void  capture_DO();
    float capture_Red_LED();
    float capture_Green_LED();
    float capture_Blue_LED();
    float capture_White_LED();

    const float get_Red_value();                           // Return the readed values from sensor for R, G, B & W leds
    const float get_Green_value();                         //
    const float get_Blue_value();                          //
    const float get_White_value();                         //

    void set_n_samples(uint8_t _n_samples);
    uint8_t get_n_samples();
    void set_ms_wait(uint16_t _ms_wait);
    uint16_t get_ms_wait();
    bool is_init();
};

#endif
