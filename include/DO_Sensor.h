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
public:
    /**
     * Constructor
     **/
    DO_Sensor();

    /**
     * Initialize the DO module
     * 
     * @param _addr  Indicates the address for BH1750 lux sensor 
     * @param _R_pin The pin that red LED is connected
     * @param _G_pin The pin that green LED is connected
     * @param _B_pin The pin that blue LED is connected
     * @return returns true if module is initialize correctly, otherwise returns false 
     **/
    bool begin(uint8_t _addr, uint8_t _R_pin, uint8_t _G_pin, uint8_t _B_pin);

    /**
     * Captures the values from sensor for every LED attached and store the results to internal array
     **/
    void capture_DO();
    
    /**
     * Capture the instant lux value without any active LED
     * 
     * @return The calculated instantaneous value of the read and filtered values
     **/
    const float capture_preLux();

    /**
     * Capture the instant lux value with only red active LED
     * 
     * @return The calculated instantaneous value of the read and filtered values
     **/
    const float capture_Red_LED();

    /**
     * Capture the instant lux value with only green active LED
     * 
     * @return The calculated instantaneous value of the read and filtered values
     **/
    const float capture_Green_LED();

    /**
     * Capture the instant lux value with only blue active LED
     * 
     * @return The calculated instantaneous value of the read and filtered values
     **/
    const float capture_Blue_LED();

    /**
     * Capture the instant lux value with all active LEDs
     * 
     * @return The calculated instantaneous value of the read and filtered values
     **/
    const float capture_White_LED();

    /**
     * Capture the instant lux value without any active LED
     * 
     * @return The calculated instantaneous value of the read (only one read)
     **/
    const float get_instant_lux();

    /**
     * Get the value captured and stored in the result array for preLux mode
     * 
     * @return The stored value for preLux mode
     **/
    const float get_preLux_value();

    /**
     * Get the value captured and stored in the result array for red LED mode
     * 
     * @return The stored value for red LED mode
     **/
    const float get_Red_value();

    /**
     * Get the value captured and stored in the result array for green LED mode
     * 
     * @return The stored value for green LED mode
     **/
    const float get_Green_value();

    /**
     * Get the value captured and stored in the result array for blue LED mode
     * 
     * @return The stored value for blue LED mode
     **/
    const float get_Blue_value();

    /**
     * Get the value captured and stored in the result array for RGB LEDs mode
     * 
     * @return The stored value for RGB LED mode
     **/
    const float get_White_value();

    /**
     * Update the number of samples to obtain in each reading
     * 
     * @param _n_samples Number of samples to obtain
     **/
    void set_n_samples(uint8_t _n_samples);

    /**
     * Get the number of samples to obtain in each reading
     * 
     * @return Number of samples to obtain
     **/
    uint8_t get_n_samples();

    /**
     * Update the time (in ms) to wait between each each read sample
     * 
     * @param _ms_reads Time to wait (in ms)
     **/
    void set_ms_reads(uint16_t _ms_reads);

    /**
     * Get the time (in ms) to wait between each each read sample
     * 
     * @return Time to wait (in ms)
     **/
    uint16_t get_ms_reads();

    /**
     * Indicates whether the module is started or not
     * 
     * @return Return true if the module is initialized, otherwise false
     **/
    bool is_init();

private:
    bool initialized;
    BH1750* bh1750_dev;                                    // Pointer to BH1750 instance

    uint8_t R_pin;                                         // Pinout for Red, Green and Blue LED connected to DO
    uint8_t G_pin;                                         //
    uint8_t B_pin;                                         //
    uint8_t n_samples;                                     // Number of samples to obtain for each reading process
    uint16_t ms_reads;                                     // Milliseconds to wait in each reading cycle

    struct buff_lux_t {                                    // Results of DO sensors [Red, Green, Blue, White]
        float preLux_value;
		float R_value;
		float G_value;
		float B_value;
		float W_value;
    } lux_results;                        

    const float capture_and_filter();                      // Capture a number of lux read values and calculate the mean value
};

#endif
