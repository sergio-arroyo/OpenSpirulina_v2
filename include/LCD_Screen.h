/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Fran Romero
 *         Sergio Arroyo (UOC)
 * 
 * General methods and functions related to LCD screen
 * 
 */
#ifndef LCD_Screen_h
#define LCD_Screen_h

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "Configuration.h"

class LCD_Screen : public LiquidCrystal_I2C {
public:
	LCD_Screen(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

	void init();
	void clear();
	void show_init_msg(const char L1[], const char L2[],
						const char L3[], const char L4[],
						const unsigned long msWait);
	void print_msg(const uint8_t col, const uint8_t row, const char msg[]);
	void print_msg_val(const char msg[], float value);
	void print_msg_val(const uint8_t col, const uint8_t row, const char msg[], const float value);
	void print_msg_val(const uint8_t col, const uint8_t row, const char str_formated[], const int32_t value);
	void add_value_read(const char msg[], const float value);

private:
	boolean backlight_enabled;
	uint8_t contrast_level;
	uint8_t x_pos;
	uint8_t y_pos;
};

#endif
