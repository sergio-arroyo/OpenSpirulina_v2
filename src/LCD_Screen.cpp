/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Fran Romero
 *         Sergio Arroyo (UOC)
 * 
 * General methods and functions related to LCD screen
 * 
 */

#include "LCD_Screen.h"


LCD_Screen::LCD_Screen(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows,
					   uint8_t contrast, uint8_t backlight) :
	LiquidCrystal_I2C(lcd_addr, lcd_cols, lcd_rows) {
	
	backlight_enabled = backlight;
	contrast_level = contrast;
}

// LCD initialize module
void LCD_Screen::init() {
	LiquidCrystal_I2C::init();
	setBacklight(backlight_enabled? 0x1:0x0);
	setContrast(contrast_level);
}

void LCD_Screen::clear() {
	LiquidCrystal_I2C::clear();
	x_pos = 0;
	y_pos = 0;
}

// LCD initial messages
void LCD_Screen::show_init_msg(const char L1[], const char L2[],
								const char L3[], const char L4[],
								const unsigned long msWait) {
	clear();        print(L1);
	setCursor(0,1);	print(L2);
	setCursor(0,2);	print(L3);
	setCursor(0,3);	print(L4);
	delay(msWait);
	clear();
}

void LCD_Screen::print_msg(const uint8_t col, const uint8_t row, const char msg[]) {
	setCursor(col, row);
	print(msg);
}

void LCD_Screen::print_msg_val(const char msg[], const float value) {
	char buffer_val[8];                          // char buffer/string

	dtostrf(value, 4, 2, buffer_val);            // Convert float to string
	print(msg);
	print(buffer_val);
}

void LCD_Screen::print_msg_val(const uint8_t col, const uint8_t row, const char msg[], const float value) {
	setCursor(col, row);
	print_msg_val(msg, value);
}

void LCD_Screen::print_msg_val(const uint8_t col, const uint8_t row, const char str_formated[], const int32_t value) {
	char buffer_val[22];                         // char buffer/string

	sprintf(buffer_val, str_formated, value);    // Convert int to string
	setCursor(col, row);
	print(buffer_val);
}

void LCD_Screen::add_value_read(const char msg[], const float value) {
	print_msg(x_pos, y_pos, "          ");       // Clean position in case there was already data
	print_msg_val(x_pos, y_pos, msg, value);
	x_pos += 10;

	if (x_pos >= 20) {
		x_pos = 0;
		y_pos++;
	}
	if (y_pos > 3) {
		x_pos = 0;
		y_pos = 0;
	}
}

