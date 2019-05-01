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
    /**
     * Constructor
     * 
     * @param lcd_addr  Addres I2C for LCD module
     * @param lcd_cols  Number of columns
     * @param lcd_rows  Number of rows
	 * @param contrast  LCD constrar
     * @param backlight LCD bightness
     **/
	LCD_Screen(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows,
				uint8_t contrast, uint8_t backlight);

    /**
     * Initialize the LCD module
     **/
	void init();

    /**
     * Clear screen and set the cursor to 0,0 position
     **/
	void clear();

    /**
     * Shows the initial message on the screen
     * 
     * @param L1 Message for the first line
     * @param L2 Message for the second line
     * @param L3 Message for the third line
	 * @param L4 Message for the fourth line
     * @param msWait Time to shows the message (in ms)
     **/
	void show_init_msg(const char L1[], const char L2[],
						const char L3[], const char L4[],
						const uint16_t msWait);
    
    /**
     * Shows message on specific position
     * 
     * @param col Column position
     * @param row Row position
     * @param msg Message that you want to display
     **/
	void print_msg(const uint8_t col, const uint8_t row, const char msg[]);

    /**
     * Shows message and value
     * 
     * @param msg Message that you want to display
     * @param value The value to show after the message
     **/
	void print_msg_val(const char msg[], float value);

    /**
     * Shows message on specific position
     * 
     * @param col Column position
     * @param row Row position
     * @param msg Message that you want to display
     * @param value The value to show after the message
     **/
	void print_msg_val(const uint8_t col, const uint8_t row, const char msg[], const float value);

    /**
     * Shows message on specific position
     * 
     * @param col Column position
     * @param row Row position
     * @param msg str_formated Structute of the message to be displayed
     * @param value The value to show after the message
     **/
	void print_msg_val(const uint8_t col, const uint8_t row, const char str_formated[], const int32_t value);

    /**
     * Add a label and its value on the data displayed on the screen
     * 
     * @param msg str_formated Structute of the message to be displayed
     * @param value The value to show after the message
     **/
	void add_value_read(const char msg[], const float value);

private:
	boolean backlight_enabled;
	uint8_t contrast_level;
	uint8_t x_pos;
	uint8_t y_pos;
};

#endif
