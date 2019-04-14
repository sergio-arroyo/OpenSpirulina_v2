/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Fran Romero https://github.com/yatan
 *         Sergio Arroyo (UOC)
 * 
 * General configuration
 * 
 */
#ifndef OpenSpir_configuration_h
#define OpenSpir_configuration_h

#include "OpenSpir_Shield_conn.h"

//===========================================================
//======================= Digital pins ======================
//===========================================================
#define CALIBRATION_SWITCH_PIN     43                      // Pin for pH calibration switch
#define ONE_WIRE_PIN               OPENSPIR_VGA_PIN14      // Where 1-Wire is connected
#define SD_CARD_SS_PIN             PIN_SPI_SS              // Slave Select/Pin lector SD (PIN_SPI_SS = 53)


#define LUX_SENSOR_ADDR            OPENSPIR_VGA_PIN7       // Pin ADDR for apply HIGH level (5v) to assign 0x5C address


//===========================================================
//======================= Serial debug ======================
//===========================================================
#define DEBUG_DEF_ENABLED          1                       // Indicates whether serial debugging is enabled or not by default
#define SERIAL_MON                 Serial                  // Serial output for debug console
#define SERIAL_BAUD                9600                    // Data rate in bits per second (baud)

//===========================================================
//======================== GSM Modem ========================
//===========================================================
#define TINY_GSM_MODEM_A6                                  // Select the GPRS modem
#define TINY_GSM_RX_BUFFER         512                     // Increase the buffer
#define SERIAL_AT                  Serial2                 // Serial port for GPRS Modem
#define DUMP_AT_COMMANDS           0                       // Active if you want to see all AT commands (0=disabled, 1=enabled)

//===========================================================
//=========================== RTC ===========================
//===========================================================
#define RTC_DEF_ENABLED            1                       // Indicates whether LCD is enabled or not by default

//===========================================================
//=========================== LCD ===========================
//===========================================================
#define LCD_DEF_ENABLED            1                       // Indicates whether LCD is enabled or not by default
#define LCD_I2C_ADDR               0x3F                    // LCD I2C address 0x27 - Alter address 0x3F
#define LCD_COLS                   20                      // LCD columns and rows definition
#define LCD_ROWS                   4                       //
#define LCD_BACKLIGHT_ENABLED      1                       // Set the LCD backlight screen to on/off
#define LCD_CONTRAST               127                     // Set the contrast value of the display (0-255)
#define LCD_INIT_MSG_L1            "   OpenSpirulina"      // Start message that will be displayed on the screen
#define LCD_INIT_MSG_L2            "   -------------"      //
#define LCD_INIT_MSG_L3            ""                      //
#define LCD_INIT_MSG_L4            "       Please wait.."  //

//===========================================================
//=========================== SD ============================
//===========================================================
#define SD_SAVE_DEF_ENABLED        1                       // Indicates whether save data sensors on SD is enabled or not by default
#define SD_MAX_FILENAME_SIZE       14                      // Defines de max size of filename

//===========================================================
//=========================== etc ===========================
//===========================================================
#define INI_FILE_NAME              "/config.ini"           // Filename of config ini file
#define INI_FILE_BUFFER_LEN        80                      // Indicates the size of the buffer to get values from the start file

#define STEP_DELAY_TIME            100                     // Wait value (in ms) between reading intervals
#define DELAY_SECS_NEXT_READ       30                      // Timer (in seconds) of waiting between readings of the sensors

//===========================================================
//======================== DO sensor ========================
//===========================================================
#define DO_SENS_ADDR               0x23                    // I2C address for module communication
#define DO_SENS_R_LED_PIN          OPENSPIR_VGA_PIN1       // Pin for DO Red LED
#define DO_SENS_G_LED_PIN          OPENSPIR_VGA_PIN6       // Pin for DO Green LED
#define DO_SENS_B_LED_PIN          OPENSPIR_VGA_PIN2       // Pin for DO Blue LED
#define DO_SENS_N_SAMP_READ        10                      // Number of samples of DO
#define DO_SENS_MS_WAIT            1000                    // Time (in ms) to wait before capture lux

//===========================================================
//====================== Current sensor =====================
//===========================================================
#define CURR_SENS_N_SAMPLES        10                      // Number of samples to read to make the average
#define CURR_SENS_MS_INTERV        100                     // Milliseconds to wait between the read intervals
#define CURR_SENS_MS_BE            30L * 1000L             // Time in seconds between current ini current end measure

#endif
