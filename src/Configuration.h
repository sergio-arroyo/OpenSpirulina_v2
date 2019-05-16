/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * General configuration
 * 
 */
#ifndef OpenSpir_configuration_h
#define OpenSpir_configuration_h

#include "OpenSpir_Shield_conn.h"

//===========================================================
//======================= Serial debug ======================
//===========================================================
#define DEBUG_DEF_ENABLED          1                       // Indicates whether serial debugging is enabled or not by default
#define SERIAL_MON                 Serial                  // Serial output for debug console
#define SERIAL_BAUD                115200                  // Data rate in bits per second (baud)


//===========================================================
//======================== GSM Modem ========================
//===========================================================
#define TINY_GSM_MODEM_A6                                  // Select the GPRS modem
#define TINY_GSM_RX_BUFFER         512                     // Increase the buffer
#define SERIAL_AT                  Serial2                 // Serial port for GPRS Modem
#define DUMP_AT_COMMANDS           0                       // Active if you want to see all AT commands (0=disabled, 1=enabled)

#define GPRS_APN                   "internet"              // GPRS credentials
#define GPRS_USER                  ""                      //
#define GPRS_PASS                  ""                      //


//===========================================================
//==================== Report server v1 =====================
//===========================================================
#define HTTP_REP1_SERVER       "sensors.openspirulina.com" // Server connect for sending data
#define HTTP_REP1_PORT             80                      // Server port
#define HTTP_REP1_ID_ARDUINO       "21"                    // Define identity of Arduino
#define HTTP_REP1_PIN_ARDUINO      "12345"                 // Define pin


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
#define LCD_INIT_TIMEOUT           5000                    // Time that the start message is displayed


//===========================================================
//=========================== SD ============================
//===========================================================
#define SD_CARD_SS_PIN             PIN_SPI_SS              // Slave Select/Pin lector SD (PIN_SPI_SS = 53)
#define SD_SAVE_DEF_ENABLED        1                       // Indicates whether save data sensors on SD is enabled or not by default
#define SD_MAX_FILENAME_SIZE       14                      // Defines de max size of filename
#define SD_INI_CFG_FILENAME        "/config.ini"           // Filename of config ini file
#define SD_DATA_DELIMITED          '#'                     // Char delimiter for tags & data bulks in SD

#define INI_FILE_BUFFER_LEN        80                      // Indicates the size of the buffer to get values from the start file


//===========================================================
//=========================== etc ===========================
//===========================================================
#define DELAY_SECS_NEXT_READ       30                      // Timer (in seconds) of waiting between readings of the sensors


//===========================================================
//======================= DHT sensor ========================
//===========================================================
#define DHT_DEF_NUM_SENSORS        1                       // Number of sensors actived by default
const uint8_t DHT_DEF_SENSORS[] =  {OPENSPIR_VGA_PIN4};    // Array for default pin for DHT sensors
#define DHT_DEF_TYPE               DHT22                   // Default DHT sensors type
#define DHT_MAX_SENSORS            5                       // Maximum number of sensors that will be allowed


//===========================================================
//======================= Lux sensor ========================
//===========================================================
#define LUX_SENS_ACTIVE            1                       // Indicates whether lux sensors is enabled or not by default
#define LUX_SENS_ADDR              0x5C                    // Pin ADDR for apply HIGH level (5v) to assign 0x5C address
#define LUX_SENS_ADDR_PIN          OPENSPIR_VGA_PIN7       // Pin ADDR for apply HIGH level (5v) to assign 0x5C address
#define LUX_SENS_N_SAMP_READ       10                      // Number of samples read from sensor

#define LUX_SENS_DEF_NUM          2                        // Number of current sensors by default
const uint8_t LUX_SENS_DEF_MODELS[]   = {1, 2};            // 1=BH1750 model, 2=MAX44009
const uint8_t LUX_SENS_DEF_ADDRESS[]  = {0x5C, 0x4A};      // Array for default address for lux sensors
const uint8_t LUX_SENS_DEF_ADDR_PIN[] = {34, 0};


//===========================================================
//======================== DO sensor ========================
//===========================================================
#define DO_SENS_ACTIVE             1                       // Indicates whether DO sensor is enabled or not by default
#define DO_SENS_ADDR               0x23                    // I2C address for module communication
#define DO_SENS_R_LED_PIN          OPENSPIR_VGA_PIN1       // Pin for DO Red LED
#define DO_SENS_G_LED_PIN          OPENSPIR_VGA_PIN6       // Pin for DO Green LED
#define DO_SENS_B_LED_PIN          OPENSPIR_VGA_PIN2       // Pin for DO Blue LED
#define DO_SENS_N_SAMP_READ        10                      // Number of samples read from sensor
#define DO_SENS_MS_READS           150                     // Time (in ms) between each reading


//===========================================================
//======================== pH sensor ========================
//===========================================================
#define PH_CALIBRATION_SWITCH_PIN  OPENSPIR_SHIELD_SW1     // Pin for pH calibration switch
#define PH_DEF_NUM_SENSORS         1                       // Number of sensors actived by default
const uint8_t PH_DEF_PIN_SENSORS[] =  {OPENSPIR_SHIELD_J1  // Array for default pins for pH sensors
                                      };
#define PH_MAX_NUM_SENSORS         3                       // Maximum number of pH sensors that can be connected
#define PH_SENS_N_SAMP_READ        10                      // Number of samples read from sensor


//===========================================================
//================== WP temperature sensor ==================
//===========================================================
#define WP_T_ONE_WIRE_PIN          OPENSPIR_VGA_PIN14      // Where 1-Wire is connected
#define WP_T_MAX_PAIRS_SENS        4

#define WP_T_DEF_NUM_PAIRS         2                       // Define the number of sensor pairs by default
const uint8_t WP_T_DEF_SENST_PAIRS[][2][8] = {             // Define the pairs
    {   // Define pair of Temp1 sensors:
        {0x28, 0xFF, 0x1B, 0xD2, 0x24, 0x17, 0x03, 0x28},
        {0x28, 0xFF, 0x72, 0x88, 0x24, 0x17, 0x03, 0x09}
    },
    {   // Define pair of Temp2 sensors:
        {0x28, 0xFF, 0x89, 0xBB, 0x60, 0x17, 0x05, 0x6D},
        {0x28, 0xFF, 0xCA, 0xE5, 0x80, 0x14, 0x02, 0x16}
    }
};


//===========================================================
//====================== Current sensor =====================
//===========================================================
#define CURR_MAX_NUM_SENSORS       6                       // Maximum number of current sensors that can be connected
#define CURR_SENS_N_SAMPLES        100                     // Number of samples to read to make the average (value between 1-255)
#define CURR_SENS_MS_INTERV        1                       // Milliseconds to wait between the read intervals
#define CURR_SENS_MS_BE            30L * 1000L             // Time in seconds between current ini current end measure

#define CURR_SENS_DEF_NUM          2                       // Number of current sensors by default
const uint8_t CURR_SENS_DEF_PINS[] = {OPENSPIR_SHIELD_J4,  // Array for default pins for current sensors
                                      OPENSPIR_SHIELD_J5};
const uint8_t CURR_SENS_DEF_MODELS[] = {0, 1};
const uint16_t CURR_SENS_DEF_VAR[] = {20, 30};


//===========================================================
//======================== CO2 sensor =======================
//===========================================================
#define CO2_DEF_NUM_SENSORS        1
#define CO2_SENS_N_SAMP_READ       15                      // Number of samples read from sensor


//===========================================================
//======================== ORP sensor =======================
//===========================================================
#define ORP_DEF_NUM_SENSORS        1                       // Number of sensors actived by default
const uint8_t ORP_DEF_ADDRS[] =    {0x62};                 // Array for default pin for ORP sensors
#define ORP_MAX_SENSORS            5                       // Maximum number of sensors that will be allowed


#endif
