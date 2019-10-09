/*
 * Monitoring of spirulina cultures
 *
 * Autors:
 *   OpenSpirulina http://www.openspirulina.com
 *   Sergio Arroyo https://github.com/sergio-arroyo
 * 
 */
#include <Arduino.h>
#include "Configuration.h"                                 // General configuration file
#include "OS_def_types.h"                                  // Define structures and enumerations 

// Third-party libs
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <MemoryFree.h>

// OpenSpirulina libs
#include "Load_SD_Config.h"                                // Read the initial configuration file
#include "Data_send.h"                                     // Send sensors data to remote server
#include "DateTime_RTC.h"                                  // Class to control RTC date time
#include "LCD_Screen.h"                                    // Class for LCD control
#include "DHT_Sensors.h"                                   // Class for control all DHT sensors
#include "DO_Sensor.h"                                     // Class for DO (Optical Density) control
#include "Lux_Sensors.h"                                   // Class for lux sensor control
#include "PH_Sensors.h"                                    // Class for pH sensors control
#include "WP_Temp_Sensors.h"                               // Class for DS18 waterproof temperature sensors control
#include "Current_Sensors.h"                               // Class for current sensors control
#include "ORP_Sensors.h"                                   // Class for ORP (Oxydo Reduction Potential) sensors control
#include "MQTT_Pub.h"                                      // Class responsible for sending MQTT messaging to the remote broker
#include "OS_Actuators.h"                                  // Class responsible for interacting with external devices (such as relays, etc.)


/*****************
 * GLOBAL CONTROL
 *****************/
Culture_ID_st culture_ID = {CULTURE_ID_COUNTRY,            // Identify the culture in a unique way
                            CULTURE_ID_CITY,
                            CULTURE_ID_CULTURE,
                            CULTURE_ID_HOST};

bool DEBUG = DEBUG_DEF_ENABLED;                            // Indicates whether the debug mode on serial monitor is active
bool LCD_enabled = LCD_DEF_ENABLED;                        // Indicates whether the LCD is active
bool RTC_enabled = RTC_DEF_ENABLED;                        // Indicates whether the RTC is active
bool SD_save_enabled = SD_SAVE_DEF_ENABLED;                // Indicates whether the save to SD is enabled
bool perf_pH_calib = false;                                // Indicates whether the calibration of the pH module should be carried out

DateTime_RTC dateTimeRTC;                                  // RTC class object (DS3231 clock sensor)

LCD_Screen lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS,
                LCD_CONTRAST, LCD_BACKLIGHT_ENABLED);      // LCD screen

DHT_Sensors dht_sensors;                                   // Handle all DHT sensors
DO_Sensor do_sensor;                                       // DO (Optical Density) sensor

Lux_Sensors *lux_sensors;                                  // Lux sensor with BH1750
PH_Sensors *pH_sensors;                                    // pH sensors class
WP_Temp_Sensors *wp_t_sensors;                             // DS18B20 Sensors class
Current_Sensors *curr_sensors;                             // Current sensors
ORP_Sensors *orp_sensors;                                  // ORP sensors

float array_CO2[CO2_DEF_NUM_SENSORS];                      // Array of CO2 sensors

File objFile;
char fileName[SD_MAX_FILENAME_SIZE] = "";                  // Name of file to save data readed from sensors

Internet_cnn_type cnn_option = NET_DEF_CNN_TYPE;           // None | Ethernet | GPRS Modem | Wifi <-- Why not? Dream on it

uint8_t eth_mac[6] = {0,};                                 // MAC address for Ethernet W5100
bool cnn_init = false;                                     // Indicates whether the connection is active

char last_send[10] = "";
uint16_t loop_count = 0;                                   // Count reading cycles

MQTT_Pub *mqtt_pub;                                        // MQTT publisher client control
OS_Actuators *os_actuators;                                // External actuators;
EthernetServer *web_server;                                // WebServer responsible for attending external requests


/*****************
 * METHODS
 *****************/

/* Capture light ambient with LDR sensor*/
float capture_LDR(uint8_t s_pin) {
    return analogRead(s_pin);
}

/* Capture CO2 sensor */
float capture_CO2(uint8_t pin) {
    DEBUG_NL(F("Capturing CO2"))

    int32_t total_v=0;
    int16_t read_v, min_v=0, max_v=0;

    for (uint8_t i=CO2_SENS_N_SAMP_READ; i>0; i--) {
        read_v = analogRead(pin);                          // Read ADC value
        read_v *= 5;                                       // Convert adc scale to voltage
        read_v >>= 10;                                     // Shift 10 bits (divide by 1024)
        read_v += 1420;                                    // Apply sensor offset

        if (read_v < min_v) min_v = read_v;
        if (read_v > max_v) max_v = read_v;
        total_v += read_v;

        delay(100);
    }

    total_v -= min_v;                                      // Discards lower and higher value for the average
    total_v -= max_v;

    return total_v / (CO2_SENS_N_SAMP_READ-2);
}

/* Show obteined vales from LCD */
void mostra_LCD() {
    lcd.clear();                        // Clear screen

    // Shows on LCD the average of the two temperatures of  the first pair
    if (wp_t_sensors && wp_t_sensors->get_n_pairs() > 0)
        lcd.add_value_read("T1:", wp_t_sensors->get_result_pair(0, WP_Temp_Sensors::S_Both));
    
    // Shows on LCD the average of the two temperatures of  the first pair
    if (wp_t_sensors && wp_t_sensors->get_n_pairs() > 1)
        lcd.add_value_read("T2:", wp_t_sensors->get_result_pair(1, WP_Temp_Sensors::S_Both));
    
    if (pH_sensors && pH_sensors->get_n_sensors() > 0)
        lcd.add_value_read("pH1:", pH_sensors->get_sensor_value(0));
    
    if (pH_sensors && pH_sensors->get_n_sensors() > 1)
        lcd.add_value_read("pH2:", pH_sensors->get_sensor_value(1));

    if (CO2_DEF_NUM_SENSORS > 0)
        lcd.add_value_read("CO2:", array_CO2[0]);
}

/* Capture data in calibration mode */
void pH_calibration() {
    DEBUG_NL(F("Starting calibration mode.."))
    char buffer_L[6];                                      // String buffer
    
    lcd.clear();                                           // Clear screen
    lcd.print(F("pH Calibration"));
    
    if (pH_sensors && pH_sensors->get_n_sensors() > 0) {
        lcd.setCursor(0, 1);                               // go to the 2nd line
        lcd.print(F("pH1:"));
        dtostrf(pH_sensors->get_sensor_value(0), 4, 2, buffer_L);
        lcd.print(buffer_L);
    }
    
    if (pH_sensors && pH_sensors->get_n_sensors() > 1) {
        lcd.setCursor(0, 2);                               // go to the 3rd line
        lcd.print(F("pH2:"));
        dtostrf(pH_sensors->get_sensor_value(1), 4, 2, buffer_L);
        lcd.print(buffer_L);
    }
}

/* Obtain the name of first free file for writting to SD */
void SD_get_next_FileName(char* _fileName) {
    static int fileCount = 0;

    do {
        sprintf(_fileName, "%06d.txt", ++fileCount);
    } while (SD.exists(_fileName));
}

/**
 * Compose a string with all the results obtained from the sensors in specified format
 * 
 * @param str_out The string result with the specified format
 * @param print_tag Indicates whether the label of each sensor should be displayed
 * @param print_value Indicates whether the value of each sensor should be displayed
 * @param delim Character that indicates the separator of the fields shown
 **/
void compose_structure_results(String &str_out, bool print_tag, bool print_value, char delim) {
    // Bulk current sensors tags: curr1#curr2#...
    if (curr_sensors)
        curr_sensors->bulk_results(str_out, false, print_tag, print_value, delim);

    // Bulk waterproof sensors tags: t1_s#t1_b#t2_s#t2_b#...
    if (wp_t_sensors)
        wp_t_sensors->bulk_results(str_out, false, print_tag, print_value, delim);

    // Bulk pH sensors
    if (pH_sensors)
        pH_sensors->bulk_results(str_out, false, print_tag, print_value, delim);

    // Bulk ORP sensors
    if (orp_sensors)
        orp_sensors->bulk_results(str_out, false, print_tag, print_value, delim);

    // Bulk DHT sensors tags: at1#ah1#atn#ah2#...
    if (dht_sensors.get_n_sensors() > 0)
        dht_sensors.bulk_results(str_out, false, print_tag, print_value, delim);

    // Bulk lux sensors tags: lux1#lux2#...
    if (lux_sensors)
        lux_sensors->bulk_results(str_out, false, print_tag, print_value, delim);
    
    // Bulk DO sensor
    if (do_sensor.is_init()) {
        do_sensor.bulk_results(str_out, false, print_tag, print_value, delim);
    }
    
    // Bulk CO2 sensors: co2_1#co2_2#...
    if (CO2_DEF_NUM_SENSORS > 0) {
        if (str_out != "") str_out.concat(F("#"));

        for (uint8_t i=0; i<CO2_DEF_NUM_SENSORS; i++) {
            if (i) str_out.concat(F("#"));
            str_out.concat(F("co2_"));
            str_out.concat(i);
        }
    }
}

/**
 * Write to SD the information collected from the sensors
 * Performs dump of all result values stored in the data array
 * 
 * @param _fileName The name of destination file t write the data
 * @param print_tag Indicates whether the label of each sensor should be displayed
 * @param print_value Indicates whether the value of each sensor should be displayed
 * @param delim Character that indicates the separator of the fields shown
 **/
void SD_write_data(const char* _fileName, const bool print_tag, const bool print_value, const char delim) {
    objFile = SD.open(_fileName, FILE_WRITE);              // Try to open file

    if (!objFile) {
        DEBUG_NL(F("Error opening SD file!"));
        return;    //Exit
    }
        
    String str_out = "";

    if (RTC_enabled) {                                     // Save datetime from RTC module
        if (print_tag) str_out.concat(F("DateTime"));
        if (print_value) str_out.concat(dateTimeRTC.getDateTime());
    }

    // Bulk all sensors information
    compose_structure_results(str_out, print_tag, print_value, delim);

    DEBUG_V2(F("Write in file: "), str_out)
    
    objFile.println(str_out);                              // Write the string result to file
    objFile.close();                                       // Close the file:
}

bool send_data_http_server(EthernetClass *eth_if, char *host, uint16_t port) {
	String str_out = "";
	
    // Bulk all sensors information
    compose_structure_results(str_out, true, true, '&');   // Bulk all sensors information

	// Send data to specific remote server
    switch (cnn_option) {
        case it_Ethernet:
            if (!cnn_init) {
                // Try to init Ethernet
                cnn_init = ETH_initialize(eth_if, eth_mac);
                if (!cnn_init) return false;
                
                return ETH_send_data_http_server(host, port, &str_out);
            }
            break;
        
        case it_GPRS:
            return MODEM_send_data(&str_out, host, port);
            break;
        
        default: return false;                             // type not defined
    }

    return false;
}

bool send_data_mqtt_broker() {
    String str_out = "";

    // Bulk all sensors information
    compose_structure_results(str_out, true, true, ',');   // Bulk all sensors information

	// Send data to specific hardware
    switch (cnn_option) {
        case it_Ethernet:
            return mqtt_pub->publish_topic(str_out.c_str());
            break;
        
        default:
            return false;                                  // type not defined
    }

    return false;
}

uint8_t WebServer_process_action(String *str) {
    int16_t pos;
    char dev_id[ACT_MAX_DEV_ID_LEN+1] = "";
    uint8_t dev_action;

    pos = str->indexOf('?');                          // clean petition string
    if (pos >= 0) str->remove(0, pos+1);
    pos = str->indexOf('=');                          // Find where is the separator for the parameter and value

    if (pos == -1)                                    // If it does not contain an equal, it is not an action
        return ACT_RES_PARAM_ERROR;

    // Extract device ID
    strncpy(dev_id, str->substring(0, pos).c_str(), ACT_MAX_DEV_ID_LEN);

    // Check ON or OFF action
    if (str->substring(pos+1).equalsIgnoreCase(F("ON"))) {
        dev_action = HIGH;
    } else if (str->substring(pos+1).equalsIgnoreCase(F("OFF"))) {
        dev_action = LOW;
    } else if (str->substring(pos+1).equalsIgnoreCase(F("SWITCH"))) {
        dev_action = 0xFF;                            // Indicates that status switch han been requested
    } else {
        return ACT_RES_PARAM_ERROR;                   // If it isn't ON or OFF, return false
    }
    
    if (DEBUG) {
        SERIAL_MON.print(F("HTTP actuator: Apply <")); SERIAL_MON.print(dev_action);
        SERIAL_MON.print(F("> on <")); SERIAL_MON.print(dev_id); SERIAL_MON.print(F(">.. "));
    }
    
    if (os_actuators->change_state(dev_id, dev_action)) {
        DEBUG_NL(F("SUCCESS"))
        return ACT_RES_PROCESS_OK;
    } else {
        DEBUG_NL(F("ERROR"))
        return ACT_RES_ACT_UNDEF;
    }
}

void WebServer_generate_response(EthernetClient *eth_client, const __FlashStringHelper *code,
                                 const __FlashStringHelper *msg) {
    eth_client->print(F("HTTP/1.1 ")); eth_client->println(code);
    eth_client->println(F("Content-Type: text/html"));
    eth_client->println(F("Access-Control-Allow-Origin: *"));
    eth_client->println(F("Connection: close"));
    eth_client->println();
    eth_client->println(msg);
}

void WebServer_response_status(EthernetClient *eth_client, Culture_ID_st *culture_id, OS_Actuators *actuators) {
    // Header
    eth_client->println(F("HTTP/1.1 200 OK"));
    eth_client->println(F("Content-Type: text/html"));
    eth_client->println(F("Access-Control-Allow-Origin: *"));
    eth_client->println(F("Connection: close"));
    eth_client->println();

    // HTML init
    eth_client->println(F("<!DOCTYPE HTML><html lang = ""en""><meta charset=""UTF-8"" /><head><title>State of the culture actuators</title>"));
    eth_client->println(F("<style type=\"text/css\">"));
    eth_client->println(F("*{font-family:sans-serif}table{width:100%;overflow:hidden;background:#FFF;color:#0373b5;border-collapse:collapse}table th,table td{padding:1em;}table th{border:1px solid #FFF;background-color:#0373b5;color:#FFF;text-align:left}table td{border:1px solid #b9e6ff}table tr:nth-child(odd){background-color:#daecf6}.ch_stat{cursor:pointer;text-decoration:underline}</style>"));
    eth_client->println(F("<script>function send_act(act_id, action) {var xhr = new XMLHttpRequest(); xhr.timeout = 20000; xhr.open(\"GET\", \"/action?\"+ act_id +'='+ action, true);"));
    eth_client->println(F("xhr.onload = function (e) { if (xhr.readyState === 4) { if (xhr.status === 200) { alert('Remote response: '+ xhr.responseText);location.reload(true);} else {alert('Error!! '+ xhr.statusText);}}};"));
    eth_client->println(F("xhr.onerror = function (e) {alert('Error!! '+ xhr.statusText);}; xhr.send(null); } </script>"));
    eth_client->println(F("</head><body>"));

    // culture ID
    eth_client->print(F("<h2>Culture:</h2><b>Country: </b> ")); eth_client->print(culture_id->country);
    eth_client->print(F("<br><b>City: </b> ")); eth_client->print(culture_id->city);
    eth_client->print(F("<br><b>Culture: </b>")); eth_client->print(culture_id->culture);
    eth_client->print(F("<br><b>Host ID: </b>")); eth_client->print(culture_id->host_id);
    eth_client->println(F("<br><br>"));
    eth_client->println(F("<table><tr><th>Device ID</th><th>State</th><th colspan=\"2\">Change state</th></tr>"));

    // print table of devices/actuators
    const char *device_id;
    for (uint8_t i=0; i < actuators->get_n_devices(); i++) {
        eth_client->print(F("<tr><td>"));
        device_id = actuators->get_device_id(i);
        eth_client->print(device_id);
        eth_client->print(F("</td><td>"));

        // indicates if device is ON or OFF
        if (actuators->get_device_state(i) == HIGH) {
            eth_client->print(F("HIGH"));
        } else {
            eth_client->print(F("LOW"));
        }

        eth_client->print(F("</td><td class=\"ch_stat\" onclick='send_act(\""));
        eth_client->print(device_id);
        eth_client->print(F("\",\"ON\")'>Send ON</td><td class=\"ch_stat\" onclick='send_act(\""));
        eth_client->print(device_id);
        eth_client->print(F("\",\"OFF\")'>Send OFF</td></tr>"));
    }
    eth_client->print(F("</table></body><html>"));
}

void WebServer_check_petition() {
    // Check if there are petitions
    EthernetClient eth_client = web_server->available();
    
    // If the ethernet client does not have requests, exit function
    if (!eth_client)
        return;

    String str;
    char c, c_prev = '\0';
    int16_t pos;

    while (eth_client.connected()) {
        if (eth_client.available()) {
            c = eth_client.read();
            str.concat(c);

            if (c == '\n' && c_prev == '\r') {                       // detect new line
                str.trim();                                          // previus clean string
                if (str.startsWith(F("GET "))) {
                    pos = str.indexOf(' ');
                    if (pos >= -1) str.remove(0, pos+1);             // remove GET method string
                    pos = str.indexOf(' ');
                    if (pos >= -1) str.remove(pos);                  // remove HTTP/1.1 string

                    if (str.startsWith(ACT_WEBSRV_ACTIONS_STR)) {
                        switch (WebServer_process_action(&str)) {
                            case ACT_RES_PROCESS_OK:
                                WebServer_generate_response(&eth_client, F("200 OK"), F("ACTION SUCCESS"));
                                break;  // switch
                            
                            case ACT_RES_PARAM_ERROR:
                                WebServer_generate_response(&eth_client, F("400 Bad Request"), F("PARAM. ERROR"));
                                break;  // switch

                            case ACT_RES_ACT_UNDEF:
                                WebServer_generate_response(&eth_client, F("400 Bad Request"), F("ACTUATOR UNDEF."));
                                break;  // switch
                        }
                        
                    }
                    else if (str.startsWith(ACT_WEBSRV_STATUS_STR)) {
                        WebServer_response_status(&eth_client, &culture_ID, os_actuators);
                    }
                } else {
                    // if request is not "/action?" or "/status" response unknown
                    WebServer_generate_response(&eth_client, F("404 Not Found"), F("PETITION UNKNOWN"));
                }

                // emptying of the information transmitted by the client
                // only the first line of request is of interest
                while (eth_client.available()) 
                    eth_client.read();
                
                break;    // exit while. from read data
            }
            c_prev = c;
        }
    }

    delay(10);                                    // wait to client do 
    eth_client.stop();                            // close connection
}

/* Capture the values of all available sensors */
void capture_all_sensors() {
    if (curr_sensors) {
        DEBUG_NL(F("Capture current.."))
        curr_sensors->capture_all_sensors();
    }
    WebServer_check_petition();                            // loop to check possible webserver petitions

    // Si tenim sondes de temperatura
    if (wp_t_sensors) {
		DEBUG_NL(F("Capture WP temperatures.."))
		wp_t_sensors->store_all_results();
	}
    WebServer_check_petition();                            // loop to check possible webserver petitions
    
	// Capture PH for each pH Sensor
    if (pH_sensors) {
        DEBUG_NL(F("Capture pH sensors.. "))
        pH_sensors->capture_all_sensors();
    }
    WebServer_check_petition();                            // loop to check possible webserver petitions

    if (orp_sensors) {
        DEBUG_NL(F("Capture ORP sensors.. "))
        orp_sensors->capture_all_sensors();
    }
    WebServer_check_petition();                            // loop to check possible webserver petitions

    // Capture PH for each pH Sensor
	if (dht_sensors.get_n_sensors() > 0) {
		DEBUG_NL(F("Capture DHT sensors.."))
		dht_sensors.capture_all_sensors();
	}
    WebServer_check_petition();                            // loop to check possible webserver petitions

    if (lux_sensors) {
		DEBUG_NL(F("Capture lux sensor.."))
		lux_sensors->capture_all_sensors();
	}
    WebServer_check_petition();                            // loop to check possible webserver petitions

    //Capture DO values (Red, Green, Blue, and White)
    if (do_sensor.is_init()) {
		DEBUG_NL(F("Capture DO sensor.."))
        do_sensor.capture_DO();
    }
    WebServer_check_petition();                            // loop to check possible webserver petitions
    
    // Capture CO2 concentration
    if (CO2_DEF_NUM_SENSORS > 0) {
		DEBUG_NL(F("Capture CO2 sensor.."))
		capture_CO2(CO2_SENS_DEF_PINS[0]);
	}
    WebServer_check_petition();                            // loop to check possible webserver petitions
}

/* Wait a certain time validating if the calibration switch is pressed
 * The time is calculated with RTC module
 * 
 * @param waiting_secs Time in seconds you want to wait
 * @return True if the calibration switch is active, otherwise false
 **/
bool wait_time_with_RTC(const uint16_t waiting_secs) {
	uint32_t time_next_loop = dateTimeRTC.inc_unixtime(waiting_secs);  // Set next timer loop for actual time + delay time

    DEBUG_V3(F("Waiting for "), waiting_secs, F(" seconds"))
    
    if (LCD_enabled) lcd.print_msg(0, 3, "Next read.:");

    unsigned long prev_S_millis = millis();
    int32_t time_diff;
    do {
        if (digitalRead(PH_CALIBRATION_SWITCH_PIN) == HIGH)
            return true;
        
        // Calculates remaining time
        time_diff = dateTimeRTC.unix_time_diff(time_next_loop);
        
        // Updates the remaining timeout
        if ((millis() - prev_S_millis) >= 1000) {
            if (LCD_enabled) lcd.print_msg_val(12, 3, "%ds ", time_diff);
            DEBUG_NN(F("."))
            prev_S_millis = millis();
        }

        WebServer_check_petition();                        // loop to check possible webserver petitions
    } while (time_diff > 0);

    return false;            // Exit without active de calibration switch
}

/* Wait a certain time validating if the calibration switch is pressed
 * The time is calculated without RTC module
 * 
 * @param waiting_secs Time in seconds you want to wait
 * @return True if the calibration switch is active, otherwise false
 **/
bool wait_time_no_RTC(const uint16_t waiting_secs) {
    unsigned long period_millis = waiting_secs * 1000L;
    unsigned long prev_L_millis = millis();
    unsigned long prev_S_millis = prev_L_millis;
    uint16_t remain_time = waiting_secs;
    
    if (LCD_enabled) lcd.print_msg(0, 3, "Next read.:");

    while ((millis() - prev_L_millis) < period_millis) {
        // If the calibration button is active, we exit the wait immediately
        if (digitalRead(PH_CALIBRATION_SWITCH_PIN) == HIGH)
            return true;     // Exit because de calibration switch is active

        // Updates the remaining timeout
        if ((millis() - prev_S_millis) >= 1000) {
            if (LCD_enabled) lcd.print_msg_val(12, 3, "%ds ", (int32_t)--remain_time);
            DEBUG_NN(F("."))
            prev_S_millis = millis();
        }

        WebServer_check_petition();                        // loop to check possible webserver petitions
    }

    return false;            // Exit without active de calibration switch
}

void setup() {
    char buffer[INI_FILE_BUFFER_LEN];                                     // Temporal string for read ini file

    pinMode(PH_CALIBRATION_SWITCH_PIN, INPUT);                            // Configure the input pin for the calibration switch

    Wire.begin();                                                         // Initialize the I2C bus (BH1750 library doesn't do this automatically)

	// Initialize serial if DEBUG default value is true
	if (DEBUG) SERIAL_MON.begin(SERIAL_BAUD);
	while (DEBUG && !SERIAL_MON) { ; }                                    // wait for serial port to connect. Needed for native USB port only
    
	// Always init SD card because we need to read init configuration file.
	if (SD.begin(SD_CARD_SS_PIN)) {
		DEBUG_NL(F("Initialization SD done."))

        // Read initial config from file
        IniFile ini(SD_INI_CFG_FILENAME);                                 // IniFile configuration

        // Try to open and check config file
        if (SD_check_IniFile(&ini)) {
            ini.getValue("debug", "enabled put", buffer,                  // Load if debug mode configuration
                         INI_FILE_BUFFER_LEN, DEBUG);
            ini.getValue("LCD", "enabled",                                // Load if LCD is enabled
                         buffer, INI_FILE_BUFFER_LEN, LCD_enabled);
            ini.getValue("RTC", "enabled",                                // Load if RTC is enabled
                         buffer, INI_FILE_BUFFER_LEN, RTC_enabled);
            ini.getValue("SD_card", "save_on_sd",                         // Load if SD save is enabled
                         buffer, INI_FILE_BUFFER_LEN, SD_save_enabled);

            SD_load_culture_ID(&ini, &culture_ID);                        // Load culture identification
            SD_load_Cnn_type(&ini, cnn_option);                           // Load connection type

            if (cnn_option == it_Ethernet) {
                SD_load_Eth_config(&ini, eth_mac);
                cnn_init = ETH_initialize(&Ethernet, eth_mac);
            } 
            else if (cnn_option == it_GPRS) {
                cnn_init = MODEM_connect_network();
            }
            
            SD_load_MQTT_config(&ini, mqtt_pub, &culture_ID);             // Load MQTT connection information

			SD_load_DHT_sensors(&ini, &dht_sensors);                      // Initialize DHT sensors configuration
            SD_load_DO_sensor(&ini, &do_sensor);                          // Initialize DO sensor
			SD_load_Lux_sensors(&ini, lux_sensors);                       // Initialize Lux light sensor
            SD_load_pH_sensors(&ini, pH_sensors);                         // Initialize pH sensors
            SD_load_ORP_sensors(&ini, orp_sensors);                       // Initialize ORP sensors
            SD_load_WP_Temp_sensors(&ini, wp_t_sensors);                  // Initialize DS18B20 waterproof temperature sensors
            SD_load_Current_sensors(&ini, curr_sensors);                  // Initialize current sensors

            SD_load_WebServerActuators(&ini, web_server, os_actuators);   // Initialize WebServer & external actuators
        }
	}
	else {
		DEBUG_NL(F("Initialization SD failed!"))
	}

    // If DEBUG is active and Serial not initialized, then start this
    if (!DEBUG_DEF_ENABLED && DEBUG) SERIAL_MON.begin(SERIAL_BAUD);
        else if (DEBUG_DEF_ENABLED && !DEBUG) SERIAL_MON.end();

    // Inicialitza LCD en cas que n'hi haigui
	if (LCD_enabled) {
		DEBUG_NL(F("Initialization LCD.."))

		lcd.init();
		lcd.show_init_msg(LCD_INIT_MSG_L1, LCD_INIT_MSG_L2,
						  LCD_INIT_MSG_L3, LCD_INIT_MSG_L4,
                          LCD_INIT_TIMEOUT);
    }

	// If save in SD card option is enabled
	if (SD_save_enabled) {
		SD_get_next_FileName(fileName);                                   // Obtain the next file name to write
        SD_write_data(fileName, true, false, SD_DATA_DELIMITED);          // Write File headers
	}

	// Inicialitza RTC en cas de disposar
	if (RTC_enabled) {
		DEBUG_NN(F("Init RTC Clock.. "))
		
		// Try to initialize the RTC module
		if (dateTimeRTC.begin()) {
			DEBUG_NL(dateTimeRTC.getDateTime())                          // RTC works. Print current time
		}
		else {
            RTC_enabled = false;
			DEBUG_NL(F("No clock working"))                              // RTC does not work
		}
	}
}

void loop() {
    if (DEBUG) {
        SERIAL_MON.print(F("\nFreeMem: ")); SERIAL_MON.print(freeMemory());
        SERIAL_MON.print(F(" - loop: ")); SERIAL_MON.println(++loop_count);
		SERIAL_MON.println(F("Getting data:"));
    }

	if (LCD_enabled)
		lcd.print_msg_val(0, 3, "Getting data.. %d", (int32_t)loop_count);

    // Capture the values of all available sensors
    capture_all_sensors();
    
    // END of capturing values
    if (LCD_enabled) mostra_LCD();
    
	if (cnn_option != it_none) {
        if (DEBUG) SERIAL_MON.print(F("Sending data to server.. "));

        // Try to send the collected data to the remote broker
        if (LCD_enabled) lcd.print_msg(0, 2, "Send: ");

        if (send_data_mqtt_broker()) {
            DEBUG_NL(F("OK"))
            
            if (LCD_enabled) lcd.print_msg(6, 2, "OK   ");                 // Show last send status

        } else {
            DEBUG_NL(F("ERROR"))

            if (LCD_enabled) lcd.print_msg(6, 2, "ERROR");                 // Show last send status
        }

        if (RTC_enabled && LCD_enabled) {                                  // Update status on the screen
            dateTimeRTC.getTime(last_send);
            lcd.print_msg(12, 2, last_send);
        }
    }
    
    WebServer_check_petition();                            // loop to check possible webserver petitions

    // Save data to SD card
    if (SD_save_enabled) SD_write_data(fileName, false, true, SD_DATA_DELIMITED);

	// Waiting time until the next reading
    if (RTC_enabled)
        perf_pH_calib = wait_time_with_RTC(DELAY_SECS_NEXT_READ);
    else
        perf_pH_calib = wait_time_no_RTC(DELAY_SECS_NEXT_READ);

    // If the pH switch is active, perform the calibration iteration
    while (perf_pH_calib || digitalRead(PH_CALIBRATION_SWITCH_PIN) == HIGH) {
        DEBUG_NL(F("\n[!] Calibration switch active."))
        pH_calibration();
        
        if (perf_pH_calib) perf_pH_calib = false;
        delay(10000);
    }
}
