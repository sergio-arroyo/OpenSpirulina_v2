/*
 * Monitoring of spirulina cultures
 *
 * Autors:
 *   OpenSpirulina http://www.openspirulina.com
 *   Fran Romero   https://github.com/yatan
 *   Sergio Arroyo https://github.com/sergio-arroyo
 * 
 */
#include <Arduino.h>
#include "Configuration.h"                                 // General configuration file

// Third-party libs
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGsmClient.h>
#include <Ethernet.h>
#include <MemoryFree.h>

// OpenSpirulina libs
#include "Load_SD_Config.h"                                // Read the initial configuration file
#include "DateTime_RTC.h"                                  // Class to control RTC date time
#include "LCD_Screen.h"                                    // Class for LCD control
#include "DHT_Sensors.h"                                   // Class for control all DHT sensors
#include "DO_Sensor.h"                                     // Class for DO (Optical Density) control
#include "Lux_Sensors.h"                                   // Class for lux sensor control
#include "PH_Sensors.h"                                    // Class for pH sensors control
#include "WP_Temp_Sensors.h"                               // Class for DS18 Waterproof Temperature Sensors control
#include "Current_Sensors.h"                               // Class for current sensors control
#include "ORP_Sensors.h"                                   // Class for ORP (Oxydo Reduction Potential) sensors control


/*****************
 * GLOBAL CONTROL
 *****************/
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

#if DUMP_AT_COMMANDS == 1                                  // GPRS Modem
    #include <StreamDebugger.h>
    StreamDebugger debugger(SERIAL_AT, SERIAL_MON);
    TinyGsm modem(debugger);
#else
    TinyGsm modem(SERIAL_AT);
#endif

// Connections to send data
bool conexio_internet = false;

TinyGsmClient client(modem);                               // GSM Modem client
EthernetClient eth_client;                                 // Ethernet Network client
Internet_cnn_type cnn_option = NET_DEF_CNN_TYPE;           // None | Ethernet | GPRS Modem | Wifi <-- Why not? Dream on it


//TODO: Cambiar tipo de dato de last_send de String a char[9]
//char last_send[9];
String last_send;                                          //Last time sended data
uint16_t loop_count = 0;



/*
  ______ _    _ _   _  _____ _______ _____ ____  _   _  _____  
 |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____| 
 | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___   
 |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \  
 | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) | 
 |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/  
 */

/* Capture light ambient with LDR sensor*/
float capture_LDR(uint8_t s_pin) {
    return analogRead(s_pin);
}

/* Capture CO2 sensor */
float capture_CO2(uint8_t pin) {
    if (DEBUG) SERIAL_MON.println(F("Capturing CO2"));

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

    if (last_send != "") {                                 // Show last send time
        lcd.print_msg(0, 2, "Last: ");
        lcd.print(last_send);
    }
}

/* Capture data in calibration mode */
void pH_calibration() {
    if (DEBUG) SERIAL_MON.println(F("Starting calibration mode.."));
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

/* Write to SD the information collected from the sensors
 * Performs dump of all result values stored in the data array
 * 
 * @param _fileName The name of destination file t write the data
 * @param print_tag Indicates whether the label of each sensor should be displayed
 * @param print_value Indicates whether the value of each sensor should be displayed
 * @param delim Character that indicates the separator of the fields shown
 **/
void SD_write_data(const char* _fileName, bool print_tag, bool print_value, char delim) {
    objFile = SD.open(_fileName, FILE_WRITE);              // Try to open file

    if (!objFile) {
        if (DEBUG) SERIAL_MON.println(F("Error opening SD file!"));
        return;    //Exit
    }

    String str_out = "";                                   // Temporal data string

    // Save datetime from RTC module
    if (RTC_enabled) {
        if (print_tag) str_out.concat(F("DateTime"));
        if (print_value) str_out.concat(dateTimeRTC.getDateTime());
    }

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

    if (DEBUG) {
        SERIAL_MON.print(F("Write in file: "));
        SERIAL_MON.println(str_out);
    }
    objFile.println(str_out);                              // Write the string result to file
    objFile.close();                                       // Close the file:
}

bool send_data_ethernet(String cadena) {
    if (!conexio_internet) {
        SERIAL_MON.println(F("[Ethernet] Getting IP by DHCP.."));
        conexio_internet = Ethernet.begin((uint8_t*) ETH_MAC);

        if (!conexio_internet) {
        SERIAL_MON.println(F("[Ethernet] Conection to router Failed"));
        return false;
        }
    }
    
    eth_client.stop();
    // if there's a successful connection:
    if (eth_client.connect(HTTP_REP1_SERVER, HTTP_REP1_PORT)) {
        if (DEBUG)
        SERIAL_MON.println(F("Connecting to openspirulina..."));

        if (DEBUG)
        SERIAL_MON.println(cadena);
        
        // Send string to internet  
        eth_client.println(cadena);
        eth_client.println(F("Host: sensors.openspirulina.com"));
        eth_client.println(F("User-Agent: arduino-ethernet-1"));
        eth_client.println(F("Connection: close"));
        eth_client.println();
        return true;
    }
    else {
        // If you couldn't make a connection:
        SERIAL_MON.println(F("Connection to openspirulina Failed"));
        return false;
    }
}

bool connect_network() {
    if(DEBUG) SERIAL_MON.println(F("Waiting for network..."));

    if (!modem.waitForNetwork()) {
        SERIAL_MON.println(F("Network [fail]"));
        delay(1000);
        return false;
    }
    else {
        if(DEBUG) SERIAL_MON.println(F("Network [OK]"));  
        return true;
    }
}

bool send_data_modem(String cadena, bool step_retry) {
    // Set GSM module baud rate
    SERIAL_AT.begin(9600);

    delay(3000);
    if (DEBUG) SERIAL_MON.println(F("Initializing modem..."));
        
    if (!modem.restart()) {
        if (DEBUG) SERIAL_MON.println(F("Modem init [fail]"));
        delay(1000);
        return false;
    }

    if (DEBUG) {
        SERIAL_MON.println(F("Modem init [OK]"));    
        SERIAL_MON.print(F("Modem: "));
        SERIAL_MON.println( modem.getModemInfo() );
    }

    // Unlock your SIM card with a PIN
    //modem.simUnlock("1234");    
    if (connect_network()) {
        // Network OK
        if (DEBUG) {
            SERIAL_MON.print(F("Connecting to "));
            SERIAL_MON.println(GPRS_APN);
        }

        if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
            if (DEBUG) SERIAL_MON.println(F("GRPS [fail]"));
            delay(1000);

            if (step_retry == false) {
                if (DEBUG) SERIAL_MON.println(F("[Modem] Retrying connection !"));
                send_data_modem(cadena, true);  // Reconnect modem and init again
            }      
            return false;
        }
    
        IPAddress local = modem.localIP();
        if (DEBUG) {
            SERIAL_MON.println(F("GPRS [OK]"));
            SERIAL_MON.print(F("Local IP: "));
            SERIAL_MON.println(local);

            SERIAL_MON.print(F("Connecting to "));
            SERIAL_MON.println(HTTP_REP1_SERVER);
        }
        
        if (!client.connect(HTTP_REP1_SERVER, HTTP_REP1_PORT)) {
            SERIAL_MON.println(F("Server [fail]"));
            delay(1000);

            if (step_retry == false) {
                SERIAL_MON.println(F("[Modem] Retrying connection !"));
                send_data_modem(cadena, true);  // Reconnect modem and init again
            }      
            return false;
        }

        if (DEBUG) SERIAL_MON.println(F("Server [OK]"));

        // Make a HTTP GET request:
        client.print(cadena + " HTTP/1.0\r\n");
        client.print(String("Host: ") + HTTP_REP1_SERVER + "\r\n");
        client.print(F("Connection: close\r\n\r\n"));

        /*
        // Wait for data to arrive
        while (client.connected() && !client.available()) {
            delay(100);
            SERIAL_MON.print('.');
        };
        
        SERIAL_MON.println(F("Received data: "));
        // Skip all headers
        //client.find("\r\n\r\n");
        // Read data
        unsigned long timeout = millis();
        unsigned long bytesReceived = 0;
        while (client.connected() && millis() - timeout < 5000L) {//client.connected() &&
            //while (client.available()) {
            char c = client.read();
            SERIAL_MON.print(c);
            bytesReceived += 1;
            timeout = millis();
            //}
        }
        */
    
        client.stop();

        if (DEBUG) SERIAL_MON.println(F("\nServer disconnected"));
        
        modem.gprsDisconnect();
        if (DEBUG) SERIAL_MON.println(F("GPRS disconnected"));
        
        return true;
        }
        else {
            SERIAL_MON.println(F("[Modem] Fail !"));
            // Try one more time, if continue fails, continue
            if (step_retry == false) {
                SERIAL_MON.println(F("[Modem] Retrying connection !"));
                send_data_modem(cadena, true);  
            }
        return false;
        }
    
    return false;
}

bool send_data_server() {
	String cadena = F("GET /afegir.php?");
	
	// Append our ID Arduino
	cadena += F("idarduino=");
	cadena += HTTP_REP1_ID_ARDUINO;

	// Append PIN for that ID
	cadena += F("&pin=");
	cadena += HTTP_REP1_PIN_ARDUINO;

	// Append temperatures
    uint8_t max_val = (wp_t_sensors)?wp_t_sensors->get_n_pairs():0;
	for (int i=0; i<max_val; i++) {
		cadena += F("&temp");
		cadena += i+1;
		cadena += F("_s=");
		cadena += wp_t_sensors->get_result_pair(i, WP_Temp_Sensors::S_Surface);

		cadena += F("&temp");
		cadena += i+1;
		cadena += F("_b=");
		cadena += wp_t_sensors->get_result_pair(i, WP_Temp_Sensors::S_Background);
	}

	// Append Ambient temperatures and Humidity
	//TODO: Cambiar el volcado de datos por el de la funcion definida ya en la clase DHT_Sensor
	for (uint8_t i=0; i<dht_sensors.get_n_sensors(); i++) {
		cadena += F("&ta");
		cadena += i+1;
		cadena += "=";
		cadena += dht_sensors.get_Temperature(i);
		cadena += F("&ha");
		cadena += i+1;
		cadena += "=";
		cadena += dht_sensors.get_Humidity(i);
	}
  
	// Append Lux sensors
	if (lux_sensors) {
        lux_sensors->bulk_results(cadena, false, true, '&');
	}

	// Append pH Sensor
    uint8_t num_max = pH_sensors? pH_sensors->get_n_sensors() : 0;
	for (int i=0; i<num_max; i++) {
		cadena += "&ph";
		cadena += i+1;
		cadena += "=";
		cadena += pH_sensors->get_sensor_value(i);
	}  

	// Append DO Sensor
	if (do_sensor.is_init()) {                             // If have DO sensor
		cadena += "&pre_L=";                               // Previous lux
		cadena += do_sensor.get_preLux_value();            // Pre Lux value
		cadena += "&do1_R=";
		cadena += do_sensor.get_Red_value();               // Red value
		cadena += "&do1_G=";
		cadena += do_sensor.get_Green_value();             // Green value
		cadena += "&do1_B=";
		cadena += do_sensor.get_Blue_value();              // Blue value
		cadena += "&do1_RGB=";
		cadena += do_sensor.get_White_value();             // RGB (white) value
	}

	// Append CO2 Sensors
	for (uint8_t i=0; i<CO2_DEF_NUM_SENSORS; i++) {
		cadena += "&co2";
		cadena += i+1;
		cadena += "=";
		cadena += array_CO2[i];
	}

	// Append Current initial & end
    num_max = curr_sensors? curr_sensors->get_n_sensors() : 0;
	for (uint8_t i=0; i<num_max; i++) {
		cadena += "&curr";
		cadena += i+1;
		cadena += "=";
		cadena += curr_sensors->get_current_value(i);
	}

	if (DEBUG) {
		SERIAL_MON.print("Server petition: ");
		SERIAL_MON.println(cadena);
	}

	// Send data to specific hardware
	if (cnn_option == it_Ethernet) {
		// Add end of GET petition for Ethernet and send
		cadena += " HTTP/1.1";
		return send_data_ethernet(cadena);
	}
	else if (cnn_option == it_GPRS) {
		// Send petition to GPRS Modem
		return send_data_modem(cadena, false);
	} else {
		return false;
	}
}

/* Capture the values of all available sensors */
void capture_all_sensors() {
    if (curr_sensors) {
        if (DEBUG) SERIAL_MON.println(F("Capture current.."));
        curr_sensors->capture_all_sensors();
    }

    // Si tenim sondes de temperatura
    if (wp_t_sensors) {
		if (DEBUG) SERIAL_MON.println(F("Capture WP temperatures.."));
		wp_t_sensors->store_all_results();
	}
    
	// Capture PH for each pH Sensor
    if (pH_sensors) {
        if (DEBUG) SERIAL_MON.println(F("Capture pH sensors.. "));
        pH_sensors->capture_all_sensors();
    }

    if (orp_sensors) {
        if (DEBUG) SERIAL_MON.println(F("Capture ORP sensors.. "));
        orp_sensors->capture_all_sensors();
    }

    // Capture PH for each pH Sensor
	if (dht_sensors.get_n_sensors() > 0) {
		if (DEBUG) SERIAL_MON.println(F("Capture DHT sensors.."));
		dht_sensors.capture_all_sensors();
	}

    if (lux_sensors) {
		if (DEBUG) SERIAL_MON.println(F("Capture lux sensor.."));
		lux_sensors->capture_all_sensors();
	}

    //Capture DO values (Red, Green, Blue, and White)
    if (do_sensor.is_init()) {
		if (DEBUG) SERIAL_MON.println(F("Capture DO sensor.."));
        do_sensor.capture_DO(); 
        delay(500);
    }
    
    // Capture CO2 concentration
    if (CO2_DEF_NUM_SENSORS > 0) {
		if (DEBUG) SERIAL_MON.println(F("Capture CO2 sensor.."));
		capture_CO2(CO2_SENS_DEF_PINS[0]);
	}
}

/* Wait a certain time validating if the calibration switch is pressed
 * The time is calculated with RTC module
 * 
 * @param waiting_secs Time in seconds you want to wait
 * @return True if the calibration switch is active, otherwise false
 **/
bool wait_time_with_RTC(const uint16_t waiting_secs) {
	uint32_t time_next_loop = dateTimeRTC.inc_unixtime(waiting_secs);  // Set next timer loop for actual time + delay time

    if (DEBUG) {
        SERIAL_MON.print(F("Waiting for "));
        SERIAL_MON.print(waiting_secs);
        SERIAL_MON.println(F(" seconds"));
    }
    
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
            if (DEBUG) SERIAL_MON.print(F("."));
            prev_S_millis = millis();
        }
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
            if (DEBUG) SERIAL_MON.print(F("."));
            prev_S_millis = millis();
        }
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
		if (DEBUG) SERIAL_MON.println(F("Initialization SD done."));

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

			SD_load_DHT_sensors(&ini, &dht_sensors);                      // Initialize DHT sensors configuration
            SD_load_DO_sensor(&ini, &do_sensor);                          // Initialize DO sensor
			SD_load_Lux_sensors(&ini, lux_sensors);                       // Initialize Lux light sensor
            SD_load_pH_sensors(&ini, pH_sensors);                         // Initialize pH sensors
            SD_load_ORP_sensors(&ini, orp_sensors);                       // Initialize ORP sensors
            SD_load_WP_Temp_sensors(&ini, wp_t_sensors);                  // Initialize DS18B20 waterproof temperature sensors
            SD_load_Current_sensors(&ini, curr_sensors);                  // Initialize current sensors
            SD_load_connection_type(&ini, cnn_option);                    // Read the connection type config. to send data
        }
	}
	else {
		if (DEBUG) SERIAL_MON.println(F("Initialization SD failed!"));
	}

    // If DEBUG is active and Serial not initialized, then start this
    if (!DEBUG_DEF_ENABLED && DEBUG) SERIAL_MON.begin(SERIAL_BAUD);
        else if (DEBUG_DEF_ENABLED && !DEBUG) SERIAL_MON.end();

    // Inicialitza LCD en cas que n'hi haigui
	if (LCD_enabled) {
		if (DEBUG) SERIAL_MON.println(F("Initialization LCD.."));

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
		if (DEBUG) SERIAL_MON.print(F("Init RTC Clock.. "));
		
		// Try to initialize the RTC module
		if (dateTimeRTC.begin()) {
			if (DEBUG) SERIAL_MON.println(dateTimeRTC.getDateTime());     // RTC works. Print current time
		}
		else {
            RTC_enabled = false;
			if (DEBUG) SERIAL_MON.println(F("No clock working"));         // RTC does not work
		}
	}

	// Initialize Ethernet shield
	if (cnn_option == it_Ethernet) {
		// give the ethernet module time to boot up:
		delay(2000);
		if (DEBUG) SERIAL_MON.println(F("Starting Ethernet Module"));

		// Start the Ethernet connection using a fixed IP address and DNS server:
		// Ethernet.begin(eth_mac, ip, myDns, gateway, subnet);
		// DHCP IP ( For automatic IP )
		conexio_internet = Ethernet.begin((uint8_t*) ETH_MAC);

		if (!conexio_internet) SERIAL_MON.println(F("[Ethernet] Fail obtain IP"));
		
		// Print the Ethernet board/shield's IP address:
		if (DEBUG) {
			SERIAL_MON.print(F("My IP address: "));
			SERIAL_MON.println(Ethernet.localIP());
		}
	}
	// Initialize GPRS Modem
	else if(cnn_option == it_GPRS) {
		// Not implemented jet
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
		// Si s'envia correctament actualitzar last_send
		if  (send_data_server()) {
			delay(200);
			last_send = dateTimeRTC.getTime();
            delay(100);
            last_send += F(" OK");
            if (LCD_enabled) mostra_LCD();
        }
        else {
            delay(200);
            last_send = dateTimeRTC.getTime();
            delay(100);
            last_send += F(" FAIL");

            if (LCD_enabled) mostra_LCD();
        }
    }

    // Save data to SD card
    if (SD_save_enabled) SD_write_data(fileName, false, true, SD_DATA_DELIMITED);

	// Waiting time until the next reading
    if (RTC_enabled)
        perf_pH_calib = wait_time_with_RTC(DELAY_SECS_NEXT_READ);
    else
        perf_pH_calib = wait_time_no_RTC(DELAY_SECS_NEXT_READ);

    // If the pH switch is active, perform the calibration iteration
    while (perf_pH_calib || digitalRead(PH_CALIBRATION_SWITCH_PIN) == HIGH) {
        if (DEBUG) SERIAL_MON.println(F("\n[!] Calibration switch active."));
        pH_calibration();
        
        if (perf_pH_calib) perf_pH_calib = false;
        delay(10000);
    }
}
