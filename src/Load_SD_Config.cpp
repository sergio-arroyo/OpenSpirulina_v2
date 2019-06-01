/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Read the initial configuration file to activate or deactivate the different modules.
 * 
 */

#include "Load_SD_Config.h"

extern bool DEBUG;


/* Open Ini config file and check if it's in correct format */
bool SD_check_IniFile(IniFile *ini) {
    char buffer[INI_FILE_BUFFER_LEN];

    if (!ini->open()) {
        if (DEBUG) {
            SERIAL_MON.print(F("Ini file "));
            SERIAL_MON.print(ini->getFilename());
            SERIAL_MON.println(F(" does not exist"));
        }        
        return false;
    }

    if (!ini->validate(buffer, sizeof(buffer))) {          // Check the file is valid
        if (DEBUG) {
            SERIAL_MON.print(F("Ini file "));
            SERIAL_MON.print(ini->getFilename());
            SERIAL_MON.print(F(" not valid: "));
        }
        return false;
    }

    return true;
}

void SD_load_culture_ID(IniFile *ini, Culture_ID_st *culture_id) {
    char buffer[INI_FILE_BUFFER_LEN] = "";
    const char *section = "culture";

    if (DEBUG) SERIAL_MON.println(F("Loading culture ID.."));

    if (ini->getValue(section, "country", buffer, sizeof(buffer)))
        strncpy(culture_id->country, buffer, 3);
    
    if (ini->getValue(section, "city", buffer, sizeof(buffer)))
        strncpy(culture_id->city, buffer, 3);

    if (ini->getValue(section, "culture", buffer, sizeof(buffer)))
        strncpy(culture_id->culture, buffer, 6);

    if (ini->getValue(section, "host_id", buffer, sizeof(buffer)))
        strncpy(culture_id->host_id, buffer, 10);

    if (DEBUG) {
        Serial.print(F("  > country: ")); Serial.println(culture_id->country);
        Serial.print(F("  > city   : ")); Serial.println(culture_id->city);
        Serial.print(F("  > culture: ")); Serial.println(culture_id->culture);
        Serial.print(F("  > host_id: ")); Serial.println(culture_id->host_id);
    }
}

void SD_load_MQTT_config(IniFile *ini, MQTT_Pub *&mqtt_pub, Culture_ID_st *culture_id) {
    char buffer[INI_FILE_BUFFER_LEN] = "";
    const char *section = "rpt:MQTT";
    MQTT_Cnn_st mqtt_info = {                              // MQTT broker connection information
        MQTT_SERVER_DEST,
        MQTT_PORT_DEST,
        MQTT_BROKER_USR,
        MQTT_BROKER_PSW
    };
    
    if (DEBUG) SERIAL_MON.println(F("Loading MQTT conn. info.."));

    if (ini->getValue(section, "server", buffer, sizeof(buffer)))
        strncpy(mqtt_info.server, buffer, SERVER_MAX_NAME_SIZE);
    
    ini->getValue(section, "port", buffer, sizeof(buffer), mqtt_info.port);
    
    if (ini->getValue(section, "usr", buffer, sizeof(buffer)))
        strncpy(mqtt_info.usr, buffer, 20);

    if (ini->getValue(section, "psw", buffer, sizeof(buffer)))
        strncpy(mqtt_info.psw, buffer, 20);
    
    // Instanciate MQTT publisher
    mqtt_pub = new MQTT_Pub(&mqtt_info, culture_id);
}

void SD_load_Cnn_type(IniFile *ini, Internet_cnn_type &option) {
   	char buffer[INI_FILE_BUFFER_LEN] = "";
    
    if (DEBUG) SERIAL_MON.println(F("Loading connection type.."));

    // Read the connection type
    if (ini->getValue("net", "cnn_type", buffer, sizeof(buffer))) {
        // Ethernet cnn type
        if (strcmp(buffer, "eth") == 0) {
            option = it_Ethernet;
            if (DEBUG) Serial.println(F("  > Ethernet"));
        }
        // GPRS cnn type
        else if (strcmp(buffer, "grps") == 0) {
            option = it_GPRS;
            if (DEBUG) Serial.println(F("  > GPRS"));
        }
        // WiFi cnn type
        else if (strcmp(buffer, "wifi") == 0) {
            option = it_Wifi;
            if (DEBUG) Serial.println(F("  > WiFi"));
        }
        // Undefined cnn type => no connection
        else {
            option = it_none;
            if (DEBUG) Serial.println(F("  > (No selection)"));
        }
    }
    
    // If the cnn_type option is not defined, the default option is maintained
}

void SD_load_Eth_config(IniFile *ini, uint8_t *mac) {
   	char buffer[INI_FILE_BUFFER_LEN] = "";
    
    if (DEBUG) SERIAL_MON.println(F("Loading Ethernet config.."));

    if (ini->getMACAddress("net", "eth_mac", buffer, sizeof(buffer), mac)) {
        if (DEBUG) {
            SERIAL_MON.print(F("  > MAC addr found = "));
            SERIAL_MON.println(buffer);
        }
    } else {
        memcpy(mac, ETH_MAC, 6);
        if (DEBUG) {
            SERIAL_MON.print(F("  > MAC addr found = "));
            print_mac_address(mac);
        }
    }
}

void SD_load_DHT_sensors(IniFile *ini, DHT_Sensors* sensors) {
	char buffer[INI_FILE_BUFFER_LEN] = "";
	char tag_sensor[14] = "";
	bool found;
	uint8_t i = 1;

	if (DEBUG) SERIAL_MON.println(F("Loading DHT sensors config.."));
	do {
		sprintf(tag_sensor, "sensor%d.pin", i++);
		found = ini->getValue("sensors:DHT", tag_sensor, buffer, sizeof(buffer));
		if (found) {
			uint8_t pin = atoi(buffer);
			if (DEBUG) {
                SERIAL_MON.print(F("  > Found config: ")); SERIAL_MON.print(tag_sensor);
			    SERIAL_MON.print(F(". Pin = ")); SERIAL_MON.println(pin);
            }
			sensors->add_sensor(pin);
		}
	} while (found);

	// If no configuration found in IniFile..
	if (sensors->get_n_sensors() == 0) {
		if (DEBUG) SERIAL_MON.println(F("No DHT config. found. Loading default.."));
		for (i=0; i<DHT_DEF_NUM_SENSORS; i++) {
			if (DEBUG)  {
                SERIAL_MON.print(F("  > Found config: sensor")); Serial.print(i+1);
			    SERIAL_MON.print(F(". Pin = ")); Serial.println(DHT_DEF_SENSORS[i]);
            }
            sensors->add_sensor(DHT_DEF_SENSORS[i]);
		}
	}
}

void SD_load_DO_sensor(IniFile *ini, DO_Sensor* sensor) {
   	char buffer[INI_FILE_BUFFER_LEN] = "";
    
    if (DEBUG) SERIAL_MON.println(F("Loading DO sensor config.."));

    // Read DO sensor address (hexadecimal format)
    if (ini->getValue("sensor:DO", "address", buffer, sizeof(buffer))) {
        uint8_t address, led_R_pin, led_G_pin, led_B_pin;

        address = (uint8_t)strtol(buffer, NULL, 16);    // Convert hex char[] to byte value
		if (DEBUG) { SERIAL_MON.print(F("  > Found config address: 0x")); SERIAL_MON.println(address, HEX); }

        // Read red LED pin
        if (!ini->getValue("sensor:DO", "led_R_pin", buffer, sizeof(buffer), led_R_pin))
            led_R_pin = DO_SENS_R_LED_PIN;

        // Read green LED pin
        if (!ini->getValue("sensor:DO", "led_G_pin", buffer, sizeof(buffer), led_G_pin))
            led_G_pin = DO_SENS_G_LED_PIN;
        
        // Read blue LED pin
        if (!ini->getValue("sensor:DO", "led_B_pin", buffer, sizeof(buffer), led_B_pin))
            led_B_pin = DO_SENS_B_LED_PIN;

        // Configure DO sensor with load parametern from IniFile
        sensor->begin(address, led_R_pin, led_G_pin, led_B_pin);
    }
    else if (DO_SENS_ACTIVE) {
        // Configure DO sensor with default configuration
        if (DEBUG) SERIAL_MON.print(F("No config found. Loading default.."));
        sensor->begin(DO_SENS_ADDR, DO_SENS_R_LED_PIN, DO_SENS_G_LED_PIN, DO_SENS_B_LED_PIN);
    }
}

void SD_load_pH_sensors(IniFile *ini, PH_Sensors *&sensors) {
	char buffer[INI_FILE_BUFFER_LEN] = "";
	char tag_sensor[14] = "";
	bool found;
	uint8_t i = 1;

	if (DEBUG) SERIAL_MON.println(F("Loading pH sensors config.."));
	do {
		sprintf(tag_sensor, "sensor%d.pin", i++);
		found = ini->getValue("sensors:pH", tag_sensor, buffer, sizeof(buffer));
		if (found) {
			uint8_t pin = atoi(buffer);
			Serial.print(F("  > Found config: ")); Serial.print(tag_sensor);
			Serial.print(F(". Pin = ")); Serial.println(pin);

            if (!sensors) sensors = new PH_Sensors();
            sensors->add_sensor(pin);
		}
	} while (found);

	// If no configuration found in IniFile..
	if (!sensors && PH_DEF_NUM_SENSORS > 0) {
		if (DEBUG) SERIAL_MON.println(F("No pH config. found. Loading default.."));
		for (i=0; i<PH_DEF_NUM_SENSORS; i++) {
            if (DEBUG) {
                SERIAL_MON.print(F("  > Found config: sensor")); SERIAL_MON.print(i+1);
			    SERIAL_MON.print(F(". Pin = ")); SERIAL_MON.println(PH_DEF_PIN_SENSORS[i]);
            }
            sensors->add_sensor(PH_DEF_PIN_SENSORS[i]);
		}
	}
}

bool extract_str_params_Lux_sensor(char *str, Lux_Sensors::Lux_Sensor_model_t &model, uint8_t &addr, uint8_t &addr_pin) {
    char *pch;

    pch = strtok(str, ",");                                // Get the first piece, sensor model
    if (pch == NULL) return false;                         // Check piece, if it's not correct, exit

    while (isspace(*pch)) ++pch;                           // Skip possible white spaces
    char *tmp = pch;                                       // Remove trailing white space
    while (*tmp != ',' && *tmp != '\0')
        if (*tmp == ' '|| *tmp == '\t') *tmp++ = '\0'; else tmp++;

    if (strcmp(pch, "BH1750") == 0)                        // Select sensor model
        model = Lux_Sensors::Lux_Sensor_model_t::mod_BH1750;
    else if (strcmp(pch, "MAX44009") == 0)
        model = Lux_Sensors::Lux_Sensor_model_t::mod_MAX44009;
    else
        return false;
    
    pch = strtok(NULL, ",");                               // Get the second piece, address
    addr = (uint8_t)strtol(pch, NULL, 16);                 // Obtain the pin value

    pch = strtok(NULL, ",");                               // Get the third piece, pin ADDR
    addr_pin = (uint16_t)strtol(pch, NULL, 10);            // Obtain the pin value

    return true;
}

void SD_load_Lux_sensors(IniFile *ini, Lux_Sensors *&sensors) {
	char buffer[INI_FILE_BUFFER_LEN] = "";
	char tag_sensor[11] = "";
    bool sens_cfg;
    uint8_t i = 1;
    uint8_t s_addr, s_addr_pin;
    Lux_Sensors::Lux_Sensor_model_t s_model;

    if (DEBUG) SERIAL_MON.println(F("Loading Lux sensors config.."));
    do {
		sprintf(tag_sensor, "sensor%d", i++);
		sens_cfg = ini->getValue("sensors:lux", tag_sensor, buffer, sizeof(buffer));

		if (sens_cfg && extract_str_params_Lux_sensor(buffer, s_model, s_addr, s_addr_pin)) {
			if (DEBUG) {
                SERIAL_MON.print(F("  > Found config: ")); Serial.print(tag_sensor);
			    SERIAL_MON.print(F(". Addr = 0x")); Serial.print(s_addr, 16);
                SERIAL_MON.print(F(", model = "));
                switch (s_model) {
                    case Lux_Sensors::Lux_Sensor_model_t::mod_BH1750:
                        SERIAL_MON.println(F("BH1750"));
                        break;
                    case Lux_Sensors::Lux_Sensor_model_t::mod_MAX44009:
                        SERIAL_MON.println(F("MAX44009"));
                        break;
                    default: SERIAL_MON.println(F("Undefined"));
                }
            }

            if (!sensors) sensors = new Lux_Sensors();       //If the object has not been initialized yet, we do it now
            sensors->add_sensor(s_model, s_addr, s_addr_pin);
        }
    } while (sens_cfg);

    // Load default configuration
    if (!sensors && LUX_SENS_DEF_NUM > 0) {
        if (DEBUG) SERIAL_MON.println(F("No lux config found. Loading default.."));
        sensors = new Lux_Sensors();

        for (uint8_t i=0; i<LUX_SENS_DEF_NUM; i++) {
            sensors->add_sensor((Lux_Sensors::Lux_Sensor_model_t) LUX_SENS_DEF_MODELS[i],
                                    LUX_SENS_DEF_ADDRESS[i],
                                    LUX_SENS_DEF_ADDR_PIN[i]);
            if (DEBUG) {
                SERIAL_MON.print(F("  > Found default config: "));
			    SERIAL_MON.print(F("model: ")); Serial.println(LUX_SENS_DEF_MODELS[i]);
                SERIAL_MON.print(F(", addr: 0x")); Serial.println(LUX_SENS_DEF_ADDRESS[i], 16);
            }
        }
    }
}

void SD_load_ORP_sensors(IniFile *ini, ORP_Sensors *&sensors) {
	char buffer[INI_FILE_BUFFER_LEN] = "";
	char tag_sensor[15] = "";
	bool found;
	uint8_t i = 1;
    
	if (DEBUG) SERIAL_MON.println(F("Loading ORP sensors config.."));
	do {
		sprintf(tag_sensor, "sensor%d.addr", i++);
		found = ini->getValue("sensors:ORP", tag_sensor, buffer, sizeof(buffer));
		if (found) {
			uint8_t addr = (uint8_t)strtol(buffer, NULL, 16);    // Convert hex char[] to byte value
			Serial.print(F("  > Found config: ")); Serial.print(tag_sensor);
			Serial.print(F(". Addr = 0x")); Serial.println(addr, HEX);

            if (!sensors) sensors = new ORP_Sensors();     // If the object has not been initialized yet, we do it now
            sensors->add_sensor(addr);
		}
	} while (found);

	// If no configuration found in IniFile..
	if (!sensors && ORP_DEF_NUM_SENSORS > 0) {
		if (DEBUG) SERIAL_MON.println(F("No ORP config. found. Loading default.."));
		for (i=0; i<ORP_DEF_NUM_SENSORS; i++) {
            if (DEBUG) {
                SERIAL_MON.print(F("  > Found config: sensor")); SERIAL_MON.print(i+1);
			    SERIAL_MON.print(F(". Addr = 0x")); SERIAL_MON.println(ORP_DEF_ADDRS[i], HEX);
            }
            sensors->add_sensor(ORP_DEF_ADDRS[i]);
		}
	}
}

void SD_load_WP_Temp_sensors(IniFile *ini, WP_Temp_Sensors *&sensors) {
	char buffer[INI_FILE_BUFFER_LEN] = "";
	char tag_sensor[20] = "";
    uint8_t i=1, addr_s[8], addr_b[8];
	uint16_t one_wire_pin;

    if (DEBUG) SERIAL_MON.println(F("Loading WP temperature sensors config.."));
    
    bool found = ini->getValue("sensors:wp_temp", "one_wire_pin",
                                buffer, sizeof(buffer), one_wire_pin); // Load One Wire config

	while (found) {
        // Load surface sensor address
		sprintf(tag_sensor, "addr_t%d_s", i);
		found = ini->getValue("sensors:wp_temp", tag_sensor, buffer, sizeof(buffer));
        if (!found || !convert_str_to_addr(buffer, addr_s, 8)) break; // If can't find the sensor or the address is not correct, exit

        // Load background sensor address
        sprintf(tag_sensor, "addr_t%d_b", i);
        found = ini->getValue("sensors:wp_temp", tag_sensor, buffer, sizeof(buffer));
        if (!found || !convert_str_to_addr(buffer, addr_b, 8)) break; // If can't find the sensor or the address is not correct, exit

        if (DEBUG) {
            SERIAL_MON.print(F("  > Found config: sensor")); SERIAL_MON.print(i);
            SERIAL_MON.println(F(" pair"));
        }
        
        if (!sensors) sensors = new WP_Temp_Sensors(one_wire_pin);    // If the obj has not been initialized yet, we do it now
        sensors->add_sensors_pair(addr_s, addr_b);

        i++;
	}

    // Load default configuration
    if (!sensors && WP_T_DEF_NUM_PAIRS > 0) {
        if (DEBUG) SERIAL_MON.println(F("No WP config found. Loading default.."));
        
        sensors = new WP_Temp_Sensors(WP_T_ONE_WIRE_PIN);
        for (i=0; i<WP_T_DEF_NUM_PAIRS; i++) {
            if (DEBUG) {
                SERIAL_MON.print(F("  > Found config: sensor")); SERIAL_MON.print(i+1);
                SERIAL_MON.println(F(" pair"));
            }
            sensors->add_sensors_pair(WP_T_DEF_SENST_PAIRS[i][0], WP_T_DEF_SENST_PAIRS[i][1]);   
        }
    }
}

bool extract_str_params_Current_sensor(char *str, uint8_t &pin, Current_Sensors::Current_Model_t &model, uint16_t &var) {
    char *pch;

    pch = strtok(str, ",");                                // Get the first piece
    if (pch == NULL) return false;                         // Check piece, if it's not correct, exit
    pin = (uint8_t)strtol(pch, NULL, 10);                  // Obtain the pin value
    
    pch = strtok(NULL, ",");                               // Get the second piece

    while (isspace(*pch)) ++pch;                           // Skip possible white spaces
    char *tmp = pch;                                       // Remove trailing white space
    while (*tmp != ',' && *tmp != '\0')
        if (*tmp == ' '|| *tmp == '\t') *tmp++ = '\0'; else tmp++;

    if (strcmp(pch, "ACS712") == 0)                        // Select sensor model
        model = Current_Sensors::Current_Model_t::ACS712;
    else if (strcmp(pch, "SCT013") == 0)
        model = Current_Sensors::Current_Model_t::SCT013;
    else
        return false;
    
    pch = strtok(NULL, ",");                               // Get the third piece
    if (pch == NULL) return false;
    var = (uint16_t)strtol(pch, NULL, 10);                 // Obtain the variation value

    return true;
}

void SD_load_Current_sensors(IniFile *ini, Current_Sensors *&sensors) {
	char buffer[INI_FILE_BUFFER_LEN] = "";
	char tag_sensor[11] = "";
    bool sens_cfg;
    uint8_t i = 1;
    uint8_t pin;
    Current_Sensors::Current_Model_t s_model;
    uint16_t var;

    if (DEBUG) SERIAL_MON.println(F("Loading Current sensors config.."));
    do {
		sprintf(tag_sensor, "sensor%d", i++);
		sens_cfg = ini->getValue("sensors:current", tag_sensor, buffer, sizeof(buffer));

		if (sens_cfg && extract_str_params_Current_sensor(buffer, pin, s_model, var)) {
			if (DEBUG) {
                SERIAL_MON.print(F("  > Found config: ")); Serial.print(tag_sensor);
			    SERIAL_MON.print(F(". Pin = ")); Serial.println(pin);
            }

            if (!sensors) sensors = new Current_Sensors();      //If the object has not been initialized yet, we do it now
            sensors->add_sensor(pin, s_model, var);
        }
    } while (sens_cfg);

    // Load default configuration
    if (!sensors && CURR_SENS_DEF_NUM > 0) {
        if (DEBUG) SERIAL_MON.println(F("No current config found. Loading default.."));
        sensors = new Current_Sensors();

        for (uint8_t i=0; i<CURR_SENS_DEF_NUM; i++) {
            sensors->add_sensor(CURR_SENS_DEF_PINS[i],
                                (Current_Sensors::Current_Model_t) CURR_SENS_DEF_MODELS[i],
                                CURR_SENS_DEF_VAR[i]);
            if (DEBUG) {
                SERIAL_MON.print(F("  > Found config: "));
			    SERIAL_MON.print(F(". Pin = ")); Serial.println(CURR_SENS_DEF_PINS[i]);
            }
        }
    }
}

bool extract_params_Actuator(char *str, uint8_t &dev_pin, char *dev_id, uint8_t &ini_val) {
    char *pch;

    pch = strtok(str, ",");                                // Get the first piece
    if (pch == NULL) return false;                         // Check piece, if it's not correct, exit
    dev_pin = (uint8_t)strtol(pch, NULL, 10);              // Obtain the pin value
    
    pch = strtok(NULL, ",");                               // Get the second piece

    while (isspace(*pch)) ++pch;                           // Skip possible white spaces
    char *tmp = pch;                                       // Remove trailing white space
    while (*tmp != ',' && *tmp != '\0')
        if (*tmp == ' '|| *tmp == '\t') *tmp++ = '\0'; else tmp++;

    strncpy(dev_id, pch, ACT_MAX_DEV_ID_LEN);              // Copy the actuator ID

    pch = strtok(NULL, ",");                               // Get the third piece
    if (pch == NULL) return false;

    // Read HIGH value
    if (strcmp(pch, "HIGH") == 0) {                        // Check the initial value
        ini_val = HIGH;
    }
    // Otherwise, default value is LOW
    else {
        ini_val = LOW;
    }

    return true;
}

void SD_load_WebServerActuators(IniFile *ini, EthernetServer *&web_server, OS_Actuators *&actuators) {
	char buffer[INI_FILE_BUFFER_LEN] = "";
    const char *section = "actuators";
    

    if (DEBUG) SERIAL_MON.println(F("Loading WebServer & actuators config.."));
    
    // Load WebServer configurarion
    uint16_t srv_port = ACT_WEB_SRV_DEF_PORT;
    if (!ini->getValue(section, "srv_port", buffer, sizeof(buffer), srv_port)) {
        if (DEBUG)
            SERIAL_MON.println(F("  > WebServer config. not found. Charge default.."));
    }

    // Start WebServer
    if (DEBUG) {
        SERIAL_MON.print(F("  > WebServer start at port ")); SERIAL_MON.println(srv_port);
    }
    web_server = new EthernetServer(srv_port);
    
    // Load actuators configuration
	char act_n[10] = "";
    uint8_t dev_pin;
    char dev_id[ACT_MAX_DEV_ID_LEN+1] = "";
    uint8_t ini_val;
    bool found;
    uint8_t i = 1;

    do {
		sprintf(act_n, "act%d", i++);
		found = ini->getValue(section, act_n, buffer, sizeof(buffer));

		if (found && extract_params_Actuator(buffer, dev_pin, dev_id, ini_val)) {
			if (DEBUG) {
                SERIAL_MON.print(F("  > Found config: ")); SERIAL_MON.print(act_n);
			    SERIAL_MON.print(F(". Pin = ")); SERIAL_MON.print(dev_pin);
                SERIAL_MON.print(F(", Act. ID = "));
                SERIAL_MON.println(dev_id);
            }

            if (!actuators) actuators = new OS_Actuators();      //If the object has not been initialized yet, we do it now
            actuators->add_device(dev_id, dev_pin, ini_val);
        }
    } while (found);

    // Load default configuration
    if (!actuators && ACT_DEV_DEF_NUM > 0) {
        if (DEBUG) SERIAL_MON.println(F("No actuators config found. Loading default.."));
        actuators = new OS_Actuators();

        for (uint8_t i=0; i<ACT_DEV_DEF_NUM; i++) {
            actuators->add_device(ACT_DEF_IDS[i],
                                  ACT_DEF_PINS[i],
                                  ACT_DEF_INI_VAL[i]);
            if (DEBUG) {
                SERIAL_MON.print(F(". Pin = ")); SERIAL_MON.print(ACT_DEF_PINS[i]);
                SERIAL_MON.print(F(", Act. ID = "));
                SERIAL_MON.println(ACT_DEF_IDS[i]);
            }
        }
    }
}
