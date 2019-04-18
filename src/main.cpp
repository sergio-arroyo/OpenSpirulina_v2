/*
 * Send Sensors Data To HTTP (PHP) Server
 *
 * Autor: Fran Romero https://github.com/yatan
 *        OpenSpirulina http://www.openspirulina.com
 *
 * Based on:
 * 
 * Ethernet web client sketch:
 *   Repeating Web client
 *   http://www.arduino.cc/en/Tutorial/WebClientRepeating
 *
 * DS18B20 sensor de temperatura para líquidos con Arduino:
 *   https://programarfacil.com/blog/arduino-blog/ds18b20-sensor-temperatura-arduino/
 *
 * pH
 *   phMeterSample.ino from: YouYou
 *
 * BH1750: lux sensor to mesure spirulina's biomass concentration.
 *   https://github.com/claws/BH1750
 * 
 * LCD LiquidCrystal_I2C LiquidCrystal Arduino library for the DFRobot I2C LCD displays:
 *   https://github.com/marcoschwartz/LiquidCrystal_I2C
 * 
 * GSM/GPRS A6 modem library:
 *   https://github.com/vshymanskyy/TinyGSM
 * 
 */
#include <Arduino.h>
#include "Configuration.h"

// Third-party libs
#include <SPI.h>
#include <SD.h>
#include <DallasTemperature.h>
#include <TinyGsmClient.h>
#include "MemoryFree.h"                                    //TODO: Gestió de memòria             
#include <Ethernet2.h>                                     // Shield W5500 with Ethernet2.h will be used in this version of the project

// OpenSpirulina libs
#include "Load_SD_Config.h"
#include "DateTime_RTC.h"                                  // Class to control RTC date time
#include "LCD_Screen.h"                                    // Class for LCD control
#include "DHT_Sensor.h"                                    // Class for control all DHT sensors
#include "DO_Sensor.h"                                     // Class for Optical Density control
#include "Lux_Sensor.h"                                    // Class for lux sensor control


/*****************
 * GLOBAL CONTROL
 *****************/
bool DEBUG = DEBUG_DEF_ENABLED;                            // Indicates whether the debug mode on serial monitor is active
bool LCD_enabled = LCD_DEF_ENABLED;                        // Indicates whether the LCD is active
bool RTC_enabled = RTC_DEF_ENABLED;                        // Indicates whether the RTC is active
bool SD_save_enabled = SD_SAVE_DEF_ENABLED;                // Indicates whether the save to SD is enabled

DHT_Sensor dht_sensors;                                    // Handle all DHT sensors
Lux_Sensor lux_sensor;                                     // Lux sensor with BH1750
DO_Sensor  do_sensor;                                      // Sensor 1 [DO] ("Optical Density")

File myFile;
char fileName[SD_MAX_FILENAME_SIZE] = "";                  // Name of file to save data readed from sensors

DateTime_RTC dateTimeRTC;                                  // RTC class object (DS3231 clock sensor)

LCD_Screen lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS,
                LCD_CONTRAST, LCD_BACKLIGHT_ENABLED);      // LCD screen






/*****
AUTHENTICATION ARDUINO
*****/
//Define de identity of Arduino
const uint16_t id_arduino  = 21;
const uint16_t pin_arduino = 12345;

/****
NETWORK SETTINGS 
****/
// Server connect for sending data
const char  server[] = "sensors.openspirulina.com";
const uint16_t  port = 80;
// assign a MAC address for the ethernet controller:
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "internet";
const char user[] = "";
const char pass[] = "";

bool conexio_internet = false;

/*
  _   _ _    _ __  __ ____  ______ _____     _____ ______ _   _  _____  ____  _____   _____
 | \ | | |  | |  \/  |  _ \|  ____|  __ \   / ____|  ____| \ | |/ ____|/ __ \|  __ \ / ____|
 |  \| | |  | | \  / | |_) | |__  | |__) | | (___ | |__  |  \| | (___ | |  | | |__) | (___
 | . ` | |  | | |\/| |  _ <|  __| |  _  /   \___ \|  __| | . ` |\___ \| |  | |  _  / \___ \
 | |\  | |__| | |  | | |_) | |____| | \ \   ____) | |____| |\  |____) | |__| | | \ \ ____) |
 |_| \_|\____/|_|  |_|____/|______|_|  \_\ |_____/|______|_| \_|_____/ \____/|_|  \_\_____/
 */
const uint8_t num_T = 4;                                   // Temperature of the culture. Sensor DS18B20.MAX 6
                                                           // T1_s T1_b -- T2_s T2_b = 4

const uint8_t num_PIR = 0;                                 // PIR movement sensor. MAX 3 --> S'ha de traure i enlloc seu posar Current pin 
const uint8_t num_DO = 1;                                  // Optical Density Sensor Module made by OpenSpirulina includes a RGB led + BH1750 lux sensor
const uint8_t num_pH = 1;                                  // pH sensor. MAX 3
const uint8_t num_CO2 = 0;                                 // CO2 sensor MAX ?

enum option_internet_type {                                // Valid internet types
	it_none = 0,
	it_Ethernet,
	it_GPRS,
	it_Wifi
};

enum option_current_sensor {                               // No sensor | ACS 712:Sensor inside | SCT013: no invasive
	cs_none = 0,
	cs_ACS712,                                             // Invasive sensor. Source: https://naylampmechatronics.com/blog/48_tutorial-sensor-de-corriente-acs712.html
	cs_SCT013                                              // Non invasive sensor with internal burden resistence. http://www.gonzalogalvan.es/medidor-de-consumo-lectura-de-la-corriente-con-arduino/ 
};

const uint8_t num_current_sensor = 1;                      // Current sensor. MAX 3 
const option_current_sensor current_sensors_type = cs_none;// Define de type of sensors

//Choose the current sensibility of current sensor:
//float current_sensibility = 0.185;                       // ACS712 de  5 Amperes
float current_sensibility = 0.100;                         // ACS712 de 20 Amperes
//float current_sensibility = 0.066;                       // ACS712 de 30 Amperes
//const int regulation = 29;                               // SCT013 de 15 Amperes con resistencia interna.
//const int regulation = 10;                               // SCT013 de 30 Amperes con resistencia interna ATENTION CONFIRM VALUE: 


const option_internet_type option_internet = it_none;      // None | Ethernet | GPRS Modem | Wifi <-- Why not? Dream on it




/*
  _____ _____ _   _  _____
 |  __ \_   _| \ | |/ ____|
 | |__) || | |  \| | (___
 |  ___/ | | | . ` |\___ \
 | |    _| |_| |\  |____) |
 |_|   |_____|_| \_|_____/
*/

/*   ANALOG PINS  */
const uint8_t pins_ph[num_pH] = {8};                       // pH Pins (Analog)
const uint8_t pins_co2[1] = {11};                          // CO2 pin (Analog)//const int pins_co2[num_CO2] = {11};     // CO2 pin (Analog)

const uint8_t pins_pir[num_PIR] = {};                      // PIR Pins  //S'ha de traure.
const uint8_t pins_current[num_current_sensor] = {38};     // Current sensor PINs, next: 39,40



/*
   _____ _      ____  ____          _       __      __     _____   _____
  / ____| |    / __ \|  _ \   /\   | |      \ \    / /\   |  __ \ / ____|
 | |  __| |   | |  | | |_) | /  \  | |       \ \  / /  \  | |__) | (___
 | | |_ | |   | |  | |  _ < / /\ \ | |        \ \/ / /\ \ |  _  / \___ \
 | |__| | |___| |__| | |_) / ____ \| |____     \  / ____ \| | \ \ ____) |
  \_____|______\____/|____/_/    \_\______|     \/_/    \_\_|  \_\_____/
*/









const uint8_t co2_samples_number = 15;

OneWire oneWireObj(ONE_WIRE_PIN);
DallasTemperature sensorDS18B20(&oneWireObj);

// Array de temperatures amb tamany num_temp sensors assignats
float array_temps[num_T];

// Array of pH sensors
float array_ph[num_pH];

// Array of PIR sensors
int array_pir[num_PIR];

// Array of Current sensors
float array_current_ini[num_current_sensor];
float array_current_end[num_current_sensor];

//Time waiting for correct agitation of culture for correct mesuring DO
const unsigned long time_current = 30L * 1000L; //Time un seconds between current ini current end measure.

// Array of CO2 sensors
float array_co2[num_CO2];


//Define Temperature sensors adress: 
//Define pair of Temp1 sensors
DeviceAddress sensor_t1_b = {0x28, 0xFF, 0x72, 0x88, 0x24, 0x17, 0x03, 0x09};
DeviceAddress sensor_t1_s = {0x28, 0xFF, 0x1B, 0xD2, 0x24, 0x17, 0x03, 0x28};
// Define pair of Temp2 sensors
DeviceAddress sensor_t2_b = {0x28, 0xFF, 0xCA, 0xE5, 0x80, 0x14, 0x02, 0x16};
DeviceAddress sensor_t2_s = {0x28, 0xFF, 0x89, 0xBB, 0x60, 0x17, 0x05, 0x6D};

DeviceAddress* array_tSensor_addrs[num_T];                 //Array of Temperatures from the culture

// Lux ambient value
float lux;
float pre_lux;

String last_send;                                          //Last time sended data
uint16_t loop_count = 0;           //TODO: Contador de ciclos de lectura


// GPRS Modem
#if DUMP_AT_COMMANDS == 1
    #include <StreamDebugger.h>
    StreamDebugger debugger(SERIAL_AT, SERIAL_MON);
    TinyGsm modem(debugger);
#else
    TinyGsm modem(SERIAL_AT);
#endif

// GSM Modem client
TinyGsmClient client(modem);

// Ethernet Network client
EthernetClient eth_client;




/*
  ______ _    _ _   _  _____ _______ _____ ____  _   _  _____  
 |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____| 
 | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___   
 |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \  
 | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) | 
 |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/  
 */


// Setup DS18B20 array address 
void setup_DS18B20_addr() {
	array_tSensor_addrs[0] = &sensor_t1_b;
	array_tSensor_addrs[1] = &sensor_t1_s;
	array_tSensor_addrs[2] = &sensor_t2_b;
	array_tSensor_addrs[3] = &sensor_t2_s;
}

/* Captura les temperatures via array de sensors */
void capture_temperatures() {
	sensorDS18B20.requestTemperatures();   // Requests culture temperatures from oneWire bus

	// Read temperatures array
	for (uint8_t i = 0; i < num_T; i++) {
		array_temps[i] = sensorDS18B20.getTempC(*array_tSensor_addrs[i]);
		delay(10);
	}
}

// Return ph value from SensorPin--> Pregunta, com sap quin és el SensorPin...?
float capture_pH(uint8_t SensorPin) {
    unsigned long int avgValue;         // Store the average value of the sensor feedback
    int buf[10], temp;

    for (int i=0; i<10; i++) {          // Get 10 sample value from the sensor for smooth the value
		buf[i] = analogRead(SensorPin);
		delay(10);
    }
	
	// sort the analog from small to large
    for (int i=0; i<9; i++) {
		for (int j=i+1; j<10; j++) {
			if (buf[i] > buf[j]) {
				temp = buf[i];
				buf[i] = buf[j];
				buf[j] = temp;
			}
        }
    }

    avgValue = 0;
    for (int i=2; i<8; i++)                      //take the average value of 6 center sample
        avgValue += buf[i];
	
    float phValue = (float)avgValue*5.0/1024/6;  //convert the analog into millivolt
    phValue = 3.5 * phValue;                     //convert the millivolt into pH value
    
    // Return phValue for that SensorPin
    return phValue;
}

// Deteccio si hi ha moviment via PIR
bool detect_PIR(int pin) {
    //for(int i=0; i<num_PIR; i++){
    // Si hi ha moviment al pin PIR retorna true
    return (digitalRead(pin) == HIGH);
}

/* Capture light ambient with LDR sensor*/
float capture_LDR() {
    if (option_lux == lux_BH1750)              // Return Lux value with BH1750
        return lux_sensor.readLightLevel();
    else if (option_lux == lux_LDR)            // Return Lux value with LDR
        return analogRead(ldr_pin);
    return 0;
}

void sort_array(float* arr, const uint8_t n_samples) {
    float tmp_val;

    for (int i=0; i < (n_samples-1); i++) {
        for (int j=0; j < (n_samples-(i+1)); j++) {
            if (arr[j] > arr[j+1]) {
                tmp_val = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = tmp_val;
            }
        }
    }
}

float sort_and_filter_CO2(float* llistat) {
	// Define array without first and last n elements
	float llistat_output;
	// Sort normally
	sort_array(llistat, co2_samples_number);

	// Descartem els 2 primers i 2 ultims valors
	for (uint8_t i=2; i<(co2_samples_number-2); i++) {
		llistat_output += llistat[i];
	}
	return llistat_output / (co2_samples_number - 4);
}

/* Capture CO2 */
void capture_CO2() {
  if (DEBUG) SERIAL_MON.println(F("Capturing CO2"));

  // Take co2_samples_number samples of Every co2 sensor
  float mostres_co2[co2_samples_number];
  for (uint8_t j=0; j<co2_samples_number; j++) {
    // Read voltage
    // Paso 1, conversión ADC de la lectura del pin analógico
    float adc = analogRead(pins_co2[0]);  
    // Paso 2, obtener el voltaje
    float voltaje = adc * 5.0 / 1023.0;

    float co2conc = voltaje*(-3157.89)+1420.0;
    SERIAL_MON.print(co2conc);
    SERIAL_MON.print(F(" ppm CO2 ["));
    SERIAL_MON.print(j);
    SERIAL_MON.println(F("]"));

    // Save voltatge value
    mostres_co2[j] = co2conc;
    delay(100);
    }
  array_co2[0] = sort_and_filter_CO2(mostres_co2);
}

/* Show obteined vales from LCD */
void mostra_LCD() {
    lcd.clear();                        // Clear screen

    // Shows on LCD the average of the two temperatures of  the first pair
    if (num_T > 0)
        lcd.add_value_read("T1:", (array_temps[0] + array_temps[1]) / 2);
    
    // Shows on LCD the average of the two temperatures of  the first pair
    if (num_T > 2)
        lcd.add_value_read("T2:", (array_temps[2] + array_temps[3]) / 2);
    
    if (num_pH > 0)
        lcd.add_value_read("pH1:", array_ph[0]);
    
    if (num_pH > 1)
        lcd.add_value_read("pH2:", array_ph[1]);

    if (num_CO2 > 0)
        lcd.add_value_read("CO2:", array_co2[0]);

    //TODO: Seria recomendable mostrar la fecha de la ultima captura o de la siguiente que se producirá ???
    /*
    lcd.setCursor(0, 2);       // go to the 3rd line
    if (last_send != "") {
        lcd.print(F("LAST: "));
        lcd.print(last_send);
    }
    */
}

/* Capture data in calibration mode */
void do_a_calibration_pH() {
  if (DEBUG) SERIAL_MON.println(F("Starting calibration mode..."));
  char buffer_L[6];               // String buffer
  
  lcd.clear();                    // Clear screen
  lcd.home ();                    // Linea 1
  lcd.print(F("pH Calibration"));

  if (num_pH > 0) {
    lcd.setCursor ( 0, 1 );       // go to the 2nd line
    lcd.print(F("pH1:"));
    dtostrf(capture_pH(pins_ph[0]),4,2,buffer_L);
    lcd.print(buffer_L);
  }
  if (num_pH > 1) {
    lcd.setCursor ( 0, 2 );       // go to the 3rd line
    lcd.print(F("pH2:"));
    dtostrf(capture_pH(pins_ph[1]),4,2,buffer_L);
    lcd.print(buffer_L);
  }
}

/* Obtain current value on specific Invasive sensor ACS712 */
const float get_current_ACS712(const uint8_t s_pin, const uint8_t n_samples, const float sensibility) {
    float voltage;
    float currentSum = 0.0;

    for (uint8_t i=0; i<n_samples; i++) {
        voltage = analogRead(s_pin) * 5.0 / 1023.0;
        currentSum += (voltage - 2.5) / sensibility;
        delay(CURR_SENS_MS_INTERV);
    }
    return (currentSum / n_samples);
}

/* Capture current values for all ACS712 configurated current sensors */
void capture_current_ACS712(const uint8_t n_samples, const float sensibility) {
    float consumption = 0;

    for (uint8_t i=0; i<num_current_sensor; i++) {
        array_current_ini[i] = get_current_ACS712(pins_current[i], n_samples, sensibility);
        consumption += array_current_ini[i];
    }

    if (consumption > 0) {
        if (DEBUG) { SERIAL_MON.print(F("Intensity: ")); SERIAL_MON.println(consumption); }
        delay(CURR_SENS_MS_BE);
    }

    for (uint8_t i=0; i<num_current_sensor; i++) {
        array_current_end[i] = get_current_ACS712(pins_current[i], n_samples, sensibility);
    }
}

void capture_current_SCT013() {
    // Not implemented jet
}

/* Obtain the name of first free file for writting to SD */
void SD_get_next_FileName(char* _fileName) {
    static int fileCount = 0;

    do {
        sprintf(_fileName, "%06d.txt", ++fileCount);
    } while (SD.exists(_fileName));
}

/* Writing title headers to file */
void SD_write_header(const char* _fileName) {
    myFile = SD.open(_fileName, FILE_WRITE);

    // If file not opened okay, exit
    if (!myFile) return;

    if (RTC_enabled) myFile.print(F("DateTime#"));

    // T1_s#T1_b#......#Tn_s#Tn_b#
    for (uint8_t i=0, j=0; i<num_T; i++) {
        if (i%2 == 0) {
            myFile.print(F("T"));
            myFile.print(j);
            myFile.print(F("_s#"));
        }
        else {
            myFile.print(F("T"));
            myFile.print(j);
            myFile.print(F("_b#"));
            j++;
        }
    }

    // ambient1_temp#ambient1_humetat#
    for (uint8_t i=0; i<dht_sensors.get_num_sensors(); i++) {
        myFile.print(F("ambient_"));
        myFile.print(i);
        myFile.print(F("_temp#"));
        myFile.print(F("ambient_"));  
        myFile.print(i);
        myFile.print(F("_humetat#"));      
    }

    // Lux sensor
    if (lux_sensor.is_init()) myFile.print(F("lux#"));
    
    // pH Sensor
    for (uint8_t i=0; i<num_pH; i++) {
        myFile.print(F("pH_"));
        myFile.print(i);
        myFile.print(F("#"));
    }

    // DO Sensor
    for (uint8_t i=0; i<num_DO; i++) {
        // pre_lux value
        myFile.print(F("pre_lux#"));  
        // R
        myFile.print(F("DO_"));
        myFile.print(i);
        myFile.print(F("_R#"));
        // G
        myFile.print(F("DO_"));
        myFile.print(i);
        myFile.print(F("_G#"));
        // B
        myFile.print(F("DO_"));
        myFile.print(i);
        myFile.print(F("_B#"));
        // RGB
        myFile.print(F("DO_"));
        myFile.print(i);
        myFile.print(F("_RGB#"));                  
    }

    // CO2 Sensors
    for (uint8_t i=0; i<num_CO2; i++) {
        myFile.print(F("CO2_"));
        myFile.print(i);
        myFile.print(F("#"));
    }

    //TODO: Falta afegit les capçaleras de corrent ini & end

    myFile.println(F(""));               // End of line
    myFile.close();                      // close the file:
}

/* Writing results to file in SD card */
void SD_save_data(const char* _fileName) {
    String tmp_data = "";                                  // Temporal string to concatenate the information

    if (DEBUG) {
        SERIAL_MON.print("Try to open SD file: ");
        SERIAL_MON.println(_fileName);
    }
    
    myFile = SD.open(_fileName, FILE_WRITE);
    
    if (!myFile) {
        if (DEBUG) SERIAL_MON.println(F("Error opening SD file!"));
        return;      //Exit
    }

    //DateTime if have RTC
    if (RTC_enabled) {
        tmp_data += dateTimeRTC.getDateTime();
        tmp_data += F("#");
    }
    
    // Temperatures del cultiu Tn_s, Tn_b, ...
    for (uint8_t i=0; i<num_T; i++) {
        tmp_data += array_temps[i];
        tmp_data += F("#");
    }
    
    // Sensors DHT
	//TODO: Cambiar el volcado de datos por el de la funcion definida ya en la clase DHT_Sensor
    for (uint8_t i=0; i<dht_sensors.get_num_sensors(); i++) {
        tmp_data += dht_sensors.get_Temperature(i);
        tmp_data += F("#");
		tmp_data += dht_sensors.get_Humidity(i);
        tmp_data += F("#");
    }

    // Lux sensor
    if (lux_sensor.is_init()) {
        tmp_data += lux;
        tmp_data += F("#");
    }
    
    // pH Sensor
    for (uint8_t i=0; i<num_pH; i++) {
        tmp_data += array_ph[i];
        tmp_data += F("#");
    }
    
    // DO Sensor
    if (num_DO > 0) {                                          // If have DO sensor
        tmp_data += pre_lux;
        tmp_data += F("#");
                             
        tmp_data += do_sensor.get_Red_value();                 // R-G-B-RGB (white)
        tmp_data += F("#");
        tmp_data += do_sensor.get_Green_value();
        tmp_data += F("#");
        tmp_data += do_sensor.get_Blue_value();
        tmp_data += F("#");
        tmp_data += do_sensor.get_White_value();
        tmp_data += F("#");
    }
    
    // CO2 Sensors
    for (uint8_t i=0; i<num_CO2; i++) {
        tmp_data += array_co2[i];
        tmp_data += F("#");
    }
    
    // Current sensor init & end
    for (uint8_t i=0; i<num_current_sensor; i++) {
        tmp_data += array_current_ini[i];
        tmp_data += F("#");
    }

    for (uint8_t i=0; i<num_current_sensor; i++) {
        tmp_data += array_current_end[i];
        tmp_data += F("#");
    }

    if (DEBUG) {
        SERIAL_MON.print(F("Data to write:"));
        SERIAL_MON.println(tmp_data);
    }

    myFile.println(tmp_data);          // Save all obtained data in the file
    myFile.close();                    // And close the file
}

bool send_data_ethernet(String cadena) {
  if (!conexio_internet) {
    SERIAL_MON.println(F("[Ethernet] Conecting to router..."));
    conexio_internet = Ethernet.begin(mac);

    if (!conexio_internet) {
      SERIAL_MON.println(F("[Ethernet] Conection to router Failed"));
      return false;
    }
  }
  
  eth_client.stop();
  // if there's a successful connection:
  if (eth_client.connect(server, 80)) {
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
  if (DEBUG)
    SERIAL_MON.println(F("Initializing modem..."));
    
  if (!modem.restart()) {
    SERIAL_MON.println(F("Modem init [fail]"));
    delay(1000);
    return false;
  }

  if (DEBUG) {
    SERIAL_MON.println(F("Modem init [OK]"));    
    String modemInfo = modem.getModemInfo();
    SERIAL_MON.print(F("Modem: "));
    SERIAL_MON.println(modemInfo);
  }
  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");    
  if (connect_network()) {
    // Network OK
    SERIAL_MON.print(F("Connecting to "));
    SERIAL_MON.println(apn);
    if (!modem.gprsConnect(apn, user, pass)) {
      SERIAL_MON.println(F("GRPS [fail]"));
      delay(1000);

      if(step_retry == false) {
        SERIAL_MON.println(F("[Modem] Retrying connection !"));
        send_data_modem(cadena, true);  // Reconnect modem and init again
      }      
      return false;
    }
    SERIAL_MON.println(F("GPRS [OK]"));
  
    IPAddress local = modem.localIP();
    SERIAL_MON.print(F("Local IP: "));
    SERIAL_MON.println(local);
  
    SERIAL_MON.print(F("Connecting to "));
    SERIAL_MON.println(server);
    if (!client.connect(server, port)) {
      SERIAL_MON.println(F("Server [fail]"));
      delay(1000);
      if(step_retry == false) {
        SERIAL_MON.println(F("[Modem] Retrying connection !"));
        send_data_modem(cadena, true);  // Reconnect modem and init again
      }      
      return false;
    }
    SERIAL_MON.println(F("Server [OK]"));
  
      // Make a HTTP GET request:
    client.print(cadena + " HTTP/1.0\r\n");
    client.print(String("Host: ") + server + "\r\n");
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
      SERIAL_MON.println();
      client.stop();
      if(DEBUG) SERIAL_MON.println(F("Server disconnected"));
    
      modem.gprsDisconnect();
      if(DEBUG) SERIAL_MON.println(F("GPRS disconnected"));
      /*
      if (DEBUG) {
        SERIAL_MON.println(F("************************"));
        SERIAL_MON.print  (F("Received: "));
        SERIAL_MON.print(bytesReceived);
        SERIAL_MON.println(F(" bytes"));
      }
      */
      return true;
  }
  else {
    SERIAL_MON.println(F("[Modem] Fail !"));
    // Try one more time, if continue fails, continue
    if(step_retry == false) {
      SERIAL_MON.println(F("[Modem] Retrying connection !"));
      send_data_modem(cadena, true);  
    }
    return false; 
  }
  return false;
}

bool send_data_server() {
	String cadena = "GET /afegir.php?";
	
	// Append our ID Arduino
	cadena += "idarduino=";
	cadena += id_arduino;

	// Append PIN for that ID
	cadena += "&pin=";
	cadena += pin_arduino;

	// Append temperatures
	for (int i=0; i<num_T; i++){
		cadena += "&temp";
		cadena += i+1;
		cadena += "=";
		cadena += array_temps[i];
	}

	// Append Ambient temperatures and Humidity
	//TODO: Cambiar el volcado de datos por el de la funcion definida ya en la clase DHT_Sensor
	for (uint8_t i=0; i<dht_sensors.get_num_sensors(); i++) {
		cadena += "&ta";
		cadena += i+1;
		cadena += "=";
		cadena += dht_sensors.get_Temperature(i);
		cadena += "&ha";
		cadena += i+1;
		cadena += "=";
		cadena += dht_sensors.get_Humidity(i);
	}
  
	// Append Lux sensors
	if (lux_sensor.is_init()) {
		cadena += "&lux1=";
		cadena += lux;
	}

	//TODO: Implementar sensor LDR
	/*
	if (ldr_sensor.is_ini()) {
		switch (option_lux) {
			case lux_LDR: cadena += "&ldr1="; break;
			case lux_BH1750: cadena += "&lux1=";
		}
		cadena += lux;
	}
	*/

	// Append pH Sensor
	for (int i=0; i<num_pH; i++) {
		cadena += "&ph";
		cadena += i+1;
		cadena += "=";
		cadena += array_ph[i];
	}  

	// Append DO Sensor
	if (num_DO > 0) {                                      // If have DO sensor
		cadena += "&pre_L=";                               // Previous lux
		cadena += pre_lux;                                 // Pre Lux value
		cadena += "&do1_R=";
		cadena += do_sensor.get_Red_value();               // Red value
		cadena += "&do1_G=";
		cadena += do_sensor.get_Green_value();             // Green value
		cadena += "&do1_B=";
		cadena += do_sensor.get_Blue_value();              // Blue value
		cadena += "&do1_RGB=";
		cadena += do_sensor.get_White_value();             // RGB (white) value
	}

	// Append PIR Sensor
	for (uint8_t i=0; i<num_PIR; i++) {
		cadena += "&pir";
		cadena += i+1;
		cadena += "=";
		if(array_pir[i] == 0)
		cadena += "0";
		else if(array_pir[i] == 1)
		cadena += "1";
  	}  
	
	// Append CO2 Sensors
	for (uint8_t i=0; i<num_CO2; i++) {
		cadena += "&co2";
		cadena += i+1;
		cadena += "=";
		cadena += array_co2[i];
	}

	// Append Current initial & end
	for (uint8_t i=0; i<num_current_sensor; i++) {
		cadena += "&I_ini";
		cadena += i+1;
		cadena += "=";
		cadena += array_current_ini[i];
	}

	for (uint8_t i=0; i<num_current_sensor; i++) {
		cadena += "&I_end";
		cadena += i+1;
		cadena += "=";
		cadena += array_current_end[i];
	}  

	if (DEBUG) {
		SERIAL_MON.print("Server petition: ");
		SERIAL_MON.println(cadena);
	}

	// Send data to specific hardware
	if (option_internet == it_Ethernet) {
		// Add end of GET petition for Ethernet and send
		cadena += " HTTP/1.1";
		return send_data_ethernet(cadena);
	}
	else if (option_internet == it_GPRS) {
		// Send petition to GPRS Modem
		return send_data_modem(cadena, false);
	} else {
		return false;
	}
}

void load_SD_DHT_sensors(Load_SD_Config* ini) {
	char buffer[INI_FILE_BUFFER_LEN] = "";
	char tag_sensor[22] = "";
	bool found;
	uint8_t i = 1;

	Serial.println(F("Loading DHT sensors config.."));
	do {
		sprintf(tag_sensor, "sensor%d.pin", i++);
		found = ini->getValue("sensors:DHT", tag_sensor, buffer, sizeof(buffer));
		if (found) {
			uint8_t pin = atoi(buffer);
			Serial.print(F("  > Found config: ")); Serial.print(tag_sensor);
			Serial.print(F(". Pin = ")); Serial.println(pin);
			dht_sensors.add_sensor(pin);
		}
	} while (found);

	// If no configuration found in IniFile..
	if (dht_sensors.get_num_sensors() == 0) {
		if (DEBUG) SERIAL_MON.println(F("No DHT config. found. Loading default.."));
		for (i=0; i<DHT_DEF_NUM_SENSORS; i++) {
			Serial.print(F("  > Found config: sensor")); Serial.print(i+1);
			Serial.print(F(". Pin = ")); Serial.println(DHT_DEF_SENSORS[i]);
		}
	}
}

void load_SD_Lux_sensor(IniFile* ini) {
	char buffer[INI_FILE_BUFFER_LEN] = "";
	bool found;
	
	Serial.println(F("Loading Lux sensor config.."));
	
	// Read Lux sensor address
	found = ini->getValue("sensor:lux", "address", buffer, sizeof(buffer));
	if (found) {
		// Convert char[] address to byte value
        uint8_t address = (uint8_t)strtol(buffer, NULL, 16);
		Serial.print(F("  > Found config address: 0x")); Serial.println(address, HEX);
		
		// Read Lux Addr pin
		ini->getValue("sensor:lux", "addr_pin", buffer, sizeof(buffer));
		uint8_t addr_pin = atoi(buffer);                   // If addr_pin not found, the value is 0
		Serial.print(F("  > Addr pin: ")); Serial.println(addr_pin);

		lux_sensor.begin(address, addr_pin);               // Configure lux sensor with Ini loaded config.

        // Read number of samples to read from sensor
        ini->getValue("sensor:lux", "n_samples", buffer, sizeof(buffer));
        uint8_t n_samples = atoi(buffer);
        if (n_samples != 0) lux_sensor.set_n_samples(n_samples);
	}
    else {
		// Configure lux sensor with default configuration
        Serial.print(F("No config found. Loading default.."));
		if (LUX_SENS_ACTIVE) lux_sensor.begin(LUX_SENS_ADDR, LUX_SENS_ADDR_PIN);
	}
}

void load_SD_DO_sensor(IniFile* ini) {
   	char buffer[INI_FILE_BUFFER_LEN] = ""; 
	bool found;
    
    Serial.println(F("Loading DO sensor config.."));

    // Read DO sensor address (hexadecimal format)
	found = ini->getValue("sensor:DO", "address", buffer, sizeof(buffer));
    if (found) {
        uint8_t address, led_R_pin, led_G_pin, led_B_pin;

        address = (uint8_t)strtol(buffer, NULL, 16);    // Convert hex char[] to byte value
		Serial.print(F("  > Found config address: 0x")); Serial.println(address, HEX);

        // Read red LED pin
        if (!ini->getValue("sensor:DO", "led_R_pin", buffer, sizeof(buffer)), (uint8_t)led_R_pin)
            led_R_pin = DO_SENS_R_LED_PIN;

        // Read green LED pin
        if (!ini->getValue("sensor:DO", "led_G_pin", buffer, sizeof(buffer)), (uint8_t)led_G_pin)
            led_G_pin = DO_SENS_G_LED_PIN;
        
        // Read blue LED pin
        if (!ini->getValue("sensor:DO", "led_B_pin", buffer, sizeof(buffer)), (uint8_t)led_B_pin)
            led_B_pin = DO_SENS_B_LED_PIN;

        // Configure DO sensor with load parametern from IniFile
        do_sensor.begin(address, led_R_pin, led_G_pin, led_B_pin);
    }
    else {
        // Configure DO sensor with default configuration
        Serial.print(F("No config found. Loading default.."));
        do_sensor.begin(DO_SENS_ADDR, DO_SENS_R_LED_PIN, DO_SENS_G_LED_PIN, DO_SENS_B_LED_PIN);
    }
}



/*
   _____ ______ _______ _    _ _____
  / ____|  ____|__   __| |  | |  __ \
 | (___ | |__     | |  | |  | | |__) |
  \___ \|  __|    | |  | |  | |  ___/
  ____) | |____   | |  | |__| | |
 |_____/|______|  |_|   \____/|_|
*/
void setup() {
    pinMode(CALIBRATION_SWITCH_PIN, INPUT);                               //Configure the input pin for the calibration switch

    Wire.begin();                                                         //Initialize the I2C bus (BH1750 library doesn't do this automatically)

	// Initialize serial if DEBUG default value is true
	if (DEBUG) SERIAL_MON.begin(SERIAL_BAUD);
	while (DEBUG && !SERIAL_MON) { ; }                                    // wait for serial port to connect. Needed for native USB port only
    
	// Always init SD card because we need to read init configuration file.
	if (SD.begin(SD_CARD_SS_PIN)) {
		if (DEBUG) SERIAL_MON.println(F("Initialization SD done."));

        // Read initial config from file
        Load_SD_Config ini(INI_FILE_NAME);                                // IniFile configuration

        // Try to open and check config file
        if (ini.open_config()) {
            ini.load_bool("debug", "enabled", DEBUG);                     // Load if debug mode configuration
            ini.load_bool("LCD", "enabled", LCD_enabled);                 // Load if LCD is enabled
            ini.load_bool("RTC", "enabled", RTC_enabled);                 // Load if RTC is enabled
            ini.load_bool("SD_card", "save_on_sd", SD_save_enabled);      // Load if SD save is enabled

			load_SD_DHT_sensors(&ini);                                    // Load DHT sensors configuration
			load_SD_Lux_sensor(&ini);                                     // Initialize Lux light sensor
            load_SD_DO_sensor(&ini);                                      // Initialize DO sensor
        }
	}
	else {
		if (DEBUG) SERIAL_MON.println(F("Initialization SD failed!"));
	}

    // If DEBUG is active and Serial not initialized, then start this
    if (!DEBUG_DEF_ENABLED && DEBUG) SERIAL_MON.begin(SERIAL_BAUD);
        else if (DEBUG_DEF_ENABLED && !DEBUG) SERIAL_MON.end();

	// If save in SD card option is enabled
	if (SD_save_enabled) {
		SD_get_next_FileName(fileName);                                   // Obtain the next file name to write
		SD_write_header(fileName);                                        // Write File headers
	}

	//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx continue xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	












	// Verification that the number of temperature sensors is even
    if ((num_T % 2) != 0 && DEBUG) SERIAL_MON.println(F("[ERROR] On number of temperature sensors."));
	
    // If have DS18B20 Sensors, init them
    if (num_T > 0) {
		if (DEBUG) SERIAL_MON.print(F("Initializing DS18B20 BUS.. "));
		
		sensorDS18B20.begin();

		// Verify number of detected devices
		if (sensorDS18B20.getDeviceCount() != num_T) {
			if (DEBUG) {
				SERIAL_MON.print(F("[Error] Incorrect number DS18B20 devices detected! ["));
				SERIAL_MON.print(sensorDS18B20.getDeviceCount()); SERIAL_MON.print(F("] of: "));
				SERIAL_MON.println(num_T);
			}
		} else {
			if (DEBUG) SERIAL_MON.println(F(" Initialized OK"));
		}
		setup_DS18B20_addr();
	}



    // Inicialitza LCD en cas que n'hi haigui
	if (LCD_enabled) {
		if (DEBUG) SERIAL_MON.println(F("Initialization LCD.."));

		lcd.init();
		lcd.show_init_msg(LCD_INIT_MSG_L1, LCD_INIT_MSG_L2,
						  LCD_INIT_MSG_L3, LCD_INIT_MSG_L4, 5000);
    }

	// Inicialitza RTC en cas de disposar
	if (RTC_enabled) {
		if (DEBUG) SERIAL_MON.print(F("Init RTC Clock.. "));
		
		// Try to initialize the RTC module
		if (dateTimeRTC.begin()) {
			if (DEBUG) SERIAL_MON.println(dateTimeRTC.getDateTime());     // RTC works. Print current time
		}
		else {
			if (DEBUG) SERIAL_MON.println(F("No clock working"));         // RTC does not work
		}
	}

	// Initialize Ethernet shield
	if (option_internet == it_Ethernet) {
		// give the ethernet module time to boot up:
		delay(2000);
		if (DEBUG) SERIAL_MON.println(F("Starting Ethernet Module"));
	
		// start the Ethernet connection using a fixed IP address and DNS server:
		// Ethernet.begin(mac, ip, myDns, gateway, subnet);
		// DHCP IP ( For automatic IP )
		conexio_internet = Ethernet.begin(mac);

		if (!conexio_internet) SERIAL_MON.println(F("[Ethernet] Fail obtain IP"));
		
		// print the Ethernet board/shield's IP address:
		if (DEBUG) {
			SERIAL_MON.print(F("My IP address: "));
			SERIAL_MON.println(Ethernet.localIP());
		}
	}
	// Initialize GPRS Modem
	else if(option_internet == it_GPRS) {
		// Not implemented jet
	}
}

/*
  _      ____   ____  _____
 | |    / __ \ / __ \|  __ \
 | |   | |  | | |  | | |__) |
 | |   | |  | | |  | |  ___/
 | |___| |__| | |__| | |
 |______\____/ \____/|_|
*/
void loop() {
    // If pin calibration ph switch is HIGH
    while (digitalRead(CALIBRATION_SWITCH_PIN) == HIGH) {
        if (DEBUG) SERIAL_MON.println(F("Calibration switch active!"));
        do_a_calibration_pH();
        delay(10000);
    }

    if (DEBUG) {
        SERIAL_MON.print(F("\nFreeMem: ")); SERIAL_MON.print(freeMemory());
        SERIAL_MON.print(F(" - Loop: ")); SERIAL_MON.println(++loop_count);
		SERIAL_MON.println(F("Getting data.."));
    }

	if (LCD_enabled)
		lcd.print_msg_val(0, 3, "Getting data.. %d", (int32_t)loop_count);

    /* Capture current depending on the sensor type */
    switch (current_sensors_type) {
        case cs_ACS712:
            if (DEBUG) SERIAL_MON.println(F("Capture current ACS712.."));
            capture_current_ACS712(CURR_SENS_N_SAMPLES, current_sensibility);
            break;
        case cs_SCT013:
            if (DEBUG) SERIAL_MON.println(F("Capture current SCT013.."));
            capture_current_SCT013();
            break;
		default:
			break;
    }

    // Si tenim sondes de temperatura
    if (num_T > 0) {
		if (DEBUG) SERIAL_MON.println(F("Capture temperatures DS18B20.."));
		capture_temperatures();
	}
    
	// Capture PH for each pH Sensor
	for (uint8_t i=0; i<num_pH; i++) {
		if (DEBUG) {
			SERIAL_MON.print(F("Capture pH ")); SERIAL_MON.print(i+1);
			SERIAL_MON.print(F(".."));
		}
		array_ph[i] = capture_pH(pins_ph[i]);
	}

	if (dht_sensors.get_num_sensors() > 0) {
		if (DEBUG) SERIAL_MON.println(F("Capture DHT sensor(s).."));
		dht_sensors.capture_all_DHT();
	}

    if (lux_sensor.is_init()) {
		if (DEBUG) SERIAL_MON.println(F("Capture lux sensor.."));
		lux = lux_sensor.capture_lux();
	}

    // Capture status of PIR Sensors
    for (uint8_t i=0; i<num_PIR; i++) {
        if (detect_PIR(pins_pir[i]) == true)
            array_pir[i] = 1;
        else
            array_pir[i] = 0;
    }

    //Capture DO values (Red, Green, Blue, and White)
    if (num_DO > 0) {
		if (DEBUG) SERIAL_MON.println(F("Capture DO sensor.."));
        pre_lux = do_sensor.readLightLevel();
        do_sensor.capture_DO(); 
        delay(500);
    }
    
    // Capture CO2 concentration
    if (num_CO2 > 0) {
		if (DEBUG) SERIAL_MON.println(F("Capture CO2 sensor.."));
		capture_CO2();
	}

    // END of capturing values
    if (LCD_enabled) mostra_LCD();
    
	if (option_internet != it_none) {
		// Si s'envia correctament actualitzar last_send
		if  (send_data_server()) {
			delay(200);
			last_send = dateTimeRTC.getTime();
            delay(100);
            last_send += " OK";
            if (LCD_enabled) mostra_LCD();
        }
        else {
            delay(200);
            last_send = dateTimeRTC.getTime();
            delay(100);
            last_send += " FAIL";

            if (LCD_enabled) mostra_LCD();
        }
    }

    // Save data to SD card
    if (SD_save_enabled) SD_save_data(fileName);


	/********************************************
	***  Waiting time until the next reading  ***
	*********************************************/

	/* If have RTC clock make timer delay if not internal delay */
    if (RTC_enabled) {
		uint32_t time_next_loop = dateTimeRTC.inc_unixtime(DELAY_SECS_NEXT_READ);    // Set next timer loop for actual time + delay time

        if (DEBUG) {
            SERIAL_MON.print(F("Waiting for "));
            SERIAL_MON.print( dateTimeRTC.unix_time_remain(time_next_loop) );
            SERIAL_MON.println(F(" seconds"));
        }
        
        int32_t time_remain;
        do {
            time_remain = dateTimeRTC.unix_time_remain(time_next_loop);

            if (LCD_enabled) lcd.print_msg_val(0, 3, "Next read.: %ds ", time_remain);
            if (DEBUG) SERIAL_MON.print(F("."));
            if (digitalRead(CALIBRATION_SWITCH_PIN) == HIGH) {
                if (DEBUG) SERIAL_MON.println(F("Calibration switch activated!"));
                break;
            }

            delay(1000);
        } while (time_remain > 0);
    }

  /* If RTC is not available */
  else {
    // Waiting for 10 minutes
    for (uint16_t j=0; j<DELAY_SECS_NEXT_READ; j++) {
      if (digitalRead(CALIBRATION_SWITCH_PIN) == HIGH) {
        if (DEBUG) SERIAL_MON.println(F("Calibration switch activated!"));
        break;
      }

      if (DEBUG) SERIAL_MON.print(F("."));
      delay(1000);
    }
  }
  SERIAL_MON.flush();
}
