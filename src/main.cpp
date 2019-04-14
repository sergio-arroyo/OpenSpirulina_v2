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
#include <Ethernet2.h>                           // Shield W5500 with Ethernet2.h will be used in this version of the project
#include <DHT.h>
#include <DHT_U.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <TinyGsmClient.h>
#include "MemoryFree.h"                          //TODO: Gestió de memòria             

// OpenSpirulina libs
#include "Load_SD_Config.h"
#include "DateTime_RTC.h"                        // Class to control RTC date time
#include "LCD_Screen.h"                          // Class for LCD control
#include "DO_Sensor.h"                           // Class for optical density control


/*****************
 * GLOBAL CONTROL
 *****************/
bool DEBUG = DEBUG_DEF_ENABLED;                  // Indicates whether the debug mode on serial monitor is active
bool LCD_enabled = LCD_DEF_ENABLED;              // Indicates whether the LCD is active
bool RTC_enabled = RTC_DEF_ENABLED;              // Indicates whether the RTC is active
bool SD_save_enabled = SD_SAVE_DEF_ENABLED;      // Indicates whether the save to SD is enabled




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
const uint8_t num_DHT = 1;                                 // Humidity and temperature ambient sensor. MAX 3
#define DHTTYPE DHT22                                      // Type of DHT sensor DHT11 - DHT22

const uint8_t num_PIR = 0;                                 // PIR movement sensor. MAX 3 --> S'ha de traure i enlloc seu posar Current pin 
const uint8_t num_DO = 1;                                  // Optical Density Sensor Module made by OpenSpirulina includes a RGB led + BH1750 lux sensor
const uint8_t num_pH = 1;                                  // pH sensor. MAX 3
const uint8_t num_CO2 = 0;                                 // CO2 sensor MAX ?

enum option_lux_type {                                     // Valid option_lux_type
	lux_none = 0,
	lux_LDR,
	lux_BH1750
};

const option_lux_type option_lux = lux_BH1750;             // No sensor | ldr sensor | lux BH1750

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
#if option_lux == lux_ldr                                  // Lux sensor with LDR
const uint8_t ldr_pin = 3;                                 // LDR pin (Analog)
#endif

const uint8_t pins_dht[num_DHT] = {OPENSPIR_VGA_PIN4};     // DHT Pins
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

File myFile;
char fileName[SD_MAX_FILENAME_SIZE] = "";                  // Name of file to save data readed from sensors

DateTime_RTC dateTimeRTC;                                  // RTC class object (DS3231 clock sensor)

LCD_Screen lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS,
                LCD_CONTRAST, LCD_BACKLIGHT_ENABLED);      // LCD screen

DO_Sensor do_sensor1(DO_SENS_ADDR,                         
                     DO_SENS_R_LED_PIN,
					 DO_SENS_G_LED_PIN,
					 DO_SENS_B_LED_PIN,
                     DO_SENS_N_SAMP_READ,
                     DO_SENS_MS_WAIT);                     // Sensor 1 [DO] ("Optical Density")








const uint8_t co2_samples_number = 15;

OneWire oneWireObj(ONE_WIRE_PIN);
DallasTemperature sensorDS18B20(&oneWireObj);

// Array de temperatures amb tamany num_temp sensors assignats
float array_temps[num_T];

// Array of DHT sensors
DHT* array_DHT[num_DHT];
// Array temperatures of DHT
float array_DHT_T[num_DHT];
// Array Humiditys of DHT
float array_DHT_H[num_DHT];

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


// Lux sensor with BH1750
BH1750 lux_sensor(0x5C);      //Si el ADDR està amb més de 0.7V, correspon l'irradiació que arriva a l'hivernacle.

//Define Temperature sensors adress: 
//Define pair of Temp1 sensors
DeviceAddress sensor_t1_b = {0x28, 0xFF, 0x72, 0x88, 0x24, 0x17, 0x03, 0x09};
DeviceAddress sensor_t1_s = {0x28, 0xFF, 0x1B, 0xD2, 0x24, 0x17, 0x03, 0x28};
// Define pair of Temp2 sensors
DeviceAddress sensor_t2_b = {0x28, 0xFF, 0xCA, 0xE5, 0x80, 0x14, 0x02, 0x16};
DeviceAddress sensor_t2_s = {0x28, 0xFF, 0x89, 0xBB, 0x60, 0x17, 0x05, 0x6D};

DeviceAddress* array_tSensor_addrs[num_T];         //Array of Temperatures from the culture

// Lux ambient value
float lux;
float pre_lux;

String last_send;                                  //Last time sended data
uint16_t loop_count = 0;                           //TODO: Contador de ciclos de lectura


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

// Captura les dades temperatures/humitat dels DHT
void capture_DHT() {
  for (uint8_t i=0; i<num_DHT; i++) {
    // Read Temperature
    array_DHT_T[i] = array_DHT[i]->readTemperature();
    // Read Humidity
    array_DHT_H[i] = array_DHT[i]->readHumidity();
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
bool detecta_PIR(int pin) {
    //for(int i=0; i<num_PIR; i++){
    // Si hi ha moviment al pin PIR retorna true
    return (digitalRead(pin) == HIGH);
}

// Capture Lux ambient
float capture_lux() {
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
	for (uint8_t l=2; l<co2_samples_number-2; l++) {
		llistat_output += llistat[l];
	}
	return llistat_output / (co2_samples_number - 4);
}

// Capture CO2
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

// Mostra per LCD les dades
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

// Capture data in calibration mode
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
    for (uint8_t i=0; i<num_DHT; i++) {
        myFile.print(F("ambient_"));
        myFile.print(i);
        myFile.print(F("_temp#"));
        myFile.print(F("ambient_"));  
        myFile.print(i);
        myFile.print(F("_humetat#"));      
    }

    // Lux sensor
    if (option_lux != lux_none) myFile.print(F("lux#"));
    
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
    for (uint8_t i=0; i<num_DHT; i++) {
        tmp_data += array_DHT_T[i];
        tmp_data += F("#");
        tmp_data += array_DHT_H[i];
        tmp_data += F("#");
    }

    // Lux sensor
    if (option_lux != lux_none) {
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
                             
        tmp_data += do_sensor1.get_Red_value();                // R-G-B-RGB (white)
        tmp_data += F("#");
        tmp_data += do_sensor1.get_Green_value();
        tmp_data += F("#");
        tmp_data += do_sensor1.get_Blue_value();
        tmp_data += F("#");
        tmp_data += do_sensor1.get_White_value();
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
	for( int i=0; i<num_T; i++){
		cadena += "&temp";
		cadena += i+1;
		cadena += "=";
		cadena += array_temps[i];
	}

	// Append Ambient temperatures and Humidity
	for (uint8_t i=0; i<num_DHT; i++) {
		cadena += "&ta";
		cadena += i+1;
		cadena += "=";
		cadena += array_DHT_T[i];
		cadena += "&ha";
		cadena += i+1;
		cadena += "=";
		cadena += array_DHT_H[i];
	}
  
	// Append Lux sensors
	if (option_lux != lux_none) {
		switch (option_lux) {
			case lux_LDR: cadena += "&ldr1="; break;
			case lux_BH1750: cadena += "&lux1=";
		}
		cadena += lux;
	}

	// Append pH Sensor
	for (int i=0; i<num_pH; i++) {
		cadena += "&ph";
		cadena += i+1;
		cadena += "=";
		cadena += array_ph[i];
	}  

	// Append DO Sensor
	if (num_DO > 0) {                                      // If have DO sensor
		// Previous lux
		cadena += "&pre_L=";
		cadena += pre_lux;                                 // Pre Lux value
		cadena += "&do1_R=";
		cadena += do_sensor1.get_Red_value();              // Red value
		cadena += "&do1_G=";
		cadena += do_sensor1.get_Green_value();            // Green value
		cadena += "&do1_B=";
		cadena += do_sensor1.get_Blue_value();             // Blue value
		cadena += "&do1_RGB=";
		cadena += do_sensor1.get_White_value();            // RGB (white) value
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
		SERIAL_MON.print(F("Server petition: "));
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
        SERIAL_MON.print(F("\nFreeMem: ")); SERIAL_MON.println(freeMemory());
        SERIAL_MON.println(F("Getting data.."));
        SERIAL_MON.print(F("Loop: "));
        SERIAL_MON.println(++loop_count);
    }

	if (LCD_enabled) {
		//TODO: Prueba de contador de ciclos de lectura
		//lcd.print_msg(0, 3, "Getting data..");
		lcd.print_msg_val(0, 3, "Getting data.. %d", (int32_t)loop_count);
	}

    /* Capture current depending on the sensor type */
    switch (current_sensors_type) {
        case cs_ACS712:
            //TODO: DEBUG
            if (DEBUG) SERIAL_MON.println(F("Capture current ACS712.."));  //???????????
            capture_current_ACS712(CURR_SENS_N_SAMPLES, current_sensibility);
            break;
        case cs_SCT013:
            //TODO: DEBUG
            if (DEBUG) SERIAL_MON.println(F("Capture current SCT013.."));  //???????????
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

    if (num_DHT > 0) {
		if (DEBUG) SERIAL_MON.println(F("Capture DHT sensor.."));
		capture_DHT();
	}

    if (option_lux != lux_none) {
		if (DEBUG) SERIAL_MON.println(F("Capture lux sensor.."));
		lux = capture_lux();
	}

    // Capture status of PIR Sensors
    for (uint8_t i=0; i<num_PIR; i++) {
        if (detecta_PIR(pins_pir[i]) == true)
            array_pir[i] = 1;
        else
            array_pir[i] = 0;
    }

    //Capture DO values (Red, Green, Blue, and White)
    if (num_DO > 0) {
		if (DEBUG) SERIAL_MON.println(F("Capture DO sensor.."));
        pre_lux = do_sensor1.readLightLevel();
        do_sensor1.capture_DO(); 
        delay(500);
		//TODO: [BEBUG] seguimiento error
		if (DEBUG) SERIAL_MON.println(F("Finalize capture DO sensor"));
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
            ini.load_value("debug", "enabled", DEBUG);                    // Load if debug mode configuration
            ini.load_value("LCD", "enabled", LCD_enabled);                // Load if LCD is enabled
            ini.load_value("RTC", "enabled", RTC_enabled);                // Load if RTC is enabled
            ini.load_value("SD_card", "save_on_sd", SD_save_enabled);     // Load if SD save is enabled
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

	// Declaring array of DHT22
	if (num_DHT > 0) {
		for (uint8_t i=0; i < num_DHT; i++) {
			array_DHT[i] = new DHT(pins_dht[i], DHTTYPE);
			// Init DHT
			if (DEBUG) {
				SERIAL_MON.print(F("Initializing DHT Sensor "));
				SERIAL_MON.print(i+1);
				SERIAL_MON.println(F(" .."));
			}
        	array_DHT[i]->begin();
        }
    }

    //Establint modo output per al LED si hi ha DO
    if (num_DO > 0) {
        if (do_sensor1.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {        
          if (DEBUG) SERIAL_MON.println(F("DO sensor started"));
        }
        else {
          if (DEBUG) SERIAL_MON.println(F("DO sensor: error initializing"));
        }
    }

    // Initialize BH1750 light sensor
	if (option_lux == lux_BH1750) {
		pinMode(LUX_SENSOR_ADDR, OUTPUT);                                 // Set ADDR in LUX_SENSOR_ADDR pin
		digitalWrite(LUX_SENSOR_ADDR, HIGH);                              // Apply Vcc in ADDR pin for set address to 0X5C
		delay(200);

		if (lux_sensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
			if (DEBUG) SERIAL_MON.println(F("Light BH1750 sensor started"));
      	}
		else {
			if (DEBUG) SERIAL_MON.println(F("Error initialising light sensor BH1750"));
		}
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
