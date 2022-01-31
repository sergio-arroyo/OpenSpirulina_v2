#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif
#include "ph_grav.h" 
#include <InfluxDbClient.h>
String inputstring = "";                              //a string to hold incoming data from the PC
boolean input_string_complete = false;                //a flag to indicate have we received all the data from the PC
char inputstring_array[10];                           //a char array needed for string parsing
Gravity_pH pH = A0; 

// Reemplazar con usuario y clave de su red

// WiFi AP SSID
#define WIFI_SSID "NODEMCU"
// WiFi password
#define WIFI_PASSWORD "12345678"

// For InfluxDB 1 e.g. http://192.168.1.48:8086
#define INFLUXDB_URL "http://192.168.1.42:8086"
#define INFLUXDB_DB_NAME "testdb"



// InfluxDB client instance
//InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
// InfluxDB client instance for InfluxDB 1
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);

// Data point
Point sensor("nodemcu");

void setup() {
  Serial.begin(115200);
if (pH.begin()) { Serial.println("Loaded EEPROM");} 
  Serial.println(F("Use commands \"CAL,4\", \"CAL,7\", and \"CAL,10\" to calibrate the circuit to those respective values"));
  Serial.println(F("Use command \"CAL,CLEAR\" to clear the calibration"));
    
  // Connect WiFi
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Set InfluxDB 1 authentication params
  //client.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);

  // Add constant tags - only once

  sensor.addTag("type", "dynamic");

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
}
void serialEvent() {                                  //if the hardware serial port_0 receives a char
  inputstring = Serial.readStringUntil(13);           //read the string until we see a <CR>
  input_string_complete = true;                       //set the flag used to tell if we have received a completed string from the PC
}

void loop() {
  // Store measured value into point
  sensor.clearFields();
  
   if (input_string_complete == true) {                //check if data received
    inputstring.toCharArray(inputstring_array, 30);   //convert the string to a char array
    parse_cmd(inputstring_array);                     //send data to pars_cmd function
    input_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the PC
    inputstring = "";                                 //clear the string
  }
  Serial.println(pH.read_ph()); 
  
  // Esperando por nuevos clientes
  //start = millis()+10000;
  //while (millis() < start){
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Nuevo cliente");
    // Boleano para identificar cuando finaliza la solicitud HTTP
    boolean line = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n' && line) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");
          client.println();
          // Servidor Web muestra datos de temperatura
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          client.println("<head><meta http-equiv=""refresh"" content=""5""></head><body><h1>Enviando altura del agua o movimiento al servidor Web con HCSR04 y ESP8266");
          client.println("<table><tr><td>");
          client.println(resultado);
          client.println("</td></tr></table>");
          client.println("</body></html>");
          break;
        }
        if (c == '\n') {
          // Cuando comienza a leer una nueva linea
          line = true;
        }
        else if (c != '\r') {
          // Cuando en encuntra un caracter en la línea actual
          line = false;
        }
      }
    }
    // closing the client connection
    delay(10);
    client.stop();
    Serial.println("Cliente disconectado");

    int dist=resultado;
  
  Serial.println("Dist "+String(dist));
  // Report RSSI of currently connected network

  sensor.addField("dist", dist);
  
  Serial.println(sensor.toLineProtocol());
  // If no Wifi signal, try to reconnect it
  if ((WiFi.RSSI() == 0) && (wifiMulti.run() != WL_CONNECTED))
    Serial.println("Wifi connection lost");
  // Write point
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  //Wait 10s
  Serial.println("Wait 10s");
  delay(10000);
}
}

void parse_cmd(char* string) {                      //For calling calibration functions
  strupr(string);                                   //convert input string to uppercase

  if (strcmp(string, "CAL,4") == 0) {               //compare user input string with CAL,4 and if they match, proceed
    pH.cal_low();                                   //call function for low point calibration
    Serial.println("LOW CALIBRATED");
  }
  else if (strcmp(string, "CAL,7") == 0) {          //compare user input string with CAL,7 and if they match, proceed
    pH.cal_mid();                                   //call function for midpoint calibration
    Serial.println("MID CALIBRATED");
  }
  else if (strcmp(string, "CAL,10") == 0) {         //compare user input string with CAL,10 and if they match, proceed
    pH.cal_high();                                  //call function for highpoint calibration
    Serial.println("HIGH CALIBRATED");
  }
  else if (strcmp(string, "CAL,CLEAR") == 0) {      //compare user input string with CAL,CLEAR and if they match, proceed
    pH.cal_clear();                                 //call function for clearing calibration
    Serial.println("CALIBRATION CLEARED");
  }
}
