#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

#include <InfluxDbClient.h>


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

#include <BH1750FVI.h>
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);
# define  PIN_RELE 0 //D3

// Data point
Point sensor("nodemcu");

void setup() {
  Serial.begin(115200);

   LightSensor.begin();
pinMode(PIN_RELE, OUTPUT);
digitalWrite(PIN_RELE, HIGH);

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

float densidadoptica=0;
unsigned long tomarMedida = millis();
void loop() {
  // Store measured value into point
  sensor.clearFields();
  
  if(millis()>tomarMedida){
  digitalWrite(PIN_RELE, LOW);
  delay(250);
  //Serial.println("Empieza");
  unsigned long start = millis()+1500;
  float media=0;
  int i=0;
  while (millis() < start) {
    uint16_t lux = LightSensor.GetLightIntensity();
    media=media+lux;
    i=i+1;
    //Serial.print("Light: ");
    //Serial.print(lux);
    //Serial.println(" lux");
}
  //Serial.println("Acaba");
  media=media/i;
  densidadoptica=media/11000;                   //11000 segun los valores calculados = 1
  Serial.println(densidadoptica);
  //Serial.print("La media es ");
  //Serial.print(media);
  //Serial.println(" lux");
  delay(250);
  digitalWrite(PIN_RELE, HIGH);  
  tomarMedida = millis()+180000;           //cambiar tiempo entre medidas
  }
  
  sensor.addField("Lightbiomass", densidadoptica);
  
  Serial.println(sensor.toLineProtocol());



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
            client.println("<head><meta http-equiv=""refresh"" content=""5""></head><body><h1>Enviando densidad optica al servidor Web con BH1750 y ESP8266");
            client.println("<table><tr><td>");
            client.println(densidadoptica);
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
