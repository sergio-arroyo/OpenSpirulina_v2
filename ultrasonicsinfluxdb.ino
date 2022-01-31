#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

#include <InfluxDbClient.h>
# define  PIN_TRIG 5
# define  PIN_ECHO 4

// Reemplazar con usuario y clave de su red

// WiFi AP SSID
#define WIFI_SSID "NODEMCU"
// WiFi password
#define WIFI_PASSWORD "12345678"

// For InfluxDB 1 e.g. http://192.168.1.48:8086
#define INFLUXDB_URL "http://192.168.1.42:8086"
#define INFLUXDB_DB_NAME "testdb"
float distancia;
int medida[600];


// InfluxDB client instance
//InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
// InfluxDB client instance for InfluxDB 1
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);

// Data point
Point sensor("nodemcu");

void setup() {
  Serial.begin(115200);

    pinMode (PIN_TRIG, OUTPUT);
  pinMode (PIN_ECHO, INPUT);
    
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

void loop() {
  // Store measured value into point
  sensor.clearFields();
  
  
   float max = 0, min = 1000, diferencia = 0, distanciaTotal = 0, media = 0;
  int i = 0;
  String resultado;
  unsigned long start = millis() + 5000;
  while (millis() < start) {
    digitalWrite(PIN_TRIG, LOW);  // para generar un pulso limpio ponemos a LOW 4us
    delayMicroseconds(4);

    digitalWrite(PIN_TRIG, HIGH);  // generamos Trigger (disparo) de 10us
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);


 tiempo = pulseIn (PIN_ECHO, HIGH);
    distancia = (tiempo / 2) / 29.1;
    distanciaTotal = distanciaTotal + distancia;
    medida[i] = distancia;
    i++;
    //Serial.println(distancia);
    delay(10);

//    if (distancia < min) {
//      min = distancia;
//    }
//    if (distancia > max) {
//      max = distancia;
//    }
  }
  BubbleSortAsc(medida, sizeof(medida));
  diferencia = medida[30] - medida[300];
  //Serial.println(max);
  //Serial.println(" ");
  //Serial.println(min);
  //Serial.println(" ");
  if (diferencia > 0.7) {
    resultado = "Movimiento";
  } else {
    media = distanciaTotal / i;
    resultado = media;
  }
  Serial.println(resultado);
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
