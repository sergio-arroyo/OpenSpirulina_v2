/**
 * OpenSpirulina http://www.openspirulina.com
 * 
 * Autors: Sergio Arroyo (UOC)
 * 
 * Sending data to remote server
 * 
 */

#include "Data_send.h"

#if DUMP_AT_COMMANDS == 1                                  // GPRS Modem
    #include <StreamDebugger.h>
    StreamDebugger debugger(SERIAL_AT, SERIAL_MON);
    TinyGsm modem(debugger);
#else
    TinyGsm modem(SERIAL_AT);
#endif

TinyGsmClient client(modem);                               // GSM Modem client

extern bool DEBUG;


bool ETH_initialize(EthernetClass *eth, uint8_t *mac) {
    DEBUG_NL(F("[Eth] Try to get IP by DHCP.."))
    
    if (!eth->begin((uint8_t*) mac)) {
        DEBUG_NL(F("  > Error to obtain IP addr"))
        return false;
    }

    DEBUG_V2(F("  > IP = "), eth->localIP())

    return true;
}

bool ETH_send_data_http_server(const char *host, uint16_t port, String *str_out) {
    EthernetClient eth_cli;

    eth_cli.stop();

    // if there's a successful connection:
    if (eth_cli.connect(host, port)) {
        DEBUG_V2(F("Connecting to "), host)

        // Send string to internet
        eth_cli.println(F("GET "));
        eth_cli.print(*str_out);                        // GET /search.asp?xxx
        eth_cli.println(F(" HTTP/1.1"));
        eth_cli.print(F("Host: "));
        eth_cli.println(host);              // ${server_addr} \r\n
        eth_cli.println(F("User-Agent: arduino-mcu"));
        eth_cli.println(F("Connection: close"));
        eth_cli.println();

        return true;
    }
    // If you couldn't make a connection:
    else {
        DEBUG_NL(F("  > Connection to server failed"))
        
        return false;
    }
}

bool MODEM_connect_network() {
    DEBUG_NL(F("[Modem] Waiting for network..."))

    if (!modem.waitForNetwork()) {
        DEBUG_NL(F("  > Network fail"))
        return false;
    }
    else {
        DEBUG_NL(F("  > Network OK"))
        return true;
    }
}

bool MODEM_send_data(String *str_out, const char *host, uint16_t port) {
    // Set GSM module baud rate
    SERIAL_AT.begin(9600);
    delay(3000);

    DEBUG_NL(F("[Modem] Initializing.."))
    
    if (!modem.restart()) {
        DEBUG_NL(F("  > Init fail"))
        return false;
    }

    DEBUG_NL(F("  > Init OK"))
    DEBUG_V2(F("  > Modem: "), modem.getModemInfo())

    if (!MODEM_connect_network()) {
        DEBUG_NL(F("[Modem] fail!"))
        return false;
    }

    // Unlock your SIM card with a PIN
    // Network OK
    DEBUG_V2(F("Connecting to "), GPRS_APN)

    if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
        DEBUG_NL(F("GRPS [fail]"))

        return false;
    }
    
    IPAddress local = modem.localIP();
    DEBUG_NL(F("GPRS [OK]"))
    DEBUG_V2(F("Local IP: "), local)
    DEBUG_V2(F("Connecting to "), host)
        
    if (!client.connect(host, port)) {
        SERIAL_MON.println(F("  > Connect fail")); 
        return false;
    }

    DEBUG_NL(F("Server [OK]"))
    
    // Make a HTTP GET request:
    client.print(F("GET "));
    client.print(*str_out);
    client.print(F(" HTTP/1.0"));
    client.print(F("Host: "));
    client.println(host);
    client.println(F("Connection: close\r\n"));
    client.stop();

    DEBUG_NL(F("\nServer disconnected"))
    
    modem.gprsDisconnect();
    DEBUG_NL(F("GPRS disconnected"))
    
    return true;
}

