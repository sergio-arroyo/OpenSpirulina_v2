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
    if (DEBUG) SERIAL_MON.println(F("[Eth] Try to get IP by DHCP.."));
    
    if (!eth->begin((uint8_t*) mac)) {
        if (DEBUG) SERIAL_MON.print(F("  > Error to obtain IP addr"));
        return false;
    }
    
    if (DEBUG) {
        SERIAL_MON.print(F("  > IP = "));
        SERIAL_MON.println(eth->localIP());
    }

    return true;
}

bool ETH_send_data_http_server(const char *host, uint16_t port, String *str_out) {
    EthernetClient eth_cli;

    eth_cli.stop();

    // if there's a successful connection:
    if (eth_cli.connect(host, port)) {
        if (DEBUG) {
            SERIAL_MON.print(F("Connecting to "));
            SERIAL_MON.println(host);
        }

        // Send string to internet
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
        if (DEBUG) 
            SERIAL_MON.println(F("  > Connection to server failed"));
        
        return false;
    }
}

bool MODEM_connect_network() {
    if (DEBUG)
        SERIAL_MON.println(F("[Modem] Waiting for network..."));

    if (!modem.waitForNetwork()) {
        if (DEBUG) SERIAL_MON.println(F("  > Network fail"));
        return false;
    }
    else {
        if (DEBUG) SERIAL_MON.println(F("  > Network OK"));
        return true;
    }
}

bool MODEM_send_data(String *str_out, const char *host, uint16_t port) {
    // Set GSM module baud rate
    SERIAL_AT.begin(9600);
    delay(3000);

    if (DEBUG) SERIAL_MON.println(F("[Modem] Initializing.."));
    
    if (!modem.restart()) {
        if (DEBUG) SERIAL_MON.println(F("  > Init fail"));
        return false;
    }

    if (DEBUG) {
        SERIAL_MON.println(F("  > Init OK"));    
        SERIAL_MON.print(F("  > Modem: "));
        SERIAL_MON.println(modem.getModemInfo());
    }

    if (!MODEM_connect_network()) {
        SERIAL_MON.println(F("[Modem] fail!"));
        return false;
    }

    // Unlock your SIM card with a PIN
    // Network OK
    if (DEBUG) {
        SERIAL_MON.print(F("Connecting to "));
        SERIAL_MON.println(GPRS_APN);
    }

    if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
        if (DEBUG) SERIAL_MON.println(F("GRPS [fail]"));
        return false;
    }
    
    IPAddress local = modem.localIP();
    if (DEBUG) {
        SERIAL_MON.println(F("GPRS [OK]"));
        SERIAL_MON.print(F("Local IP: "));
        SERIAL_MON.println(local);

        SERIAL_MON.print(F("Connecting to "));
        SERIAL_MON.println(host);
    }
        
    if (!client.connect(host, port)) {
        SERIAL_MON.println(F("  > Connect fail")); 
        return false;
    }

    if (DEBUG) SERIAL_MON.println(F("Server [OK]"));

    // Make a HTTP GET request:
    client.print(*str_out);
    client.print(F(" HTTP/1.0"));
    client.print(F("Host: "));
    client.println(host);
    client.println(F("Connection: close\r\n"));
    client.stop();

    if (DEBUG) SERIAL_MON.println(F("\nServer disconnected"));
    
    modem.gprsDisconnect();
    if (DEBUG) SERIAL_MON.println(F("GPRS disconnected"));
    
    return true;
}

