/**
 * OpenSpirulina http://www.openspirulina.com
 * 
 * Autors: Sergio Arroyo (UOC)
 * 
 * Sending data to remote server
 * 
 */
#ifndef OS_Data_Sender_h
#define OS_Data_Sender_h

#include <Arduino.h>
#include "Configuration.h"

#include <Ethernet.h>
#include <TinyGsmClient.h>


//TODO: documentar

bool ETH_initialize(EthernetClass *eth, uint8_t *mac);

bool ETH_send_data_http_server(const char *host, uint16_t port, String *str_out);

bool MODEM_connect_network();

bool MODEM_send_data(String *str_out, const bool step_retry);


#endif
