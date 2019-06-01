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

/**
 * Initialize the Ethernet interface with a specific MAC address
 * Note: The W5100 Ethernet does not have hardcoded MAC address
 * 
 * @param eth The EthernetClass addr where you want to load the object
 * @param mac The MAC address to use for request the IP by DHCP
 * @return returs true if the process has obtained an IP or false otherwise
 **/
bool ETH_initialize(EthernetClass *eth, uint8_t *mac);

/**
 * Send a specific data by HTTP GET method
 * 
 * @param host The address of the server to make the request
 * @param port The port of the server to make the request
 * @param str_out The string data to add in GET method
 * @return returs true if the request execute correcty or false otherwise
 **/
bool ETH_send_data_http_server(const char *host, uint16_t port, String *str_out);

/**
 * Initialize the modem interface
 * 
 * @return returs true if the initialization execute correcty or false otherwise
 **/
bool MODEM_connect_network();

/**
 * Send a specific data by HTTP GET method
 * 
 * @param str_out The string data to add in GET method
 * @param host The address of the server to make the request
 * @param port The port of the server to make the request
 * @return returs true if the request execute correcty or false otherwise
 **/
bool MODEM_send_data(String *str_out, const char *host, uint16_t port);

#endif
