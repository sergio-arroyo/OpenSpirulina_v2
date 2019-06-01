/**
 * OpenSpirulina http://www.openspirulina.com
 * 
 * Autors: Sergio Arroyo (UOC)
 * 
 * General methods and functions
 * designed by OpenSpirulina
 * 
 */
#ifndef OS_General_Functions_h
#define OS_General_Functions_h

#include <Arduino.h>
#include "Configuration.h"

/**
 * Converts a string to bytes array like a device address
 * 
 * @param str The data string containing hexadecimal values separated by commas
 *            MAC address example: "0x0xA0,0x75,0xCB,0xD6,0x4D,0x64"
 * @param addr The target array
 * @param max_len The maximum length of the target array
 * @return returs true if the conversion execute correcty or false otherwise
 **/
bool convert_str_to_addr(char* str, uint8_t* addr, uint8_t max_len);

/**
 * Print MAC address to Serial port
 * 
 * @param mac_addr The MAC address array
 **/
void print_mac_address(uint8_t *mac_addr);

#endif
