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


// TODO: documentar
bool convert_str_to_addr(char* str, uint8_t* addr, uint8_t max_len);

void print_mac_address(uint8_t *mac_addr);


#endif
