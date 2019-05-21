/**
 * OpenSpirulina http://www.openspirulina.com
 * 
 * Autors: Sergio Arroyo (UOC)
 * 
 * General methods and functions
 * designed by OpenSpirulina
 * 
 */

#include "Genenal_functions.h"

/* Converts a string to bytes array like a device address */
bool convert_str_to_addr(char* str, uint8_t* addr, uint8_t max_len) {
    char *pch;
    uint8_t len = 0;
    
    pch = strtok(str, ",");
    while (pch != NULL && len < max_len) {
        addr[len] = (uint8_t)strtol(pch, NULL, 16);
        len++;
        pch = strtok(NULL, ",");
    }

    return (len <= max_len);
}

