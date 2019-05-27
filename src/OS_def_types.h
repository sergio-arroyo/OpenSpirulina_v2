/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Define the specific data types
 * 
 */

#ifndef OS_def_types_h
#define OS_def_types_h

enum Internet_cnn_type : uint8_t {                         // Valid Internet connection types
	it_none = 0,
	it_Ethernet,
	it_GPRS,
	it_Wifi
};

struct Culture_ID_st {                                     // Identify a specific culture
    char country[4];
    char city[4];
    char culture[7];
    char host_id[11];
};

struct MQTT_Cnn_st {
    char server[51];
    uint16_t port;
    char usr[21]; 
    char psw[21];
};

#endif
