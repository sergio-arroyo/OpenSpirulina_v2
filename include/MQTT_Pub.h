/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * MQTT_Pub class used to publish topic messages to remote broker server
 * 
 */
#ifndef OS_MQTT_Publisher_h
#define OS_MQTT_Publisher_h

#include <Arduino.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include "Configuration.h"
#include "OS_def_types.h"


//TODO: documentar clase

class MQTT_Pub {
public:
    MQTT_Pub(MQTT_Cnn_st *mqtt_inf, Culture_ID_st *_culture_id);
    
    bool broker_reconnect();
    bool publish_topic(const char *payload);

private:
    EthernetClient eth_cli;
    PubSubClient mqtt_cli;
    MQTT_Cnn_st mqtt_inf;

    char pub_topic[21];
    Culture_ID_st culture_id;

    void add_tags_struct(String *str_out);
};

#endif
