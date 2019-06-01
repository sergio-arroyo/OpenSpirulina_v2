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


class MQTT_Pub {
public:
    /**
     * Constructor
     * 
     * @param mqtt_inf Structure that identifies the remote broker server
     * @param _culture_id Structure that identifies the culture
     **/
    MQTT_Pub(MQTT_Cnn_st *mqtt_inf, Culture_ID_st *_culture_id);
    
    /**
     * Reconnect to remote broker configured in constructor method
     * 
     * @return returns true if the process executed correctly, otherwise returns false 
     **/
    bool broker_reconnect();
    
    /**
     * Reconnect to remote broker configured in constructor method
     * 
     * @param payload The char array of payload for send to remote broker
     * @return returns true if the process executed correctly, otherwise returns false 
     **/
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
