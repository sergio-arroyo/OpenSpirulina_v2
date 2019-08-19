/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * MQTT_Pub class used to publish topic messages to remote broker server
 * 
 */

#include "MQTT_Pub.h"

extern bool DEBUG;


MQTT_Pub::MQTT_Pub(MQTT_Cnn_st *_mqtt_inf, Culture_ID_st *_culture_id)
{
    memcpy(&mqtt_inf, _mqtt_inf, sizeof(MQTT_Cnn_st));        // Copy the MQTT connection inf.
    memcpy(&culture_id, _culture_id, sizeof(Culture_ID_st));  // Copy the culture ID struct
    sprintf(pub_topic, "%s/sensors", culture_id.host_id);     // Compose topic to publish

    mqtt_cli.setClient(eth_cli);
    mqtt_cli.setServer(mqtt_inf.server, mqtt_inf.port);
}

bool MQTT_Pub::broker_reconnect() {
    DEBUG_NL(F("[I] MQTT reconnect:"))
    DEBUG_V2(F("  > ID : "), culture_id.host_id)
    DEBUG_V2(F("  > Usr: "), mqtt_inf.usr)
    DEBUG_V2(F("  > Psw: "), mqtt_inf.psw)
    
    return mqtt_cli.connect(culture_id.host_id, mqtt_inf.usr, mqtt_inf.psw);
}

bool MQTT_Pub::publish_topic(const char *payload) {
    // If not connected to broker, try to reconnect
    if (!mqtt_cli.connected()) {
        DEBUG_NL(F("[I] Not connected to broker, try to reconnect.."))

        if (broker_reconnect()) {
            DEBUG_NL(F("OK"))
        } else {
            DEBUG_NL(F("ERROR"))

            return false;
        }
    }

    String str_tmp = F(INFLUXDB_MEASUREMENT);
    add_tags_struct(&str_tmp);                             // Adding tags

    str_tmp.concat(F(" "));
    str_tmp.concat(payload);                               // Adding fields

    if (DEBUG) {
        SERIAL_MON.println(F("\nPublishing MQTT msg:"));
        SERIAL_MON.print(F("  > Topic      = ")); SERIAL_MON.println(pub_topic);
        SERIAL_MON.print(F("  > Payload    = ")); SERIAL_MON.println(str_tmp.c_str());
        SERIAL_MON.print(F("  > Total size = "));
        SERIAL_MON.println(MQTT_MAX_HEADER_SIZE + 2 + strlen(pub_topic) + str_tmp.length());

        if (MQTT_MAX_PACKET_SIZE < MQTT_MAX_HEADER_SIZE + 2 
                + strlen(pub_topic) + str_tmp.length())
        {
            SERIAL_MON.print(F("[!] WARNING! topic+payload+2 > "));
            SERIAL_MON.println(MQTT_MAX_PACKET_SIZE);
        }
    }
    
    if (!mqtt_cli.publish(pub_topic, str_tmp.c_str())) {
        Serial.println(F("[E] ERROR sending topic"));

        return false;
    }

    Serial.println(F("[I] Topic sended OK"));
    
    return true;
}

void MQTT_Pub::add_tags_struct(String *str_out) {
    // Compose the tags stream data
    (*str_out).concat(F(",country="));
    (*str_out).concat(culture_id.country);
    (*str_out).concat(F(",city="));
    (*str_out).concat(culture_id.city);
    (*str_out).concat(F(",culture="));
    (*str_out).concat(culture_id.culture);
    (*str_out).concat(F(",host="));
    (*str_out).concat(culture_id.host_id);
}
