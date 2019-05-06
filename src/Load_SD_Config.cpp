/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Read the initial configuration file to activate or deactivate the different modules.
 * 
 */

#include "Load_SD_Config.h"

//TODO: Quitar clase Load_SD_Config y dejar solo clase base "IniFile.h"

/* Contructor class*/
Load_SD_Config::Load_SD_Config(const char* filename) : IniFile(filename) {
    
}

/* Open Ini config file and check if it's in correct format */
bool Load_SD_Config::open_config() {
    if (!open()) {
        SERIAL_MON.print(F("Ini file "));
        SERIAL_MON.print(getFilename());
        SERIAL_MON.println(F(" does not exist"));

        return false;
    }

    // Check the file is valid
    if (!validate(buffer, INI_FILE_BUFFER_LEN)) {        
        SERIAL_MON.print(F("Ini file "));
        SERIAL_MON.print(getFilename());
        SERIAL_MON.print(F(" not valid: "));
        
        return false;
    }

    return true;
}

/* Load required configuration value from section & key */
void Load_SD_Config::load_bool(const char* section, const char* key, bool &val) {
    SERIAL_MON.print(F("Ini. config: "));
    SERIAL_MON.print(section); SERIAL_MON.print(F(".")); SERIAL_MON.print(key); 
    
    if (getValue(section, key, buffer, INI_FILE_BUFFER_LEN, val)) {
        SERIAL_MON.print(F(" = "));
        SERIAL_MON.println(val? "TRUE" : "FALSE");
    } else {
        SERIAL_MON.println(F(" (not found)"));
    }
}
