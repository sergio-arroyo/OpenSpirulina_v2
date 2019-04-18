/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * Read the initial configuration file to activate or deactivate the different modules.
 * 
 */

#ifndef LOAD_SD_CONFIG_h
#define LOAD_SD_CONFIG_h

#include <Arduino.h>
#include <IniFile.h>
#include "Configuration.h"

class Load_SD_Config : public IniFile {
private:
	char buffer[INI_FILE_BUFFER_LEN];

public:
	Load_SD_Config(const char* filename);

	bool open_config();
    void load_bool(const char* section, const char* key, bool &val);
};

#endif
