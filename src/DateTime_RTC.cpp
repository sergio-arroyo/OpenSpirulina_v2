/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Fran Romero
 *         Sergio Arroyo (UOC)
 * 
 * General methods and functions related to time management (date & time)
 * 
 */
#include <Arduino.h>
#include "DateTime_RTC.h"


DateTime_RTC::DateTime_RTC(void) : RTC_DS3231() {
    initialized = false;
}

boolean DateTime_RTC::begin() {
    initialized = RTC_DS3231::begin();

    return initialized;
}

//TODO: Revisar forma de retornar char*
const char* DateTime_RTC::getDateTime() const {
  DateTime dt_now = now();
  static char dttm[21];

  sprintf(dttm, "%02d/%02d/%02d %02d:%02d:%02d",
    dt_now.day(), dt_now.month(), dt_now.year(),
    dt_now.hour(), dt_now.minute(), dt_now.second());
  
  return dttm;
}

const char* DateTime_RTC::getDate() const {
  DateTime dt_now = now();
  static char dt[12];
  
  sprintf(dt, "%02d/%02d/%02d", dt_now.day(), dt_now.month(), dt_now.year());
  return dt;
}

const char* DateTime_RTC::getTime() const {
  DateTime dt_now = now();
  static char tm[10];
  
  sprintf(tm, "%02d:%02d:%02d", dt_now.hour(),  dt_now.minute(), dt_now.second());
  return tm;
}

uint32_t DateTime_RTC::inc_unixtime(int32_t diffSecs) {
  return now().unixtime() + diffSecs;
}

int32_t DateTime_RTC::unix_time_remain(int32_t target) {
  return target - now().unixtime();
}
