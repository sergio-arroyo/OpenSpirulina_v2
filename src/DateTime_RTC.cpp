/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * DateTime_RTC class used to obtain and control time with RTC
 * 
 */
#include "DateTime_RTC.h"


DateTime_RTC::DateTime_RTC() : RTC_DS3231() {
    initialized = false;
}

bool DateTime_RTC::begin() {
    initialized = RTC_DS3231::begin();

    return initialized;
}

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

void DateTime_RTC::getTime(char *tm) {
    DateTime dt_now = now();
    
    sprintf(tm, "%02d:%02d:%02d", dt_now.hour(), dt_now.minute(), dt_now.second());
}

uint32_t DateTime_RTC::inc_unixtime(int32_t diffSecs) {
    return now().unixtime() + diffSecs;
}

int32_t DateTime_RTC::unix_time_diff(int32_t target) {
    return target - now().unixtime();
}

void DateTime_RTC::set_DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    adjust(DateTime(year, month, day, hour, min, sec));
}
