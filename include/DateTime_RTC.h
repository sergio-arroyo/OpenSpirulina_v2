/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Fran Romero
 *         Sergio Arroyo (UOC)
 * 
 * General methods and functions related to time management (date & time)
 * 
 */
#ifndef DATETIME_RTC_h
#define DATETIME_RTC_h

#include <RTClib.h>

class DateTime_RTC: public RTC_DS3231 {
public:
    DateTime_RTC(void);

    boolean begin(); 
    const char* getDateTime() const;
    const char* getDate() const;
    const char* getTime() const;
    uint32_t inc_unixtime(int32_t diffSecs);
    int32_t unix_time_remain(int32_t target);
    RTC_DS3231 rtc;

private:
    boolean initialized;

};

#endif
