/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * DateTime_RTC class used to obtain and control time with RTC
 * 
 */
#ifndef DATETIME_RTC_h
#define DATETIME_RTC_h

#include <Arduino.h>
#include <RTClib.h>

class DateTime_RTC : public RTC_DS3231 {
public:
    /** 
     * Constructor
     **/
    DateTime_RTC();

    /**
     * Initialize the RTC module
     * 
     * @return returns true if module is initialize correctly, otherwise returns false 
     **/
    bool begin();

    /**
     * Return if the RTC module is initialized
     * 
     * @return returns true if module is initialize correctly, otherwise returns false 
     **/
    bool is_init();

    /**
     * Get the date and time from RTC module
     * 
     * @return the date and time on format "DD/MM/YYYY hh:mm:ss"
     **/
    const char *getDateTime() const;

    /**
     * Get the date from RTC module
     * 
     * @return the date on format "DD/MM/YYYY"
     **/
    const char *getDate() const;

    /**
     * Get the time from RTC module
     * 
     * @return the time on format "hh:mm:ss"
     **/
    const char *getTime() const;

    /**
     * Get the UNIX time adding the time you want
     * 
     * @param diffSecs the number of seconds you want to add to the time obtained
     * @return the calculated unix time (in seconds)
     **/
    uint32_t inc_unixtime(int32_t diffSecs);

    /**
     * Calculates the time difference between the unix time of the RTC system and
     * the value provided
     * 
     * @param target the target UNIX time
     * @return the time difference (in seconds)
     **/
    int32_t unix_time_diff(int32_t target);

private:
    bool initialized;
    
};

#endif
