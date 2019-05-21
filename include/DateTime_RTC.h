/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * DateTime_RTC class used to obtain and control time with RTC
 * 
 */
#ifndef DateTime_RTC_h
#define DateTime_RTC_h

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

    /**
     * Change date & time in RTC modeule
     * 
     * @year Year to update
     * @month Month to update
     * @day Day to update
     * @hour Hour to update
     * @min Minute to update
     * @sec Seconds to update
     **/
    void set_DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);

private:
    bool initialized;
    
};

#endif
