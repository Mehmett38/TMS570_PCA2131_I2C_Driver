/*
 * ae_epoch.c
 *
 *  Created on: Dec 23, 2023
 *      Author: Mehmet Dincer
 */

#include "ae_epoch.h"


uint16_t month[12] = { 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 };
uint16_t monthLeap[12] = { 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366};
RTC_TimeEpoch rtc;
uint8_t utcVal = 0;


/**
*@brief convert epoch value to year,month,hour....
*@param[in] epoch time
*@return RTC_Time this structure store the all time information
*/
RTC_TimeEpoch epoch2time(uint64_t epochTim)
{
#if UTC == 1
    epochTim += (int)((float)utcVal * 3600.0);
#endif

    uint32_t totalDay = 0;
    uint8_t leakDay = 0;
    uint16_t i = 0;

    totalDay = epochTim / EPOCH_IN_A_DAY;
    rtc.years = 1970 + totalDay / DAY_IN_A_YEAR;            /// not fully correct year

    // year calculation
    for (i = 1972; i < rtc.years; i += 4)                   /// fully correct year
    {
        leakDay++;
    }

    rtc.years = 1970 + (totalDay - leakDay) / DAY_IN_A_YEAR;

    // month and day calculation
    if (rtc.years % 4 != 0)
    {
        rtc.days = (totalDay - leakDay) % DAY_IN_A_YEAR;

        for (i = 0; i < 12; i++)
        {
            if (rtc.days < month[i])
            {
                rtc.months = i + OFFSET;

                (i != 0) ? (rtc.days = rtc.days - month[i - 1] + OFFSET) : rtc.days;
                break;
            }
        }
    }
    else
    {
        rtc.days = (totalDay - leakDay) % DAY_IN_A_YEAR + OFFSET; // 365 + 1 day

        for (i = 0; i < 12; i++)
        {
            if (rtc.days < monthLeap[i])
            {
                rtc.months = i + OFFSET;

                (i != 0) ? (rtc.days = rtc.days - monthLeap[i - 1] + OFFSET) : rtc.days;
                break;
            }
        }
    }

    // weadkday calcuation
    rtc.weekDays = ((totalDay % 7) + 3) % 7 + OFFSET;   /// the first day is thursday +3 comes

    //hour calculation
    uint32_t currentDaySec = epochTim % EPOCH_IN_A_DAY;
    rtc.hours = currentDaySec / EPOCH_IN_A_HOUR;

    //minute calculation
    currentDaySec %= EPOCH_IN_A_HOUR;
    rtc.minutes = currentDaySec / 60;

    //second calculation
    currentDaySec %= EPOCH_IN_A_MINUTE;
    rtc.seconds = currentDaySec;

    return rtc;
}

/**
* @brief convert year, hour... information to epoch
*@param[in] RTC_Time this structure store the all time information
*@return epoch time
*/
uint64_t time2epoch(RTC_TimeEpoch rtc)
{
    uint64_t epoch = 0;
    uint16_t i = 0;


    uint8_t leapDays = 0;

    for (i = 1972; i < rtc.years; i += 4)
        leapDays++;

    // add the days of year
    if (rtc.years % 4 != 0)
    {
        (rtc.months == 1) ? rtc.days : (rtc.days += month[rtc.months - 2]);
    }
    else
    {
        (rtc.months == 1) ? rtc.days : (rtc.days += monthLeap[rtc.months - 2]);
    }

    rtc.days += leapDays - 1;   /// the current day is not finished

    epoch += (rtc.years - 1970) * (EPOCH_IN_A_YEAR);
    epoch += rtc.days * (EPOCH_IN_A_DAY);
    epoch += rtc.hours * (EPOCH_IN_A_HOUR);
    epoch += rtc.minutes * (EPOCH_IN_A_MINUTE);
    epoch += rtc.seconds;

    return epoch;
}


#if UTC == 1
/**
* @brief if UTC_CONTROL enabled can set the utc
*/
void setUtc(uint8_t utc)
{
    utcVal = utc;
}

/**
* @brief if UTC_CONTROL enabled can get the utc
*/
uint8_t getUtc()
{
    return utcVal;
}
#endif










