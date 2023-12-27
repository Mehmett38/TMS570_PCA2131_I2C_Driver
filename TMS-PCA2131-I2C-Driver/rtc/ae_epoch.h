/*
 * ae_epoch.h
 *
 *  Created on: Dec 23, 2023
 *      Author: Mehmet Dincer
 */

#ifndef RTC_AE_EPOCH_H_
#define RTC_AE_EPOCH_H_

#include <stdint.h>

#define EPOCH_IN_A_DAY                  (86400ul)
#define DAY_IN_A_YEAR                   (365)
#define EPOCH_IN_A_HOUR                 (3600)
#define EPOCH_IN_A_MINUTE               (60)
#define EPOCH_IN_A_YEAR                 (EPOCH_IN_A_DAY * 365)
#define OFFSET                          (1)

#define UTC_CONTROL                     (0)
#define UTC                             (UTC_CONTROL)   // if utc control == 1 add utc


typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint16_t days;
    uint8_t weekDays;
    uint8_t months;
    uint16_t years;
}RTC_TimeEpoch;


RTC_TimeEpoch epoch2time(uint64_t epochTim);
uint64_t time2epoch(RTC_TimeEpoch rtc);

#if UTC == 1
void setUtc(uint8_t utc);
uint8_t getUtc();
#endif


#endif /* RTC_AE_EPOCH_H_ */
