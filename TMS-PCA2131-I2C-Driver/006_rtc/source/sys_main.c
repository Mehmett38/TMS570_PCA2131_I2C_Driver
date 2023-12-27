/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
#include "string.h"
#include "ae_rtc.h"
#include "i2c.h"
#include "gio.h"

/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */

void delay(uint32_t time)
{
    time *= 11520;
    while(time--);
}

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */

    AE_initRTC(i2cREG1, SLAVE_ADDR);

    RTC_Time rtcTime;
    RTC_Time rtcRead;
    RTC_ALARM rtcAlarm;
    RTC_TimeStp rtcTimestamp1;
    RTC_TimeStp rtcTimestamp2;
    RTC_TimeStp rtcTimestamp3;
    RTC_TimeStp rtcTimestamp4;
    memset((void*)&rtcTimestamp1, 0, sizeof(RTC_TimeStp));
    memset((void*)&rtcTimestamp2, 0, sizeof(RTC_TimeStp));
    memset((void*)&rtcTimestamp3, 0, sizeof(RTC_TimeStp));
    memset((void*)&rtcTimestamp4, 0, sizeof(RTC_TimeStp));
    uint64_t epochTime;

    rtcTime.second100.second_100Reg = 0x0;
    rtcTime.seconds.secondsReg = 0x0;
    rtcTime.minutes.minutesReg = 0x30;
    rtcTime.hours.hoursReg = 0x20;
    rtcTime.days.daysReg = 0x25;
    rtcTime.weekDays.WEEKDAYS = DAY_MONDAY;
    rtcTime.months.monthsReg = 0x12;
    rtcTime.years.yearsReg = 0x23;


    uint8_t flag = 1;

#if 0      // set and get the time and epoch conversion example
        AE_setTime(&rtcTime, RTC_BCD);
        AE_getTime(&rtcRead, RTC_BCD);

        AE_time2epoch(&rtcRead, &epochTime, RTC_BCD);
        AE_epoch2time(&rtcRead, &epochTime, RTC_BCD);
#endif


    while(1)
    {

#if 0       // when give active, drive to ground related tsx pin
            // when give passive, read the timestamp register
            // working in polling mode
        AE_timestamp1AP(&rtcTimestamp1, ACTIVE);
        AE_timestamp1AP(&rtcTimestamp1, PASSIVE);
        AE_timestamp2AP(&rtcTimestamp2, ACTIVE);
        AE_timestamp2AP(&rtcTimestamp2, PASSIVE);
        AE_timestamp3AP(&rtcTimestamp3, ACTIVE);
        AE_timestamp3AP(&rtcTimestamp3, PASSIVE);
        AE_timestamp4AP(&rtcTimestamp4, ACTIVE);
        AE_timestamp4AP(&rtcTimestamp4, PASSIVE);
#endif

#if 0       // timestramp interrupt mode
        //enable timestamp1 interrupt
        AE_interruptAP(InterruptA, NONE_MASK1, TIMESTAMP1_INT, ACTIVE);

#endif


#if 0       // polling mode control of timestamp registers if pin driven grounded externally
        if(AE_RTC_GET_TSF1())
        {
            AE_timestamp1AP(&rtcTimestamp1, PASSIVE);
        }
#endif

#if 0     //second interrupt using
        if(flag)
        {
            //activate second interrupt
            AE_interruptAP(InterruptA, SECOND_INT, NONE_MASK2, ACTIVE);
            flag = 0;
//            AE_interruptAP(InterruptA, SECOND_INT, NONE_MASK2, PASSIVE);
        }
#endif

#if 0   //minute interrupt using
        if(flag)
        {
            //activate second interrupt
            AE_interruptAP(InterruptA, MINUTE_INT, NONE_MASK2, ACTIVE);
            flag = 0;
//            AE_interruptAP(InterruptA, SECOND_INT, NONE_MASK2, PASSIVE);
        }
#endif

#if 0      //alarm interrupt using
        if(flag)
        {
            flag = 0;

            rtcAlarm.secondAlarm.AE_S = 0;  // second alarm enable
            rtcAlarm.secondAlarm.TENS_DIGIT = 1;
            rtcAlarm.secondAlarm.ONES_DIGIT = 0;

            rtcAlarm.minuteAlarm.AE_M = 0;
            rtcAlarm.minuteAlarm.TENS_DIGIT = 3;
            rtcAlarm.minuteAlarm.ONES_DIGIT = 0;

            rtcAlarm.hourAlarm.H24.AE_H = 0;
            rtcAlarm.hourAlarm.H24.TENS_DIGIT = 2;
            rtcAlarm.hourAlarm.H24.ONES_DIGIT = 0;

            rtcAlarm.weekDayAlarm.AE_W = 0;
            rtcAlarm.weekDayAlarm.WEEKDAYS = DAY_MONDAY;

            rtcAlarm.dayAlarm.AE_D = 0;
            rtcAlarm.dayAlarm.TENS_DIGIT = 2;
            rtcAlarm.dayAlarm.ONES_DIGIT = 5;

            AE_setAlarm(&rtcAlarm);

            AE_interruptAP(InterruptA, ALARM_INT, NONE_MASK2, ACTIVE);
        }
#endif

#if 0
        AE_interruptAP(InterruptA, BATTERY_LOW_INT, NONE_MASK2, ACTIVE);


#endif


        delay(100);

    }

/* USER CODE END */
}





/* USER CODE BEGIN (4) */
/* USER CODE END */
