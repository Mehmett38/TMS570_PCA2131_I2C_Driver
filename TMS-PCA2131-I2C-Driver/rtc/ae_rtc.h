/*
 * ae_rtc.h
 *
 *  Created on: 20 Ara 2023
 *      Author: mehmet.dincer
 */

#ifndef RTC_AE_RTC_H_
#define RTC_AE_RTC_H_


#include "stdint.h"
#include "i2c.h"
#include "ae_epoch.h"
#include "het.h"
#include "gio.h"
#include "sys_core.h"


#define SLAVE_ADDR                      (0x53)
#define SLAVE_ADDR_WRITE                (0xA6)
#define SLAVE_ADDR_READ                 (0xA7)

#define AE_RTC_CONTROL1_ADDR            (0x00)
#define AE_RTC_CONTROL2_ADDR            (0x01)
#define AE_RTC_CONTROL3_ADDR            (0x02)
#define AE_RTC_CONTROL4_ADDR            (0x03)
#define AE_RTC_CONTROL5_ADDR            (0x04)
#define AE_RTC_SR_RESET_ADDR            (0x05)
#define AE_RTC_SECOND100_ADDR           (0x06)
#define AE_RTC_SECONDS_ADDR             (0x07)
#define AE_RTC_MINUTE_ADDR              (0x08)
#define AE_RTC_HOURS_ADDR               (0x09)
#define AE_RTC_DAYS_ADDR                (0x0A)
#define AE_RTC_WEEKDAYS_ADDR            (0x0B)
#define AE_RTC_MONTHS_ADDR              (0x0c)
#define AE_RTC_YEARS_ADDR               (0x0D)
#define AE_RTC_SECOND_ALARM_ADDR        (0x0E)
#define AE_RTC_MINUTE_ALARM_ADDR        (0x0F)
#define AE_RTC_HOUR_ALARM_ADDR          (0x10)
#define AE_RTC_DAY_ALARM_ADDR           (0x11)
#define AE_RTC_WEEKDAY_ALARM_ADDR       (0x12)
#define AE_RTC_CLKOUT_CTL_ADDR          (0x13)
#define AE_RTC_TIMESTP_CTL1_ADDR        (0x14)
#define AE_RTC_SEC_TIMESTP1_ADDR        (0x15)
#define AE_RTC_MIN_TIMESTP1_ADDR        (0x16)
#define AE_RTC_HOUR_TIMESTP1_ADDR       (0x17)
#define AE_RTC_DAY_TIMESTP1_ADDR        (0x18)
#define AE_RTC_MON_TIMESTP1_ADDR        (0x19)
#define AE_RTC_YEAR_TIMESTP1_ADDR       (0x1A)
#define AE_RTC_TIMESTP_CTL2_ADDR        (0x1B)
#define AE_RTC_SEC_TIMESTP2_ADDR        (0x1C)
#define AE_RTC_MIN_TIMESTP2_ADDR        (0x1D)
#define AE_RTC_HOUR_TIMESTP2_ADDR       (0x1E)
#define AE_RTC_DAY_TIMESTP2_ADDR        (0x1F)
#define AE_RTC_MON_TIMESTP2_ADDR        (0x20)
#define AE_RTC_YEAR_TIMESTP2_ADDR       (0x21)
#define AE_RTC_TIMESTP_CTL3_ADDR        (0x22)
#define AE_RTC_SEC_TIMESTP3_ADDR        (0x23)
#define AE_RTC_MIN_TIMESTP3_ADDR        (0x24)
#define AE_RTC_HOUR_TIMESTP3_ADDR       (0x25)
#define AE_RTC_DAY_TIMESTP3_ADDR        (0x26)
#define AE_RTC_MON_TIMESTP3_ADDR        (0x27)
#define AE_RTC_YEAR_TIMESTP3_ADDR       (0x28)
#define AE_RTC_TIMESTP_CTL4_ADDR        (0x29)
#define AE_RTC_SEC_TIMESTP4_ADDR        (0x2A)
#define AE_RTC_MIN_TIMESTP4_ADDR        (0x2B)
#define AE_RTC_HOUR_TIMESTP4_ADDR       (0x2C)
#define AE_RTC_DAY_TIMESTP4_ADDR        (0x2D)
#define AE_RTC_MON_TIMESTP4_ADDR        (0x2E)
#define AE_RTC_YEAR_TIMESTP4_ADDR       (0x2F)
#define AE_RTC_AGING_OFFSET_ADDR        (0x30)
#define AE_RTC_INTA_MASK1_ADDR          (0x31)
#define AE_RTC_INTA_MASK2_ADDR          (0x32)
#define AE_RTC_INTB_MASK1_ADDR          (0x33)
#define AE_RTC_INTB_MASK2_ADDR          (0x34)
#define AE_RTC_WATCHDOG_TIM_CTL_ADDR    (0x35)
#define AE_RTC_WATCHDOG_TIM_VAL_ADDR    (0x36)


/*
 * @brief control register 3 macros for PWRMNG registers
 * groupPWRMNG
 */
//battery switch-over function is enabled in standard mode;
//battery low detection function is enabled
#define SWITCH_EN_STANDART_DET_EN        (000)

///battery switch-over function is enabled in standard mode;
///battery low detection function is disabled
#define SWITCH_EN_STANDART_DET_DIS       (001)

///battery switch-over function is enabled in direct switching mode;
///battery low detection function is enabled
#define SWITCH_EN_DIRECT_DET_EN          (011)

///battery switch-over function is enabled in direct switching mode;
///battery low detection function is disabled
#define SWITCH_EN_DIRECT_DET_DIS         (100)

///battery switch-over function is disabled, only one power supply (VDD);
///battery low detection function is disabled
#define SWITCH_DIS_DIRECT_DET_DIS        (111)

/*
 * @groupCOF
 * register macros of COF bits in register CLKOUT_ctl
 */
///  32768Hz
#define FREQUENCY_32768HZ               (000)
///  16384Hz
#define FREQUENCY_16384HZ               (001)
///  8192Hz
#define FREQUENCY_8192HZ                (010)
///  4096Hz
#define FREQUENCY_4096HZ                (011)
///  2048Hz
#define FREQUENCY_2048HZ                (100)
///  1024Hz
#define FREQUENCY_1024HZ                (101)
///  1Hz
#define FREQUENCY_1HZ                   (110)

/*
 * @groupTCR
 * register macros of TCR in register CLKOUT_ctl
 */
///32 min
#define TEMP_MEASURE_PER_32_MIN         (00)
///16 min
#define TEMP_MEASURE_PER_16_MIN         (01)
///8 min
#define TEMP_MEASURE_PER_8_MIN          (10)
///4 min
#define TEMP_MEASURE_PER_4_MIN          (11)

/*
 * @groupAO
 * register macros of AO in register Aging_offset
 */
#define AGING_OFFSET_16                 (0)
#define AGING_OFFSET_14                 (1)
#define AGING_OFFSET_12                 (2)
#define AGING_OFFSET_10                 (3)
#define AGING_OFFSET_8                  (4)
#define AGING_OFFSET_6                  (5)
#define AGING_OFFSET_4                  (6)
#define AGING_OFFSET_2                  (7)
#define AGING_OFFSET_0                  (8)
#define AGING_OFFSET_MINUS_2            (9)
#define AGING_OFFSET_MINUS_4            (10)
#define AGING_OFFSET_MINUS_6            (11)
#define AGING_OFFSET_MINUS_8            (12)
#define AGING_OFFSET_MINUS_10           (13)
#define AGING_OFFSET_MINUS_12           (14)
#define AGING_OFFSET_MINUS_14           (15)

/*
 * @groupWEEKDAYS
 * @brief DAYS register macros
 */
#define DAY_SUNDAY                      (0)
#define DAY_MONDAY                      (1)
#define DAY_TUESDAY                     (2)
#define DAY_WEDNESDAY                   (3)
#define DAY_THURSDAY                    (4)
#define DAY_FRIDAY                      (5)
#define DAY_SATURDAY                    (6)

/*
 * @groupTF
 * @brief timer source clock watchdog timer for Watchdg_tim_ctl register TF
 */
#define FREQUENCY_64HZ                  (00)
#define FREQUENCY_4HZ                   (01)
#define FREQUENCY_1_4HZ                 (10)
#define FREQUENCY_1_64HZ                (11)

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ENUMS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
typedef enum{
    RTC_BCD,            /// binary coded decimal
    RTC_DECIMAL         /// decimal
}RtcRWFormat;

typedef enum{
    PASSIVE,
    ACTIVE,
}RtcState;

typedef enum{
    InterruptA,
    InterruptB
}Interrupt;

typedef enum{
    NONE_MASK1 = 0x00,
    MINUTE_INT = 0x20,
    SECOND_INT = 0x10,
    WATCHDOG_INT = 0x08,
    ALARM_INT = 0x04,
    BATTERY_FLAG_INT = 0x02,
    BATTERY_LOW_INT = 0x01
}InterruptMask1;

typedef enum{
    NONE_MASK2 = 0x00,
    TIMESTAMP1_INT = 0x08,
    TIMESTAMP2_INT = 0x04,
    TIMESTAMP3_INT = 0x02,
    TIMESTAMP4_INT = 0x01,
}InterruptMask2;


///<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< TMS570LS12X is Big Endien >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
typedef union{
    uint8_t controlReg1Reg;
    struct{
        uint8_t EXT_TEST : 1;       /// normal mode/external clock test mode [0/1]
        uint8_t TC_DIS : 1;         /// Temperature compensation enabled/disabled [0/1]
        uint8_t STOP : 1;           /// RTC source clock runs/stops [0/1]
        uint8_t TH_S_DIS : 1;     /// 100th seconds counter enabled/disable [0/1]
        uint8_t POR_OVRD : 1;       /// Power-On Reset Override disable/enable [0/1]
        uint8_t H_12_24 : 1;        /// 24/12 hour mode [0/1]
        uint8_t MI : 1;             /// minute interrupt disable/enable [0/1]
        uint8_t SI : 1;             /// second interrupt disable/enable [0/1]
    };
}ControlReg_1;

typedef union{
    uint8_t controlReg2Reg;
    struct{
        uint8_t MSF : 1;            /// no/yes no minute or second interrupt enerated [0/1]
        uint8_t WDTF : 1;           /// no/yes no watchdog timer interrupt generated [0/1]
        uint8_t : 1;                /// unused
        uint8_t AF : 1;             /// no/yes flag set when alarm triggered[0/1]
        uint8_t : 2;                /// unused
        uint8_t AIE : 1;            /// no/yes interrupt generated when alarm flag set [0/1]
        uint8_t : 1;                /// unused
    };
}ControlReg_2;

typedef union{
    uint8_t controlReg3Reg;
    struct{
        uint8_t PWRMNG : 3;         /// no/yes timestamp when battery switch-over occurs [0/1]
        uint8_t BTSE : 1;           ///
        uint8_t BF : 1;             /// no/yes battery switch-over interrupt occurred [0/1]
        uint8_t BLF : 1;            /// battery status ok/low [0/1]
        uint8_t BIE : 1;            /// no/yes interrupt generated from the battery flag (BF) [0/1]
        uint8_t BLIE : 1;           /// no/yes  interrupt generated from battery low flag [0/1]
                                    /// for @params of PWRMNG search @groupPWRMNG
    };
}ControlReg_3;

typedef union{
    uint8_t controlReg4Reg;
    struct{
        uint8_t : 4;
        uint8_t TSF4 : 1;           /// no/yes timestamp interrupt generated when pin TS4 [0/1]
        uint8_t TSF3 : 1;           /// no/yes timestamp interrupt generated when pin TS3 [0/1]
        uint8_t TSF2 : 1;           /// no/yes timestamp interrupt generated when pin TS2 [0/1]
        uint8_t TSF1 : 1;           /// no/yes timestamp interrupt generated when pin TS1 [0/1]
    };
}ControlReg_4;

typedef union{
    uint8_t controlReg5Reg;
    struct{
        uint8_t : 4;                /// unused
        uint8_t TSIE4 : 1;          /// no/yes nterrupt generated from timestamp flag of TS4 [0/1]
        uint8_t TSIE3 : 1;          /// no/yes nterrupt generated from timestamp flag of TS3 [0/1]
        uint8_t TSIE2 : 1;          /// no/yes nterrupt generated from timestamp flag of TS2 [0/1]
        uint8_t TSIE1 : 1;          /// no/yes nterrupt generated from timestamp flag of TS1 [0/1]
    };
}ControlReg_5;

typedef union{
    uint8_t srResetReg;
    struct{
        uint8_t CPR : 1;            /// clear prescaler (CPR), 1010 0100 (A4h) must be sent
        uint8_t : 3;                /// unused value is 0b010
        uint8_t SR : 1;             /// trigger a software reset (SR), 0010 1100 (2Ch) must be sent
        uint8_t : 2;                /// unused value is 0b10
        uint8_t CTS : 1;            /// clear timestamp 0010 0101 (25h) must be sent to register Reset (address 05h)
    };
}SrReset;

typedef union{
    uint8_t second_100Reg;
    struct{
        uint8_t TENS_DIGIT : 4;     /// second tens place
        uint8_t ONES_DIGIT : 4;     /// second unit place
    };
}Second_100;

typedef union{
    uint8_t secondsReg;
    struct{
        uint8_t OSF : 1;            /// clock integrity is guaranteed/not quaranteed [0/1]
        uint8_t TENS_DIGIT : 3;     /// seconds BCD tens digit (ex: 59) Tens digit = 5
        uint8_t ONES_DIGIT : 4;     /// seconds BCD ones digit (ex: 59) Ones digit = 9
    };
}Seconds;

typedef union{
    uint8_t minutesReg;
    struct{
        uint8_t : 1;                /// unused
        uint8_t TENS_DIGIT : 3;     /// minute tens place
        uint8_t ONES_DIGIT : 4;     /// minute unit place
    };
}Minutes;

typedef union{
    uint8_t hoursReg;
    struct{
        uint8_t : 2;                /// unused
        uint8_t AM_PM : 1;          /// AM/PM [0/1]
        uint8_t TENS_DIGIT : 1;     /// hour tens place
        uint8_t ONES_DIGIT : 4;     /// hour unit place
    }H12;
    struct{
        uint8_t : 2;                /// unused
        uint8_t TENS_DIGIT : 2;     /// hour tens place
        uint8_t ONES_DIGIT : 4;     /// hour unit place
    }H24;
}Hours;

typedef union{
    uint8_t daysReg;
    struct{
        uint8_t : 2;                /// unused
        uint8_t TENS_DIGIT : 2;     /// days tens place
        uint8_t ONES_DIGIT : 4;     /// days unit place
    };
}Days;

typedef union{
    uint8_t weekDaysReg;
    struct{
        uint8_t : 5;                /// unused
        uint8_t WEEKDAYS : 3;       /// for @params of PWRWEEKDAYS search @groupWEEKDAYS
    };
}WeekDays;

typedef union{
    uint8_t monthsReg;
    struct{
        uint8_t : 3;                /// unused
        uint8_t TENS_DIGIT : 1;     /// months tens place
        uint8_t ONES_DIGIT : 4;     /// months unit place
    };
}Months;

typedef union{
    uint8_t yearsReg;
    struct{
        uint8_t TENS_DIGIT : 4;     /// years tens place
        uint8_t ONES_DIGIT : 4;     /// years unit place
    };
}Years;

typedef union{
    uint8_t secondAlarmReg;
    struct{
        uint8_t AE_S : 1;           /// second alarm is enabled/disabled [0/1]
        uint8_t TENS_DIGIT : 3;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    };
}SecondAlarm;

typedef union{
    uint8_t minuteAlarmReg;
    struct{
        uint8_t AE_M : 1;           /// minute alarm is enabled/disabled [0/1]
        uint8_t TENS_DIGIT : 3;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    };
}MinuteAlarm;

typedef union{
    uint8_t hourAlarmReg;
    struct{
        uint8_t AE_H : 1;           /// hour alarm is enabled/disabled [0/1]
        uint8_t : 1;                /// unused
        uint8_t AM_PM : 1;           /// AM-PM [0/1]
        uint8_t TENS_DIGIT : 1;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    }H12;
    struct{
        uint8_t AE_H : 1;           /// hour alarm is enabled/disabled [0/1]
        uint8_t : 1;                /// unused
        uint8_t TENS_DIGIT : 2;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    }H24;
}HourAlarm;

typedef union{
    uint8_t dayAlarmReg;
    struct{
        uint8_t AE_D : 1;           /// hour alarm is enabled/disabled [0/1]
        uint8_t : 1;                /// unused
        uint8_t TENS_DIGIT : 2;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    };
}DayAlarm;

typedef union{
    uint8_t weekDayAlarmReg;
    struct{
        uint8_t AE_W : 1;           /// weakDay alarm is enabled/disabled [0/1]
        uint8_t : 4;
        uint8_t WEEKDAYS: 3;        /// for @params of PWRWEEKDAYS search @groupWEEKDAYS
    };
}WeekDayAlarm;

typedef union{
    uint8_t clockOutCtlReg;
    struct{
        uint8_t COF : 3;            /// CLKOUT frequency selection or @params of COF search @groupCOF
        uint8_t : 2;                /// unused
        uint8_t OTPR: 1;            /// no/yes OTP refresh [0/1]
        uint8_t TCR : 2;            /// temperature measurement period @params of TCR search @groupTCR
    };
}ClockOutCtl;

typedef union{
    uint8_t timeStpCtlReg;
    struct{
        uint8_t TSM : 1;            /// the last/first event is stored [0/1]
        uint8_t TSOFF : 1;          /// timestamp function active/disabled [0/1]
        uint8_t : 1;                /// unused
        uint8_t SUBSEC : 5;         /// BCD @note!!!
    };
}TimeStpCtl;

typedef union{
    uint8_t secTimeStpReg;
    struct{
        uint8_t : 1;                /// unused
        uint8_t TENS_DIGIT : 3;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    };
}SecTimeStp;

typedef union{
    uint8_t minTimeStpReg;
    struct{
        uint8_t : 1;                /// unused
        uint8_t TENS_DIGIT : 3;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    };
}MinTimeStp;

typedef union{
    uint8_t hourTimeStpReg;
    struct{
        uint8_t : 2;                /// unused
        uint8_t AM_PM : 1;          /// AM-PM [0/1]
        uint8_t TENS_DIGIT : 1;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    }H12;
    struct{
        uint8_t : 2;                /// unused
        uint8_t TENS_DIGIT : 2;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    }H24;
}HourTimeStp;

typedef union{
    uint8_t dayTimeStpReg;
    struct{
        uint8_t : 2;                /// unused
        uint8_t TENS_DIGIT : 2;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    };
}DayTimeStp;

typedef union{
    uint8_t monTimeStpReg;
    struct{
        uint8_t : 3;                /// unused
        uint8_t TENS_DIGIT : 1;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    };
}MonTimeStp;

typedef union{
    uint8_t yearTimeStpReg;
    struct{
        uint8_t TENS_DIGIT : 4;     /// unit place
        uint8_t ONES_DIGIT : 4;     /// tens place
    };
}YearTimeStp;

typedef union{
    uint8_t agingOffsetReg;
    struct{
        uint8_t AO : 4;             /// aging offset value @params of AO search @groupAO
        uint8_t : 4;
    };
}AgingOffset;

typedef union{
    uint8_t intMask1Reg;
    struct{
        uint8_t : 2;                /// unused
        uint8_t MI : 1;            /// minute interrupt mask
        uint8_t SI : 1;            /// second interrupt mask
        uint8_t WD_CD : 1;         /// watchdog interrupt mask
        uint8_t AIE : 1;           /// alarm interrupt mask
        uint8_t BIE : 1;           /// battery flag interrupt mask
        uint8_t BLIE : 1;          /// battery low flag interrupt mask
    };
}IntMask1;

typedef union{
    uint8_t intMask2Reg;
    struct{
        uint8_t : 4;                /// unused
        uint8_t TSIE1 : 1;         /// time stamp 1 interrupt mask
        uint8_t TSIE2 : 1;         /// time stamp 2 interrupt mask
        uint8_t TSIE3 : 1;         /// time stamp 3 interrupt mask
        uint8_t TSIE4 : 1;         /// time stamp 4 interrupt mask
    };
}IntMask2;

typedef union{
    uint8_t watchDogTimCtlReg;
    struct{
        uint8_t WD_CD : 1;          /// watchdog timer interrupt disabled/enabled [0/1]
        uint8_t : 1;                /// unused
        uint8_t TI_TP : 1;          /// interrupt pin INTA/B    @note!!!
        uint8_t : 3;                /// unused
        uint8_t TF : 2;             /// timer source clock for watchdog timer @params of TF search @groupTF
    };
}WatchDogTimCtl;

typedef union{
    uint8_t watchDogTimValReg;
}WatchDogTimVal;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Base Structures >>>>>>>>>>>>>>>>>>>>>>>>>>>
typedef struct{
    Second_100 second100;
    Seconds seconds;
    Minutes minutes;
    Hours hours;
    Days days;
    WeekDays weekDays;
    Months months;
    Years years;
}RTC_Time;

typedef struct{
    SecondAlarm secondAlarm;
    MinuteAlarm minuteAlarm;
    HourAlarm hourAlarm;
    DayAlarm dayAlarm;
    WeekDayAlarm weekDayAlarm;
}RTC_ALARM;

typedef struct{
    TimeStpCtl timeStpCtl;
    SecTimeStp secTimeStp;
    MinTimeStp minTimeStp;
    HourTimeStp hourTimeStp;
    DayTimeStp dayTimeStp;
    MonTimeStp monTimeStp;
    YearTimeStp yearTimeStp;
}RTC_TimeStp;

typedef struct{
    IntMask1 intMask1;
    IntMask2 intMask2;
}RTC_IntMask;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Collection Of All Struct>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
typedef struct{
    ControlReg_1 controlReg1;
    ControlReg_2 controlReg2;
    ControlReg_3 controlReg3;
    ControlReg_4 controlReg4;
    ControlReg_5 controlReg5;
    SrReset srReset;
    RTC_Time rtcTime;
    RTC_ALARM rtcAlarm;
    ClockOutCtl clockOutCtl;
    RTC_TimeStp rtcTimeStp1;
    RTC_TimeStp rtcTimeStp2;
    RTC_TimeStp rtcTimeStp3;
    RTC_TimeStp rtcTimeStp4;
    AgingOffset agingOffset;
    RTC_IntMask rtcIntAMask;
    RTC_IntMask rtcIntBMask;
    WatchDogTimCtl watchDogTimCtl;
    WatchDogTimVal watchDogTimeVal;
}RTC;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Function Prototypes >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void AE_initRTC(i2cBASE_t * rtcI2c, uint16_t slaveAddres);
void AE_registerInit();
static void AE_rtcWrite(i2cBASE_t * rtcI2c, uint8_t regAddres, uint8_t *data, uint16_t dataLen);
static void AE_rtcRead(i2cBASE_t * rtcI2c, uint8_t regAddres, uint8_t *data, uint16_t dataLen);
void AE_rtcWriteReg(i2cBASE_t * rtcI2c, uint8_t regAddres, uint8_t regVal);
uint8_t AE_rtcReadReg(i2cBASE_t * rtcI2c, uint8_t regAddres);
void AE_getTime(RTC_Time * rtcTime, RtcRWFormat rtcFormat);
void AE_setTime(RTC_Time * rtcTime, RtcRWFormat rtcFormat);
void AE_bcd2decimal(RTC_Time * rtcTime);
void AE_decimal2bcd(RTC_Time * rtcTime);
void AE_delay(uint32_t time);
void AE_epoch2time(RTC_Time * rtcTime, uint64_t * epochTim, RtcRWFormat rtcFormat);
void AE_time2epoch(RTC_Time * rtc, uint64_t * epochTim, RtcRWFormat rtcFormat);
void AE_temperatureCompensateAP(RtcState rtcState);
void AE_secondCounterAP(RtcState rtcState);
void AE_24HourAP(RtcState rtcState);
RtcState AE_checkBatteyStatus();
void AE_tempMeasurePer(uint8_t period);
void AE_clockOutput(uint8_t clockFrequency);
void AE_agingOffset(uint8_t agingOffset);
void AE_timestamp1AP(RTC_TimeStp * rtcTimeStp, RtcState rtcState);
void AE_timestamp2AP(RTC_TimeStp * rtcTimeStp, RtcState rtcState);
void AE_timestamp3AP(RTC_TimeStp * rtcTimeStp, RtcState rtcState);
void AE_timestamp4AP(RTC_TimeStp * rtcTimeStp, RtcState rtcState);
void AE_setAlarm(RTC_ALARM * rtcAlarm);
void AE_interruptAP(Interrupt irq, InterruptMask1 intMask1, InterruptMask2 intMask2, RtcState rtcState);
static void AE_hetSet(hetBASE_t * hetReg, uint32 bit, uint32 value);
uint32_t AE_hetGet(hetBASE_t * hetReg, uint32 bit);
void edgeNotification(hetBASE_t * hetREG, uint32 edge);
static void AE_intCallback(Interrupt irq, InterruptMask1 intMask1, InterruptMask2 intMask2);
void AE_secondIntCallBack(Interrupt irq);
void AE_minuteIntCallBack(Interrupt irq);
void AE_alarmIntCallBack(Interrupt irq);
void AE_batteryLowIntCallBack(Interrupt irq);
void AE_timestamp1IntCallBack(Interrupt irq);
void AE_timestamp2IntCallBack(Interrupt irq);
void AE_timestamp3IntCallBack(Interrupt irq);
void AE_timestamp4IntCallBack(Interrupt irq);

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Variables >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
extern i2cBASE_t * __aeI2c;
uint8_t __tempVar;
extern uint8_t __intMaskA1;
extern uint8_t __intMaskA2;
extern uint8_t __intMaskB1;
extern uint8_t __intMaskB2;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Function Macros >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define BCD2DECIMAL(x, y)          (y = x.TENS_DIGIT * 10 + x.ONES_DIGIT)
#define DECIMAL2BCD(x, y)          (y = ((x / 10) << 4) | (x % 10))
#define UNUSED(x)                  ((void)(x))

/* Read the timestramp register TSFx flag*/
#define AE_RTC_GET_TSF1()               (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR) >> 7 & 0x01)
#define AE_RTC_GET_TSF2()               (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR) >> 6 & 0x01)
#define AE_RTC_GET_TSF3()               (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR) >> 5 & 0x01)
#define AE_RTC_GET_TSF4()               (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR) >> 4 & 0x01)

//alarm flag
#define AE_RTC_GET_AF()                 (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL2_ADDR) >> 4 & 0x01)
//minute second flag
#define AE_RTC_GET_MSF()                (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL2_ADDR) >> 7 & 0x01)
//battery low flag
#define AE_RTC_GET_BLF()                (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL3_ADDR) >> 2 & 0x01)
//timestamp1 flag
#define AE_RTC_GET_TS1F()               (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR) >> 7 & 0x01)
//timestamp1 flag
#define AE_RTC_GET_TS2F()               (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR) >> 6 & 0x01)
//timestamp1 flag
#define AE_RTC_GET_TS3F()               (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR) >> 5 & 0x01)
//timestamp1 flag
#define AE_RTC_GET_TS4F()               (AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR) >> 4 & 0x01)

/* Clear the timestramp register TSFx flag*/
#define AE_RTC_SET_TSF1()                                           \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR);    \
        __tempVar &= ~(1 << 7);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL4_ADDR, __tempVar);

#define AE_RTC_SET_TSF2()                                           \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR);    \
        __tempVar &= ~(1 << 6);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL4_ADDR, __tempVar);

#define AE_RTC_SET_TSF3()                                           \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR);    \
        __tempVar &= ~(1 << 5);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL4_ADDR, __tempVar);

#define AE_RTC_SET_TSF4()                                           \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR);    \
        __tempVar &= ~(1 << 4);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL4_ADDR, __tempVar);

#define AE_RTC_SET_AF()                                             \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL2_ADDR);    \
        __tempVar &= ~(1 << 4);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL2_ADDR, __tempVar);

#define AE_RTC_SET_MSF()                                            \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL2_ADDR);    \
        __tempVar &= ~(1 << 7);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL2_ADDR, __tempVar);

#define AE_RTC_SET_BLF()                                            \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL3_ADDR);    \
        __tempVar &= ~(1 << 2);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL3_ADDR, __tempVar);

#define AE_RTC_SET_TS1F()                                           \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR);    \
        __tempVar &= ~(1 << 7);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL4_ADDR, __tempVar);

#define AE_RTC_SET_TS2F()                                           \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR);    \
        __tempVar &= ~(1 << 6);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL4_ADDR, __tempVar);

#define AE_RTC_SET_TS3F()                                           \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR);    \
        __tempVar &= ~(1 << 5);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL4_ADDR, __tempVar);

#define AE_RTC_SET_TS4F()                                           \
        __tempVar= AE_rtcReadReg(__aeI2c, AE_RTC_CONTROL4_ADDR);    \
        __tempVar &= ~(1 << 4);                                     \
        AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL4_ADDR, __tempVar);
#endif /* RTC_AE_RTC_H_ */























