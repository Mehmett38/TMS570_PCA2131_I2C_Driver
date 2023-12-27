/*
 * ae_rtc.c
 *
 *  Created on: 20 Ara 2023
 *      Author: mehmet.dincer
 */

#include "ae_rtc.h"


i2cBASE_t * __aeI2c;
RTC __aeRtc;
uint16_t rtcSlaveAddr = SLAVE_ADDR;
uint8_t __intMaskA1 = 0;
uint8_t __intMaskA2 = 0;
uint8_t __intMaskB1 = 0;
uint8_t __intMaskB2 = 0;

/**
 * @brief initialize the rtc setting
 * @param[in] i2c register base address
 * @param[in] slave addres
 * @return none
 */
void AE_initRTC(i2cBASE_t * rtcI2c, uint16_t slaveAddres)
{
    rtcSlaveAddr = slaveAddres;
    __aeI2c = rtcI2c;

    i2cInit();
    //enable the RTC_INTA/B interrupt
    _enable_interrupt_();
    //RTC_INTA/B interrupt connected to HET1[16,18] pins
    hetInit();
    gioInit();
    /*set edge and edge interrupt in halcogen*/
    // het[16] falling edge interrupt enable
    edgeEnableNotification(hetREG1, edge0);
    // het[18] falling edge interrupt enable
    edgeEnableNotification(hetREG1, edge1);

    AE_registerInit();
}

/**
 * @brief initial value of PCA2131
 */
void AE_registerInit()
{
    __aeRtc.controlReg1.controlReg1Reg = 0x08;
    __aeRtc.controlReg2.controlReg2Reg = 0x00;
    __aeRtc.controlReg3.controlReg3Reg = 0xE0;
    __aeRtc.controlReg4.controlReg4Reg = 0x00;
    __aeRtc.controlReg5.controlReg5Reg = 0x00;
    __aeRtc.srReset.srResetReg = 0x24;
    __aeRtc.rtcTime.second100.second_100Reg = 0;
    __aeRtc.rtcTime.seconds.secondsReg = 0x80;
    __aeRtc.rtcTime.minutes.minutesReg = 0x00;
    __aeRtc.rtcTime.hours.hoursReg = 0x00;
    __aeRtc.rtcTime.days.daysReg = 0x01;
    __aeRtc.rtcTime.weekDays.weekDaysReg = 0x01;
    __aeRtc.rtcTime.months.monthsReg = 0x01;
    __aeRtc.rtcTime.years.yearsReg = 0x01;
    __aeRtc.rtcAlarm.secondAlarm.secondAlarmReg = 0x80;
    __aeRtc.rtcAlarm.minuteAlarm.minuteAlarmReg = 0x80;
    __aeRtc.rtcAlarm.hourAlarm.hourAlarmReg = 0x80;
    __aeRtc.rtcAlarm.weekDayAlarm.weekDayAlarmReg = 0x80;
    __aeRtc.rtcAlarm.dayAlarm.dayAlarmReg = 0x80;
    __aeRtc.clockOutCtl.clockOutCtlReg = 0x00;
    __aeRtc.rtcTimeStp1.timeStpCtl.timeStpCtlReg = 0x00;
    __aeRtc.rtcTimeStp1.secTimeStp.secTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp1.minTimeStp.minTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp1.hourTimeStp.hourTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp1.dayTimeStp.dayTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp1.monTimeStp.monTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp1.yearTimeStp.yearTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp2.timeStpCtl.timeStpCtlReg = 0x00;
    __aeRtc.rtcTimeStp2.secTimeStp.secTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp2.minTimeStp.minTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp2.hourTimeStp.hourTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp2.dayTimeStp.dayTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp2.monTimeStp.monTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp2.yearTimeStp.yearTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp3.timeStpCtl.timeStpCtlReg = 0x00;
    __aeRtc.rtcTimeStp3.secTimeStp.secTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp3.minTimeStp.minTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp3.hourTimeStp.hourTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp3.dayTimeStp.dayTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp3.monTimeStp.monTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp3.yearTimeStp.yearTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp4.timeStpCtl.timeStpCtlReg = 0x00;
    __aeRtc.rtcTimeStp4.secTimeStp.secTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp4.minTimeStp.minTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp4.hourTimeStp.hourTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp4.dayTimeStp.dayTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp4.monTimeStp.monTimeStpReg = 0x00;
    __aeRtc.rtcTimeStp4.yearTimeStp.yearTimeStpReg = 0x00;
    __aeRtc.agingOffset.agingOffsetReg = 0x08;
    __aeRtc.rtcIntAMask.intMask1.intMask1Reg = 0x3F;
    __aeRtc.rtcIntAMask.intMask2.intMask2Reg = 0x0F;
    __aeRtc.rtcIntBMask.intMask1.intMask1Reg = 0x3F;
    __aeRtc.rtcIntBMask.intMask2.intMask2Reg = 0x0F;
    __aeRtc.watchDogTimCtl.watchDogTimCtlReg = 0x03;
    __aeRtc.watchDogTimeVal.watchDogTimValReg = 0x00;

    AE_rtcWrite(__aeI2c, AE_RTC_CONTROL1_ADDR, &__aeRtc.controlReg1.controlReg1Reg, sizeof(RTC));
}

/**
 * @brief initialize the RTC first time
 * @param[in] RTC_Time keeps the all system time function
 * @return none
 */
void AE_setTime(RTC_Time * rtcTime, RtcRWFormat rtcFormat)
{
    if(rtcFormat == RTC_DECIMAL)
        AE_bcd2decimal(rtcTime);

    uint8_t *rtcVal = (uint8_t*)rtcTime;

    __aeRtc.controlReg1.STOP = 1;
    AE_rtcWrite(__aeI2c, AE_RTC_CONTROL1_ADDR, &__aeRtc.controlReg1.controlReg1Reg, 1);

    __aeRtc.srReset.CPR = 1;
    AE_rtcWrite(__aeI2c, AE_RTC_SR_RESET_ADDR, &__aeRtc.srReset.srResetReg, 1);

    AE_rtcWrite(__aeI2c, AE_RTC_SECOND100_ADDR, rtcVal, sizeof(RTC_Time));

    __aeRtc.controlReg1.STOP = 0;
    AE_rtcWrite(__aeI2c, AE_RTC_CONTROL1_ADDR, &__aeRtc.controlReg1.controlReg1Reg, 1);

    if(rtcFormat == RTC_DECIMAL)
        AE_decimal2bcd(rtcTime);

    /*
     * The first increment of the time circuits is between 0 s and 122 ms after STOP is released. See description for
     * STOP bit in Section 7.15
     */
    AE_delay(1);
}

/**
 * @brief get the current time
 * @param[in] RTC_Time keeps the all system time function
 * @param[in] Rtc return type @RTC_BCD
 *                            @RTC_DECIMAL
 */
void AE_getTime(RTC_Time * rtcTime, RtcRWFormat rtcFormat)
{
    AE_rtcRead(__aeI2c, AE_RTC_SECOND100_ADDR, (uint8_t*)rtcTime, sizeof(RTC_Time));

    if (rtcFormat == RTC_DECIMAL)
    {
        AE_bcd2decimal(rtcTime);
    }

    AE_delay(1);
}

/**
 * @brief i2c write function
 * @param[in] i2c base address
 * @param[in] slave devices register address
 * @param[in] write data pointer
 * @param[in] data len
 * @return none
 */
static void AE_rtcWrite(i2cBASE_t * rtcI2c, uint8_t regAddres, uint8_t *data, uint16_t dataLen)
{
    ///configure the slave address
    i2cSetSlaveAdd(__aeI2c, rtcSlaveAddr);

    /// direction configuration
    i2cSetDirection(__aeI2c, I2C_TRANSMITTER);

    /// sending and receiving data count is 1
    i2cSetCount(__aeI2c, dataLen + 1);      // + 1 for register addres

    ///Set mode as Master
    i2cSetMode(__aeI2c, I2C_MASTER);

    /// set stop after programmed count
    i2cSetStop(__aeI2c);

    /// transmit start condition
    i2cSetStart(__aeI2c);

    /// send register address
    i2cSendByte(__aeI2c, regAddres);

    ///* Tranmit DATA_COUNT number of data in Polling mode */
    if(dataLen != 0)
        i2cSend(__aeI2c, dataLen, data);

    /* Wait until Bus Busy is cleared */
    while(i2cIsBusBusy(__aeI2c) == true);

    /* Wait until Stop is detected */
    while(i2cIsStopDetected(__aeI2c) == 0);

    /* Clear the Stop condition */
    i2cClearSCD(__aeI2c);

    AE_delay(1);
}

/**
 * @brief i2c read function
 * @param[in] i2c base address
 * @param[in] slave devices register address
 * @param[in] read data pointer
 * @param[in] data len
 * @return none
 */
static void AE_rtcRead(i2cBASE_t * rtcI2c, uint8_t regAddres, uint8_t *data, uint16_t dataLen)
{
    /// few cycles after Bus Busy is cleared ///
    while(i2cIsMasterReady(__aeI2c) != true);

    /// Configure address of Slave to talk to
    i2cSetSlaveAdd(__aeI2c, rtcSlaveAddr);

    /// direction configuration
    i2cSetDirection(__aeI2c, I2C_TRANSMITTER);

    /// sending and receiving data count is dataLen
    i2cSetCount(__aeI2c, 1);

    ///Set mode as Master
    i2cSetMode(__aeI2c, I2C_MASTER);

    /// set stop after programmed count
    i2cSetStop(__aeI2c);

    /// transmit start condition
    i2cSetStart(__aeI2c);

    /// send slave address
    i2cSendByte(__aeI2c, regAddres);

    /* Wait until Bus Busy is cleared */
    while(i2cIsBusBusy(__aeI2c) == true);

    /* Wait until Stop is detected */
    while(i2cIsStopDetected(__aeI2c) == 0);

    /* Clear the Stop condition */
    i2cClearSCD(__aeI2c);

    /// few cycles after Bus Busy is cleared ///
    while(i2cIsMasterReady(__aeI2c) != true);

    /// Configure address of Slave to talk to
    i2cSetSlaveAdd(__aeI2c, rtcSlaveAddr);

    /// direction configuration
    i2cSetDirection(__aeI2c, I2C_RECEIVER);

    /// sending and receiving data count is dataLen
    i2cSetCount(__aeI2c, dataLen);

    ///Set mode as Master
    i2cSetMode(__aeI2c, I2C_MASTER);

    /// set stop after programmed count
    i2cSetStop(__aeI2c);

    /// transmit start condition
    i2cSetStart(__aeI2c);

    ////* Tranmit DATA_COUNT number of data in Polling mode */
    i2cReceive(__aeI2c, dataLen, data);

    /* Wait until Bus Busy is cleared */
    while(i2cIsBusBusy(__aeI2c) == true);

    /* Wait until Stop is detected */
    while(i2cIsStopDetected(__aeI2c) == 0);

    /* Clear the Stop condition */
    i2cClearSCD(__aeI2c);

    asm(" nop");    // not opearation
    asm(" nop");
    asm(" nop");

    AE_delay(1);
}

/**
 * @brief write the defined register addres
 * @param[in] i2c base addres
 * @param[in] addres that want to write
 * @param[in] value that want to write
 * @return none
 */
void AE_rtcWriteReg(i2cBASE_t * rtcI2c, uint8_t regAddres, uint8_t regVal)
{
    AE_rtcWrite(rtcI2c, regAddres, &regVal, 1);
}

/**
 * @brief read the defined addres register
 * @param[in] i2c base address
 * @param[in] address that want to read
 * @return register value
 */
uint8_t AE_rtcReadReg(i2cBASE_t * rtcI2c, uint8_t regAddres)
{
    uint8_t reg;
    AE_rtcRead(rtcI2c, regAddres, &reg, 1);
    return reg;
}

/**
 * @brief convert rtcTime to epoch
 * @param[in] rtcTime that keeps the time information
 * @param[in] epoch time pointer
 * @param[in] rtcTime store type can be RTC_BCD RTC_DECIMAL
 * @return none
 */
void AE_epoch2time(RTC_Time * rtcTime, uint64_t * epochTim, RtcRWFormat rtcFormat)
{
    RTC_TimeEpoch rtcEpoch;

    rtcEpoch = epoch2time(*epochTim);

    rtcTime->seconds.secondsReg = rtcEpoch.seconds;
    rtcTime->minutes.minutesReg = rtcEpoch.minutes;
    rtcTime->hours.hoursReg = rtcEpoch.hours;
    rtcTime->weekDays.weekDaysReg = rtcEpoch.weekDays;
    rtcTime->days.daysReg = rtcEpoch.days;
    rtcTime->months.monthsReg = rtcEpoch.months;
    rtcTime->years.yearsReg = rtcEpoch.years - 2000;  // last to digit

    if(rtcFormat == RTC_BCD)
    {
        AE_decimal2bcd(rtcTime);
    }
}

/**
 * @brief epoch to rtcTime
 * @param[in] rtcTime that keeps the time information
 * @param[in] epoch time pointer
 * @param[in] rtcTime store type can be RTC_BCD RTC_DECIMAL
 * @return none
 */
void AE_time2epoch(RTC_Time * rtcTime, uint64_t * epochTim, RtcRWFormat rtcFormat)
{
    RTC_TimeEpoch rtcEpoch;

    AE_bcd2decimal(rtcTime);

    rtcEpoch.seconds = rtcTime->seconds.secondsReg;
    rtcEpoch.minutes = rtcTime->minutes.minutesReg;
    rtcEpoch.hours = rtcTime->hours.hoursReg;
    rtcEpoch.weekDays = rtcTime->weekDays.weekDaysReg;
    rtcEpoch.days = rtcTime->days.daysReg;
    rtcEpoch.months = rtcTime->months.monthsReg;
    rtcEpoch.years = rtcTime->years.yearsReg + 2000;

    *epochTim = time2epoch(rtcEpoch);

    if(rtcFormat == RTC_BCD)
    {
        AE_decimal2bcd(rtcTime);
    }
}

/**
 * @brief The frequency of tuning fork quartz crystal oscillators is temperature-dependent.
 * In the PCA2131, the frequency deviation caused by temperature variation is corrected
 *  by adjusting the RTC frequency divider with digital method.
 * @param[in] PASSIVE = 0, ACTIVE = 1
 */
void AE_temperatureCompensateAP(RtcState rtcState)
{

    __aeRtc.controlReg1.SI = rtcState;
    __aeRtc.watchDogTimCtl.TI_TP = rtcState;
    __aeRtc.rtcIntAMask.intMask1.SI = rtcState;

    AE_rtcWrite(__aeI2c, AE_RTC_CONTROL1_ADDR, &__aeRtc.controlReg1.controlReg1Reg, 1);
    AE_rtcWrite(__aeI2c, AE_RTC_WATCHDOG_TIM_CTL_ADDR, &__aeRtc.watchDogTimCtl.watchDogTimCtlReg, 1);
    AE_rtcWrite(__aeI2c, AE_RTC_INTA_MASK1_ADDR, &__aeRtc.rtcIntAMask.intMask1.intMask1Reg, 1);
}

/**
 * @brief 100th seconds counter active or passive configuration
 * @param[in] PASSIVE = 0, ACTIVE = 1
 * @return none
 */
void AE_secondCounterAP(RtcState rtcState)
{
    __aeRtc.controlReg1.TH_S_DIS = !rtcState;
    AE_rtcWrite(__aeI2c, AE_RTC_CONTROL1_ADDR, &__aeRtc.controlReg1.controlReg1Reg, 1);
}

/**
 * @brief is 24 hour mode is passive 12 hour mode is used
 * @param[in] PASSIVE = 0, ACTIVE = 1
 * @return none
 */
void AE_24HourAP(RtcState rtcState)
{
    __aeRtc.controlReg1.H_12_24= !rtcState;
    AE_rtcWrite(__aeI2c, AE_RTC_CONTROL1_ADDR, &__aeRtc.controlReg1.controlReg1Reg, 1);
}

/**
 * @brief battery status low; flag cannot be cleared by command, flag is
 * automatically cleared when battery is replaced.
 * @return if battery is okey return ACTIVE else passive
 */
RtcState AE_checkBatteyStatus()
{
    AE_rtcRead(__aeI2c, AE_RTC_CONTROL3_ADDR, &__aeRtc.controlReg3.controlReg3Reg, 1);

    return (RtcState)(!__aeRtc.controlReg3.BLF);
}

/**
 * @brief set the temperature period
 * @param[in] period macros find by searching @groupTCR in header file
 * @return none
 */
void AE_tempMeasurePer(uint8_t period)
{
    __aeRtc.clockOutCtl.TCR = period;
    AE_rtcWrite(__aeI2c, AE_RTC_CLKOUT_CTL_ADDR, &__aeRtc.clockOutCtl.clockOutCtlReg, 1);
}

/**
 * @brief set the clock period
 * @param[in] period macros find by searching @groupCOF in header file
 * @return none
 */
void AE_clockOutput(uint8_t clockFrequency)
{
    __aeRtc.clockOutCtl.COF = clockFrequency;
    AE_rtcWrite(__aeI2c, AE_RTC_CLKOUT_CTL_ADDR, &__aeRtc.clockOutCtl.clockOutCtlReg, 1);
}

/**
 * @brief The accuracy of the frequency of a quartz crystal depends on its aging.
 * The aging offset adds an adjustment, positive or negative, in the temperature
 * compensation circuit which allows correcting the aging effect
 * @param[in] aging macros find by searching @groupAO in header file
 */
void AE_agingOffset(uint8_t agingOffset)
{
    __aeRtc.agingOffset.AO = agingOffset;
    AE_rtcWrite(__aeI2c, AE_RTC_AGING_OFFSET_ADDR, &__aeRtc.agingOffset.agingOffsetReg, 1);
}

/**
 * @brief timestamp1 pin is grounded save the that time in timestamp register
 * @param[in] timestamp1 register, if rtcState is ACTIVE does not change
 * @param[in] RtcState ACTIVE or PASSIVE; if select ACTIVE ts1 pin driven to ground
 * @return none
 */
void AE_timestamp1AP(RTC_TimeStp * rtcTimeStp, RtcState rtcState)
{
    if(rtcState == ACTIVE)
    {
        AE_hetSet(hetREG1, 20, 0);
        asm(" nop");    // not opearation
        asm(" nop");    // not opearation
        AE_hetSet(hetREG1, 20, 1);
    }
    else
    {
        AE_rtcRead(__aeI2c, AE_RTC_TIMESTP_CTL1_ADDR, (uint8_t*)rtcTimeStp, sizeof(RTC_TimeStp));
        AE_RTC_SET_TSF1();      //clear timestamp1 flag
    }
}

/**
 * @brief timestamp1 pin is grounded save the that time in timestamp register
 * @param[in] timestamp1 register, if rtcState is ACTIVE does not change
 * @param[in] RtcState ACTIVE or PASSIVE; if select ACTIVE ts1 pin driven to ground
 * @return none
 */
void AE_timestamp2AP(RTC_TimeStp * rtcTimeStp, RtcState rtcState)
{
    if(rtcState == ACTIVE)
    {
        gioSetBit(gioPORTB, 2, 0);
        asm(" nop");    // not opearation
        asm(" nop");    // not opearation
        gioSetBit(gioPORTB, 2, 1);
    }
    else
    {
        AE_rtcRead(__aeI2c, AE_RTC_TIMESTP_CTL2_ADDR, (uint8_t*)rtcTimeStp, sizeof(RTC_TimeStp));
        AE_RTC_SET_TSF2();      //clear timestamp2 flag
    }
}

/**
 * @brief timestamp1 pin is grounded save the that time in timestamp register
 * @param[in] timestamp1 register, if rtcState is ACTIVE does not change
 * @param[in] RtcState ACTIVE or PASSIVE; if select ACTIVE ts1 pin driven to ground
 * @return none
 */
void AE_timestamp3AP(RTC_TimeStp * rtcTimeStp, RtcState rtcState)
{
    if(rtcState == ACTIVE)
    {
        gioSetBit(gioPORTB, 3, 0);
        asm(" nop");    // not opearation
        asm(" nop");    // not opearation
        gioSetBit(gioPORTB, 3, 1);
    }
    else
    {
        AE_rtcRead(__aeI2c, AE_RTC_TIMESTP_CTL3_ADDR, (uint8_t*)rtcTimeStp, sizeof(RTC_TimeStp));
        AE_RTC_SET_TSF3();      //clear timestamp3 flag
    }
}

/**
 * @brief timestamp1 pin is grounded save the that time in timestamp register
 * @param[in] timestamp1 register, if rtcState is ACTIVE does not change
 * @param[in] RtcState ACTIVE or PASSIVE; if select ACTIVE ts1 pin driven to ground
 * @return none
 */
void AE_timestamp4AP(RTC_TimeStp * rtcTimeStp, RtcState rtcState)
{
    if(rtcState == ACTIVE)
    {
        gioSetBit(gioPORTA, 0, 0);
        asm(" nop");    // not opearation
        asm(" nop");    // not opearation
        gioSetBit(gioPORTA, 0, 1);
    }
    else
    {
        AE_rtcRead(__aeI2c, AE_RTC_TIMESTP_CTL4_ADDR, (uint8_t*)rtcTimeStp, sizeof(RTC_TimeStp));
        AE_RTC_SET_TSF4();      //clear timestamp4 flag
    }
}

/**
 * @brif set the alar
 */
void AE_setAlarm(RTC_ALARM * rtcAlarm)
{
    __aeRtc.rtcAlarm = *rtcAlarm;

    AE_rtcWrite(__aeI2c, AE_RTC_SECOND_ALARM_ADDR, &__aeRtc.rtcAlarm.secondAlarm.secondAlarmReg, sizeof(RTC_ALARM));
}

/**
 * @brief enable the desired interrupt
 * @param[in] intMaskA or intMaskB
 * @param[in] intMask1 mask values
 * @param[in] intMask2 mask values
 * @param[in] if rtcState == active enable interrupt else disable interrupt
 * @return none
 */
void AE_interruptAP(Interrupt irq, InterruptMask1 intMask1, InterruptMask2 intMask2, RtcState rtcState)
{
    // mask1 interrupt enable
    switch(intMask1)
    {
        case SECOND_INT:
            __aeRtc.controlReg1.SI = rtcState;
            AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL1_ADDR, __aeRtc.controlReg1.controlReg1Reg);
        break;

        case MINUTE_INT:
            __aeRtc.controlReg1.MI = rtcState;
            AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL1_ADDR, __aeRtc.controlReg1.controlReg1Reg);
        break;

        case WATCHDOG_INT:
            __aeRtc.watchDogTimCtl.WD_CD = rtcState;
            AE_rtcWriteReg(__aeI2c, AE_RTC_WATCHDOG_TIM_CTL_ADDR, __aeRtc.watchDogTimCtl.watchDogTimCtlReg);
        break;

        case ALARM_INT:
            __aeRtc.controlReg2.AIE = rtcState;
            AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL2_ADDR, __aeRtc.controlReg2.controlReg2Reg);
        break;

        case BATTERY_LOW_INT:
            __aeRtc.controlReg3.BLIE = rtcState;
            AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL3_ADDR, __aeRtc.controlReg3.controlReg3Reg);
    }
    // mask2 interrupt enable
    switch(intMask2)
    {
        case (TIMESTAMP1_INT):
            __aeRtc.controlReg5.TSIE1 = rtcState;
            AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL5_ADDR, __aeRtc.controlReg5.controlReg5Reg);
        break;

        case (TIMESTAMP2_INT):
            __aeRtc.controlReg5.TSIE2 = rtcState;
            AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL5_ADDR, __aeRtc.controlReg5.controlReg5Reg);
        break;

        case (TIMESTAMP3_INT):
            __aeRtc.controlReg5.TSIE3 = rtcState;
            AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL5_ADDR, __aeRtc.controlReg5.controlReg5Reg);
        break;

        case (TIMESTAMP4_INT):
            __aeRtc.controlReg5.TSIE4 = rtcState;
            AE_rtcWriteReg(__aeI2c, AE_RTC_CONTROL5_ADDR, __aeRtc.controlReg5.controlReg5Reg);
        break;
    }

    // interrupt enable functions
    if(irq == InterruptA)
    {
        (rtcState == ACTIVE) ? (__intMaskA1 |= intMask1) : (__intMaskA1 &= (~intMask2));
        (rtcState == ACTIVE) ? (__intMaskA2 |= intMask1) : (__intMaskA2= (~intMask2));

        //enable interrupt mask
        if(rtcState == ACTIVE)
        {
            __aeRtc.rtcIntAMask.intMask1.intMask1Reg &= ~(intMask1);
            __aeRtc.rtcIntAMask.intMask2.intMask2Reg &= ~(intMask2);
        }
        else
        {
            __aeRtc.rtcIntAMask.intMask1.intMask1Reg |= (intMask1);
            __aeRtc.rtcIntAMask.intMask2.intMask2Reg |= (intMask2);
        }

        AE_rtcWrite(__aeI2c, AE_RTC_INTA_MASK1_ADDR,
                    & __aeRtc.rtcIntAMask.intMask1.intMask1Reg , sizeof(RTC_IntMask));
    }
    else
    {
        (rtcState == ACTIVE) ? (__intMaskB1 |= intMask1) : (__intMaskB1 &= (~intMask2));
        (rtcState == ACTIVE) ? (__intMaskB2 |= intMask1) : (__intMaskB2= (~intMask2));

        //enable interrupt mask
        if(rtcState == ACTIVE)
        {
            __aeRtc.rtcIntBMask.intMask1.intMask1Reg &= ~(intMask1);
            __aeRtc.rtcIntBMask.intMask2.intMask2Reg &= ~(intMask2);
        }
        else
        {
            __aeRtc.rtcIntBMask.intMask1.intMask1Reg |= (intMask1);
            __aeRtc.rtcIntBMask.intMask2.intMask2Reg |= (intMask2);
        }

        AE_rtcWrite(__aeI2c, AE_RTC_INTB_MASK1_ADDR,
                    & __aeRtc.rtcIntBMask.intMask1.intMask1Reg , sizeof(RTC_IntMask));
    }
}

/**
 * @brief set high or low desired bit
 * @param[in] hetReg1 or hetReg1
 * @param[in] related bit
 * @param[in] if value is 1 output is high else low
 * @return none
 */
static void AE_hetSet(hetBASE_t * hetReg, uint32 bit, uint32 value)
{
    if(value == 1)
    {
        hetReg->DSET = (uint32_t)1U<<bit;
    }
    else
    {
        hetReg->DCLR = (uint32_t)1U<<bit;
    }
}

/**
 * @brief get the status if pin
 * @param[in] het base address
 * @param[in] pin number
 * @return pin status
 */
uint32_t AE_hetGet(hetBASE_t * hetReg, uint32 bit)
{
    return (hetReg->DIN >> bit) & 1U;
}

/**
 * @brief convert BCD number to deciamal for ex : 0x21 = 21
 */
void AE_bcd2decimal(RTC_Time * rtcTime)
{
    BCD2DECIMAL(rtcTime->second100, rtcTime->second100.second_100Reg);
    BCD2DECIMAL(rtcTime->seconds, rtcTime->seconds.secondsReg);
    BCD2DECIMAL(rtcTime->minutes, rtcTime->minutes.minutesReg);
    BCD2DECIMAL(rtcTime->hours.H24, rtcTime->hours.hoursReg);
    BCD2DECIMAL(rtcTime->days, rtcTime->days.daysReg);
    BCD2DECIMAL(rtcTime->months, rtcTime->months.monthsReg);
    BCD2DECIMAL(rtcTime->years, rtcTime->years.yearsReg);

    /*@note!!! hours 12 saat mi 24 saat mi olduðu ayarlanacak*/
//    BCD2DECIMAL(rtcTime->hours, rtcTime->hours.hoursReg);
}

/**
 * @brief convert decimal number to BCD for ex : 2023 = 0x2023
 */
void AE_decimal2bcd(RTC_Time * rtcTime)
{
    DECIMAL2BCD(rtcTime->second100.second_100Reg, rtcTime->second100.second_100Reg);
    DECIMAL2BCD(rtcTime->seconds.secondsReg, rtcTime->seconds.secondsReg);
    DECIMAL2BCD(rtcTime->minutes.minutesReg, rtcTime->minutes.minutesReg);
    DECIMAL2BCD(rtcTime->hours.hoursReg, rtcTime->hours.hoursReg);
    DECIMAL2BCD(rtcTime->days.daysReg, rtcTime->days.daysReg);
    DECIMAL2BCD(rtcTime->months.monthsReg, rtcTime->months.monthsReg);
    DECIMAL2BCD(rtcTime->years.yearsReg, rtcTime->years.yearsReg);
}

/**
 * @brief delay Xms
 * @param[in] total delay time in ms
 * @return none
 */
void AE_delay(uint32_t time)
{
    time *= 11520;
    while(time--);
}

/**
 * @brief het interrupt callback function for ping INTA/B and TS1
 * @param[in] het register addres
 * @param[in] edge that is assined het pins
 * @return none
 */
void edgeNotification(hetBASE_t * hetREG,uint32 edge)
{
    InterruptMask1 intMask1;
    InterruptMask2 intMask2;

    switch(edge)
    {
        case edge0:
            intMask1 = (InterruptMask1)__intMaskB1;
            intMask2 = (InterruptMask2)__intMaskB2;
            AE_intCallback(InterruptB, intMask1, intMask2);
        break;
        case edge1:
            intMask1 = (InterruptMask1)__intMaskA1;
            intMask2 = (InterruptMask2)__intMaskA2;
            AE_intCallback(InterruptA, intMask1, intMask2);
        break;
    }
}

/**
 * @brief intA pin callback function
 */
static void AE_intCallback(Interrupt irq, InterruptMask1 intMask1, InterruptMask2 intMask2)
{
    //mask1 interrupt checking
    if((intMask1 & SECOND_INT) && AE_RTC_GET_MSF())
    {
        AE_RTC_SET_MSF();
        AE_secondIntCallBack(irq);
    }
    else if((intMask1 & MINUTE_INT) && AE_RTC_GET_MSF())
    {
        AE_RTC_SET_MSF();
        AE_minuteIntCallBack(irq);
    }
    else if((intMask1 == ALARM_INT) && AE_RTC_GET_AF())
    {
        AE_RTC_SET_AF();
        AE_alarmIntCallBack(irq);
    }
    else if((intMask1 == BATTERY_LOW_INT) && AE_RTC_GET_BLF())
    {
        AE_RTC_SET_BLF();
        AE_batteryLowIntCallBack(irq);
    }

    //mask2 interrupt checking
    if((intMask2 & TIMESTAMP1_INT) && AE_RTC_GET_TS1F())
    {
        AE_RTC_SET_TS1F();
        AE_timestamp1IntCallBack(irq);
    }
    else if((intMask2 & TIMESTAMP2_INT) && AE_RTC_GET_TS2F())
    {
        AE_RTC_SET_TS2F();
        AE_timestamp2IntCallBack(irq);
    }
    else if((intMask2 & TIMESTAMP3_INT) && AE_RTC_GET_TS3F())
    {
        AE_RTC_SET_TS3F();
        AE_timestamp2IntCallBack(irq);
    }
    else if((intMask2 & TIMESTAMP4_INT) && AE_RTC_GET_TS4F())
    {
        AE_RTC_SET_TS4F();
        AE_timestamp2IntCallBack(irq);
    }
}

/**
 * @brief second interrupt callback
 *  * @param[in] irq pinA or pinB
 */
#pragma WEAK(AE_secondIntCallBack)
void AE_secondIntCallBack(Interrupt irq)
{

}

/**
 * @brief minute interrupt callback
 *  * @param[in] irq pinA or pinB
 */
#pragma WEAK(AE_minuteIntCallBack)
void AE_minuteIntCallBack(Interrupt irq)
{

}

/**
 * @brief alarm interrupt callback
 *  * @param[in] irq pinA or pinB
 */
#pragma WEAK(AE_alarmIntCallBack)
void AE_alarmIntCallBack(Interrupt irq)
{

}

/**
 * @brief battery low interrupt callback
 * @param[in] irq pinA or pinB
 */
#pragma WEAK(AE_batteryLowIntCallBack)
void AE_batteryLowIntCallBack(Interrupt irq)
{

}

/**
 * @brief timestamp1 interrupt callback
 * @param[in] irq pinA or pinB
 */
#pragma WEAK(AE_timestamp1IntCallBack)
void AE_timestamp1IntCallBack(Interrupt irq)
{

}

/**
 * @brief timestamp1 interrupt callback
 * @param[in] irq pinA or pinB
 */
#pragma WEAK(AE_timestamp2IntCallBack)
void AE_timestamp2IntCallBack(Interrupt irq)
{

}

/**
 * @brief timestamp1 interrupt callback
 * @param[in] irq pinA or pinB
 */
#pragma WEAK(AE_timestamp3IntCallBack)
void AE_timestamp3IntCallBack(Interrupt irq)
{

}

/**
 * @brief timestamp1 interrupt callback
 * @param[in] irq pinA or pinB
 */
#pragma WEAK(AE_timestamp4IntCallBack)
void AE_timestamp4IntCallBack(Interrupt irq)
{

}













