/*
 * Copyright (c) 2020 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "AM1805.h"

AM1805::AM1805(pin_t wdi_pin, TwoWire &wire, uint8_t address) :
    _wire(wire),
    _address(address),
    _wdt_reg(0x00),
    _wdi_pin(wdi_pin)
{
}

uint8_t AM1805::dec2hex(uint8_t dec)
{
    return ((dec / 10) << 4) + (dec % 10);
}

uint8_t AM1805::hex2dec(uint8_t hex)
{
    return ((hex >> 4) * 10) + (hex & 0xF);
}

void AM1805::begin()
{
    if(_wdi_pin != AM1805_PIN_INVALID)
    {
        pinMode(_wdi_pin, OUTPUT);
        digitalWriteFast(_wdi_pin, HIGH);
    }

    _wire.begin();
}

uint8_t AM1805::read_register(const uint8_t reg, const uint8_t mask)
{
    uint8_t value;
    WITH_LOCK(_wire)
    {
        _wire.beginTransmission(WireTransmission(_address).timeout(AM1805_I2C_TIMEOUT_MS));
        _wire.write(reg);
        _wire.endTransmission(false);
        _wire.requestFrom(WireTransmission(_address).timeout(AM1805_I2C_TIMEOUT_MS).quantity(1));
        value = _wire.read();
    }
    return (value & mask);
}

uint8_t AM1805::write_register(const uint8_t reg, const uint8_t data)
{
    uint8_t rtv;
    WITH_LOCK(_wire)
    {
        _wire.beginTransmission(WireTransmission(_address).timeout(AM1805_I2C_TIMEOUT_MS));
        _wire.write(reg);
        _wire.write(data);
        rtv = _wire.endTransmission();
    }
    return rtv;
}

struct tm AM1805::get_datetime(void)
{
    struct tm t;
    memset(&t, 0, sizeof(t));

    t.tm_sec  = hex2dec(read_register(AM1805_SECOND_REG,  AM1805_SECOND_MASK));
    t.tm_min  = hex2dec(read_register(AM1805_MINUTE_REG,  AM1805_MINUTE_MASK));
    t.tm_hour = hex2dec(read_register(AM1805_HOUR_REG,    AM1805_HOUR_24_MASK));
    t.tm_wday = hex2dec(read_register(AM1805_WEEKDAY_REG, AM1805_DAY_MASK));
    t.tm_mday = hex2dec(read_register(AM1805_DATE_REG,    AM1805_DATE_MASK));
    // struct tm expect month from 0-11, and RTC save from 1-12
    t.tm_mon  = hex2dec(read_register(AM1805_MONTH_REG,   AM1805_MONTH_MASK)) - 1;
    // struct tm expect years since 1900, and RTC save from 00~99 which is 2000~2099
    t.tm_year = hex2dec(read_register(AM1805_YEAR_REG,    AM1805_YEAR_MASK)) + 100;
    return t;
}

bool AM1805::set_datetime(const struct tm *t)
{
    uint8_t err = 0;
    err |= write_register(AM1805_SECOND_REG,  dec2hex(t->tm_sec));
    err |= write_register(AM1805_MINUTE_REG,  dec2hex(t->tm_min));
    err |= write_register(AM1805_HOUR_REG,    dec2hex(t->tm_hour));
    err |= write_register(AM1805_WEEKDAY_REG, dec2hex(t->tm_wday));
    err |= write_register(AM1805_DATE_REG,    dec2hex(t->tm_mday));
    err |= write_register(AM1805_MONTH_REG,   dec2hex(t->tm_mon + 1));
    err |= write_register(AM1805_YEAR_REG,    dec2hex(t->tm_year - 100));
    return  (err == 0);
}

time_t AM1805::get_seconds(void)
{
    struct tm t = get_datetime();
    return mktime(&t);
}

bool AM1805::set_seconds(time_t seconds)
{
    struct tm *t = gmtime(&seconds);
    return set_datetime(t);
}

am1805_alarm_t AM1805::get_alarm(void)
{
    am1805_alarm_t alarm = {0};
    alarm.month   = hex2dec(read_register(AM1805_MONTH_ALARM_REG,   AM1805_MONTH_ALARM_MASK) - 1);
    alarm.date    = hex2dec(read_register(AM1805_DATE_ALARM_REG,    AM1805_DATE_ALARM_MASK));
    alarm.weekday = hex2dec(read_register(AM1805_WEEKDAY_ALARM_REG, AM1805_WEEKDAY_ALARM_MASK));
    alarm.hour    = hex2dec(read_register(AM1805_HOUR_ALARM_REG,    AM1805_HOUR_24_ALARM_MASK));
    alarm.minute  = hex2dec(read_register(AM1805_MINUTE_ALARM_REG,  AM1805_MINUTE_ALARM_MASK));
    alarm.second  = hex2dec(read_register(AM1805_SECOND_ALARM_REG,  AM1805_SECOND_ALARM_MASK));
    alarm.repeat  = hex2dec(read_register(AM1805_TIMER_CONTROL_REG, AM1805_TIMER_CONTROL_RPT_MASK)) >> 2;
    return alarm;
}

bool AM1805::set_alarm(const am1805_alarm_t *alarm)
{
    uint8_t err = 0;
    uint8_t reg = 0;

    switch(alarm->repeat)
    {
    case AM1805_ALARM_REPEAT_YEAR:
        err |= write_register(AM1805_MONTH_ALARM_REG,   dec2hex(alarm->month + 1));
    case AM1805_ALARM_REPEAT_MONTH:
        err |= write_register(AM1805_DATE_ALARM_REG,    dec2hex(alarm->date));
    case AM1805_ALARM_REPEAT_WEEK:
    case AM1805_ALARM_REPEAT_DAY:
        err |= write_register(AM1805_HOUR_ALARM_REG,    dec2hex(alarm->hour));
    case AM1805_ALARM_REPEAT_HOUR:
        err |= write_register(AM1805_MINUTE_ALARM_REG,  dec2hex(alarm->minute));
    case AM1805_ALARM_REPEAT_MINUTE:
        err |= write_register(AM1805_SECOND_ALARM_REG,  dec2hex(alarm->second));
    }

    if(alarm->repeat == AM1805_ALARM_REPEAT_WEEK)
    {
        err |= write_register(AM1805_WEEKDAY_ALARM_REG, dec2hex(alarm->weekday));
    }

    reg = read_register(AM1805_TIMER_CONTROL_REG) & (~AM1805_TIMER_CONTROL_RPT_MASK);
    reg |= (alarm->repeat << 2);
    err |= write_register(AM1805_TIMER_CONTROL_REG, reg);

    return (err == 0);
}

/** Get/Clean Status Register
 *  mask list
 *   - default: 0xFF all status
 *   - AM1805_STATUS_CB_MASK
 *   - AM1805_STATUS_BAT_MASK
 *   - AM1805_STATUS_WDT_MASK
 *   - AM1805_STATUS_BL_MASK
 *   - AM1805_STATUS_TIM_MASK
 *   - AM1805_STATUS_ALM_MASK
 *   - AM1805_STATUS_EX2_MASK
 *   - AM1805_STATUS_EX1_MASK
 */
uint8_t AM1805::get_status(uint8_t mask)
{
    return read_register(AM1805_STATUS_REG, mask) ? 1 : 0;
}

bool AM1805::clean_status(uint8_t mask)
{
    uint8_t reg = read_register(AM1805_STATUS_REG) & (~mask);
    return !write_register(AM1805_STATUS_REG, reg);
}

/** Set/Reset Control1 Register
 *  mask list
 *   - default: 0xFF all status
 *   - AM1805_CONTROL1_1224_MASK
 *   - AM1805_CONTROL1_RSP_MASK
 *   - AM1805_CONTROL1_ARST_MASK
 *   - AM1805_CONTROL1_PWR2_MASK
 *   - AM1805_CONTROL1_WRTC_MASK
 */
uint8_t AM1805::get_control1(uint8_t mask)
{
    return read_register(AM1805_CONTROL1_REG, mask) ? 1 : 0;
}
bool AM1805::set_control1(uint8_t mask, uint8_t value)
{
    uint8_t reg = read_register(AM1805_CONTROL1_REG);
    if(value)
    {
        reg |= mask;
    }
    else
    {
        reg &= ~mask;
    }
    return !write_register(AM1805_CONTROL1_REG, reg);
}

/** Get/Set Control2 Register
 *  mask list
 *   - AM1805_CONTROL2_RS1E_MASK
 *   - AM1805_CONTROL2_OUT2S_MASK
 *   - AM1805_CONTROL2_OUT1S_MASK
 */
uint8_t AM1805::get_control2(uint8_t mask)
{
    uint8_t reg = read_register(AM1805_CONTROL2_REG, mask);
    if(mask == AM1805_CONTROL2_RS1E_MASK)
    {
        reg >>= 5;
    }
    else if(mask == AM1805_CONTROL2_OUT2S_MASK)
    {
        reg >>= 2;
    }
    return reg;
}

bool AM1805::set_control2(uint8_t mask, uint8_t value)
{
    uint8_t reg = read_register(AM1805_CONTROL2_REG) & (~mask);
    if(value)
    {
        if(mask == AM1805_CONTROL2_RS1E_MASK)
        {
            reg |= (value << 5);
        }
        else if(mask == AM1805_CONTROL2_OUT2S_MASK)
        {
            reg |= (value << 2);
        }
        else
        {
            reg |= value;
        }
    }
    return !write_register(AM1805_CONTROL2_REG, reg);
}

/** Get/Set Interrupt Register
 *  mask list
 *   - AM1805_INTERRUPT_CEB_MASK
 *   - AM1805_INTERRUPT_IM_MASK
 *   - AM1805_INTERRUPT_BLIE_MASK
 *   - AM1805_INTERRUPT_TIE_MASK
 *   - AM1805_INTERRUPT_AIE_MASK
 *   - AM1805_INTERRUPT_EX2E_MASK
 *   - AM1805_INTERRUPT_EX1E_MASK
 */
uint8_t AM1805::get_interrupt(uint8_t mask)
{
    uint8_t reg = read_register(AM1805_INT_MASK_REG, mask);
    if(mask == AM1805_INTERRUPT_IM_MASK)
    {
        return (reg >> 5);
    }
    return reg ? 1 : 0;

}

bool AM1805::set_interrupt(uint8_t mask, uint8_t value)
{
    uint8_t reg = read_register(AM1805_INT_MASK_REG) & (~mask);
    if(value)
    {
        if(mask == AM1805_INTERRUPT_IM_MASK)
        {
            reg |= (value << 5);
        }
        else
        {
            reg |= mask;
        }
    }
    return !write_register(AM1805_INT_MASK_REG, reg);
}

/** Get/Set Sleep Register
 *  mask list
 *   - AM1805_SLEEP_CONTROL_SLP_MASK
 *   - AM1805_SLEEP_CONTROL_SLRES_MASK
 *   - AM1805_SLEEP_CONTROL_EX2P_MASK
 *   - AM1805_SLEEP_CONTROL_EX1P_MASK
 *   - AM1805_SLEEP_CONTROL_SLST_MASK
 *   - AM1805_SLEEP_CONTROL_SLTO_MASK
 */
uint8_t AM1805::get_sleep(uint8_t mask)
{
    uint8_t reg = read_register(AM1805_SLEEP_CONTROL_REG, mask);
    if(mask == AM1805_SLEEP_CONTROL_SLTO_MASK)
    {
        return reg;
    }
    return reg ? 1 : 0;

}

bool AM1805::set_sleep(uint8_t mask, uint8_t value)
{
    uint8_t reg = read_register(AM1805_SLEEP_CONTROL_REG) & (~mask);
    if(value)
    {
        if(mask == AM1805_SLEEP_CONTROL_SLTO_MASK)
        {
            reg |= value;
        }
        else
        {
            reg |= mask;
        }
    }
    return !write_register(AM1805_SLEEP_CONTROL_REG, reg);
}

/** Get/Set Timer Control Register (Countdown Control)
 *  mask list
 *   - AM1805_TIMER_CONTROL_TE_MASK
 *   - AM1805_TIMER_CONTROL_TM_MASK
 *   - AM1805_TIMER_CONTROL_TRPT_MASK
 *   - AM1805_TIMER_CONTROL_RPT_MASK
 *   - AM1805_TIMER_CONTROL_TFS_MASK
 */
uint8_t AM1805::get_timer_control(uint8_t mask)
{
    uint8_t reg = read_register(AM1805_TIMER_CONTROL_REG, mask);
    if(mask == AM1805_TIMER_CONTROL_RPT_MASK)
    {
        return reg >> 2;
    }
    else if(mask == AM1805_TIMER_CONTROL_TFS_MASK)
    {
        return reg;
    }
    return reg ? 1 : 0;
}

bool AM1805::set_timer_control(uint8_t mask, uint8_t value)
{
    uint8_t reg = read_register(AM1805_TIMER_CONTROL_REG) & (~mask);
    if(value)
    {
        if(mask == AM1805_TIMER_CONTROL_RPT_MASK)
        {
            reg |= (value >> 2);
        }
        else if(mask == AM1805_TIMER_CONTROL_TFS_MASK)
        {
            reg |= value;
        }
        else
        {
            reg |= mask;
        }
    }
    return !write_register(AM1805_TIMER_CONTROL_REG, reg);
}

/** Get/Set Outcontrol Register
 *  mask list
 *   - AM1805_OCTRL_WDBM_MASK
 *   - AM1805_OCTRL_EXBM_MASK
 *   - AM1805_OCTRL_WDDS_MASK
 *   - AM1805_OCTRL_EXDS_MASK
 *   - AM1805_OCTRL_RSEN_MASK
 *   - AM1805_OCTRL_O4EN_MASK
 *   - AM1805_OCTRL_O3EN_MASK
 *   - AM1805_OCTRL_O1EN_MASK
 */
uint8_t AM1805::get_out_control(uint8_t mask)
{
    return read_register(AM1805_OCTRL_REG, mask) ? 1 : 0;
}

bool AM1805::set_out_control(uint8_t mask, uint8_t value)
{
    uint8_t reg = read_register(AM1805_OCTRL_REG);
    if(value)
    {
        reg |= mask;
    }
    else
    {
        reg &= ~mask;
    }
    return !write_register(AM1805_OCTRL_REG, reg);
}

/** Get/Set OSC Control Register
 *  mask list
 *   - AM1805_OSC_CONTROL_OSEL_MASK
 *   - AM1805_OSC_CONTROL_ACAL_MASK
 *   - AM1805_OSC_CONTROL_AOS_MASK
 *   - AM1805_OSC_CONTROL_FOS_MASK
 *   - AM1805_OSC_CONTROL_PWGT_MASK
 *   - AM1805_OSC_CONTROL_OFIE_MASK
 *   - AM1805_OSC_CONTROL_ACIE_MASK
 */
uint8_t AM1805::get_osc_control(uint8_t mask)
{
    uint8_t reg = read_register(AM1805_OSC_CONTROL_REG, mask);
    if(mask == AM1805_OSC_CONTROL_ACAL_MASK)
    {
        return (reg >> 5);
    }
    return reg ? 1 : 0;
}

bool AM1805::set_osc_control(uint8_t mask, uint8_t value)
{
    uint8_t reg = read_register(AM1805_OSC_CONTROL_REG) & (~mask);
    if(value)
    {
        if(mask == AM1805_OSC_CONTROL_ACAL_MASK)
        {
            reg |= (value << 5);
        }
        else
        {
            reg |= mask;
        }
    }
    return !write_register(AM1805_OSC_CONTROL_REG, reg);
}

/** Get/Set WDT Register
 *  mask list
 *   - AM1805_WDT_REGISTER_WDS_MASK
 *   - AM1805_WDT_REGISTER_BMB_MASK
 *   - AM1805_WDT_REGISTER_WRB_MASK
 */
uint8_t AM1805::get_wdt(uint8_t mask)
{
    uint8_t reg = read_register(AM1805_WDT_REG, mask);
    if(mask == AM1805_WDT_REGISTER_WDS_MASK)
    {
        return (reg >> 7);
    }
    else if(mask == AM1805_WDT_REGISTER_BMB_MASK)
    {
        return (reg >> 2);
    }
    else //if(mask == AM1805_WDT_REGISTER_WRB_MASK)
    {
        return reg;
    }
}

bool AM1805::set_wdt(uint8_t mask, uint8_t value)
{
    uint8_t reg = read_register(AM1805_WDT_REG) & (~mask);
    if(value)
    {
        if(mask == AM1805_WDT_REGISTER_WDS_MASK)
        {
            reg |= (value << AM1805_WDT_REGISTER_WDS_SHIFT);
        }
        else if(mask == AM1805_WDT_REGISTER_BMB_MASK)
        {
            reg |= (value << AM1805_WDT_REGISTER_BMB_SHIFT);
        }
        else //if(mask == AM1805_WDT_REGISTER_WRB_MASK)
        {
            reg |= (value << AM1805_WDT_REGISTER_WRB_SHIFT);
        }
    }
    _wdt_reg = reg;
    return !write_register(AM1805_WDT_REG, reg);
}

bool AM1805::configure_wdt(bool reset, int cycles, int frequency)
{
    uint8_t reg = 0x00;

    reg |= (reset << AM1805_WDT_REGISTER_WDS_SHIFT) & AM1805_WDT_REGISTER_WDS_MASK;
    reg |= (cycles << AM1805_WDT_REGISTER_BMB_SHIFT) & AM1805_WDT_REGISTER_BMB_MASK;
    reg |= (frequency << AM1805_WDT_REGISTER_WRB_SHIFT) & AM1805_WDT_REGISTER_WRB_MASK;

    _wdt_reg = reg;

    return !write_register(AM1805_WDT_REG, _wdt_reg);
}

bool AM1805::reset_wdt()
{
    if(_wdi_pin != AM1805_PIN_INVALID)
    {
        // reset watchdog timer by change the WDI level
        digitalWriteFast(_wdi_pin, !HAL_GPIO_Read(_wdi_pin));
        return true;
    }
    else
    {
        // reset watchdog timer by write data to register
        return !write_register(AM1805_WDT_REG, _wdt_reg);
    }
}

bool AM1805::enable_wdt(uint8_t seconds)
{
    int freq, cycle;
    if(seconds > 31)
    {
        freq = AM1805_WDT_REGISTER_WRB_QUARTER_HZ;
        cycle = seconds / 4;
    }
    else
    {
        freq = AM1805_WDT_REGISTER_WRB_ONE_HZ;
        cycle = seconds;
    }
    clean_status(AM1805_STATUS_WDT_MASK);
    return !configure_wdt(true, cycle, freq);
}

bool AM1805::disable_wdt()
{
    _wdt_reg = 0x00;
    return !write_register(AM1805_WDT_REG, _wdt_reg);
}

void AM1805::sleep_init()
{
    clean_status(AM1805_STATUS_WDT_MASK);
    clean_status(AM1805_STATUS_TIM_MASK);
    clean_status(AM1805_STATUS_ALM_MASK);
    clean_status(AM1805_STATUS_EX2_MASK);
    clean_status(AM1805_STATUS_EX1_MASK);
    set_sleep(AM1805_SLEEP_CONTROL_SLST_MASK, 0);
}

void AM1805::sleep_exit()
{
    set_interrupt(AM1805_INTERRUPT_EX1E_MASK, 1);
    set_sleep(AM1805_SLEEP_CONTROL_EX1P_MASK, 1); // rising edge trigger interrupt
}

/**
 * @brief User app sleep for seconds
 *
 * @details
 *  - input seconds need to power off
 *  - set AM1805 wakeup trigger by countdown timer
 *  - set AM1805 wakeup trigger by EXTI
 *  - set AM1805 to to sleep mode
 *
 * @param[in] seconds : sleep seconds, range is [1, 255]
 * @param[in] enable_exit : enable trigger external interrupt
 */
void AM1805::sleep_seconds(uint8_t seconds, bool enable_exit)
{
    do
    {
        sleep_init();

        // set countdonwn timer: interrupt-pulse, single mode, frequency 1Hz
        set_timer_control(AM1805_TIMER_CONTROL_TE_MASK, 0);
        set_timer_control(AM1805_TIMER_CONTROL_TM_MASK, 0);
        set_timer_control(AM1805_TIMER_CONTROL_TRPT_MASK, 0);
        set_timer_control(AM1805_TIMER_CONTROL_TFS_MASK, 2);

        // set countdown timer:
        write_register(AM1805_TIMER_REG, seconds);
        set_interrupt(AM1805_INTERRUPT_TIE_MASK, 1);
        set_timer_control(AM1805_TIMER_CONTROL_TE_MASK, 1);
        Log.info("sleep for %d seconds", seconds);

        // enable external interrupt trigger
        if(enable_exit)
        {
            sleep_exit();
        }
    } while(!enter_sleep_mode());
}

/**
 * @brief User app sleep for minutes
 *
 * @details
 *  - input minutes need to power off
 *  - set AM1805 wakeup trigger by countdown timer
 *  - set AM1805 wakeup trigger by EXTI
 *  - set AM1805 to to sleep mode
 *
 * @param[in] minutes : sleep minutes, range is [1, 255]
 * @param[in] enable_exit : enable trigger external interrupt
 */
void AM1805::sleep_minutes(uint8_t minutes, bool enable_exit)
{
    do
    {
        sleep_init();

        // set countdonwn timer: interrupt-pulse, single mode, frequency 1/60Hz
        set_timer_control(0xFF, 0);
        set_timer_control(AM1805_TIMER_CONTROL_TFS_MASK, 3);

        // set countdown timer:
        write_register(AM1805_TIMER_REG, minutes);
        set_interrupt(AM1805_INTERRUPT_TIE_MASK, 1);
        set_timer_control(AM1805_TIMER_CONTROL_TE_MASK, 1);
        Log.info("sleep for %d minutes", minutes);

        // enable external interrupt trigger
        if(enable_exit)
        {
            sleep_exit();
        }
    } while(!enter_sleep_mode());
}

/**
 * @brief User app sleep until the setting datetime
 *
 * @details
 *  - input the wakeup time
 *  - set AM1805 wakeup trigger by date alarm
 *  - set AM1805 wakeup trigger by EXTI
 *  - set AM1805 to to sleep mode
 *
 * @param[in] hour : wake datetime of hour
 * @param[in] minute : wake datetime of minute
 * @param[in] second : wake datetime of second
 * @param[in] enable_exit : enable trigger external interrupt
 */
void AM1805::sleep_until(uint8_t hour, uint8_t minute, uint8_t second, bool enable_exit)
{
    am1805_alarm_t alarm = {0};
    alarm.hour = hour;
    alarm.minute = minute;
    alarm.second = second;
    alarm.repeat = AM1805_ALARM_REPEAT_DAY;
    do
    {
        sleep_init();

        // set datetime alarm
        set_alarm(&alarm);
        set_interrupt(AM1805_INTERRUPT_AIE_MASK, 1);
        Log.info("sleep until %02d:%02d:%02d", hour, minute, second);

        // enable external interrupt trigger
        if(enable_exit)
        {
            sleep_exit();
        }
    } while(!enter_sleep_mode());
}

/**
 * @brief User app sleep forever
 *
 * @details
 *  - set AM1805 wakeup trigger by EXTI
 *  - set AM1805 to to sleep mode
 *
 * @param[in] enable_exit : enable trigger external interrupt
 */
void AM1805::sleep_forever(bool enable_exit)
{
    do
    {
        sleep_init();

        if(enable_exit)
        {
            sleep_exit();
        }
        Log.info("sleep forever");
    } while(!enter_sleep_mode());
}


bool AM1805::enter_sleep_mode()
{
    set_control2(AM1805_CONTROL2_OUT2S_MASK, 6);
    set_sleep(AM1805_SLEEP_CONTROL_SLTO_MASK, 1);
    set_sleep(AM1805_SLEEP_CONTROL_SLP_MASK, 1);
    int slp = get_sleep(AM1805_SLEEP_CONTROL_SLP_MASK);
    Log.info("SLP=%d", slp);
    delay(100);
    return slp;
}
