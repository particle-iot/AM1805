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

#pragma once

#include "Particle.h"
#include "stdint.h"
#include "time.h"

// Device Address
#define AM1805_I2C_ADDR             0x69

#define AM1805_I2C_TIMEOUT_MS       (2)
#define AM1805_PIN_INVALID          (PIN_INVALID)

// Register Address
#define AM1805_HUNDRETH_REG         0x00
#define AM1805_SECOND_REG           0x01
#define AM1805_MINUTE_REG           0x02
#define AM1805_HOUR_REG             0x03
#define AM1805_DATE_REG             0x04
#define AM1805_MONTH_REG            0x05
#define AM1805_YEAR_REG             0x06
#define AM1805_WEEKDAY_REG          0x07
#define AM1805_HUNDRETH_ALARM_REG   0x08
#define AM1805_SECOND_ALARM_REG     0x09
#define AM1805_MINUTE_ALARM_REG     0x0A
#define AM1805_HOUR_ALARM_REG       0x0B
#define AM1805_DATE_ALARM_REG       0x0C
#define AM1805_MONTH_ALARM_REG      0x0D
#define AM1805_WEEKDAY_ALARM_REG    0x0E
#define AM1805_STATUS_REG           0x0F
#define AM1805_CONTROL1_REG         0x10
#define AM1805_CONTROL2_REG         0x11
#define AM1805_INT_MASK_REG         0x12
#define AM1805_SQW_REG              0x13
#define AM1805_CAL_XT_REG           0x14
#define AM1805_CAL_RC_HI_REG        0x15
#define AM1805_CAL_RC_LOW_REG       0x16
#define AM1805_SLEEP_CONTROL_REG    0x17
#define AM1805_TIMER_CONTROL_REG    0x18
#define AM1805_TIMER_REG            0x19
#define AM1805_TIMER_INITIAL_REG    0x1A
#define AM1805_WDT_REG              0x1B
#define AM1805_OSC_CONTROL_REG      0x1C
#define AM1805_OSC_STATUS_REG       0x1D
#define AM1805_CONFIG_KEY_REG       0x1F
#define AM1805_TRICKLE_REG          0x20
#define AM1805_BREF_CONTROL_REG     0x21
#define AM1805_AFCTRL_REG           0x26
#define AM1805_BATMODE_REG          0x27
#define AM1805_ID0_REG              0x28
#define AM1805_ID1_REG              0x29
#define AM1805_ID2_REG              0x2A
#define AM1805_ID3_REG              0x2B
#define AM1805_ID4_REG              0x2C
#define AM1805_ID5_REG              0x2D
#define AM1805_ID6_REG              0x2E
#define AM1805_ASTAT_REG            0X2F
#define AM1805_OCTRL_REG            0x30
#define AM1805_XTENSION_ADDR_REG    0x3F

// Datatime Mask
#define AM1805_TENTH_MASK           0xF0
#define AM1805_HUNDRETH_MASK        0x0F
#define AM1805_SECOND_MASK          0x7F
#define AM1805_MINUTE_MASK          0x7F
#define AM1805_HOUR_24_MASK         0x3F
#define AM1805_HOUR_12_MASK         0x1F
#define AM1805_AM_PM_MASK           0x20
#define AM1805_DATE_MASK            0x3F
#define AM1805_MONTH_MASK           0x1F
#define AM1805_YEAR_MASK            0xFF
#define AM1805_DAY_MASK             0x07
#define AM1805_TENTH_ALARM_MASK     0xF0
#define AM1805_HUNDRETH_ALARM_MASK  0xF0
#define AM1805_SECOND_ALARM_MASK    0x7F
#define AM1805_MINUTE_ALARM_MASK    0x7F
#define AM1805_HOUR_24_ALARM_MASK   0x3F
#define AM1805_HOUR_12_ALARM_MASK   0x1F
#define AM1805_DATE_ALARM_MASK      0x3F
#define AM1805_MONTH_ALARM_MASK     0x1F
#define AM1805_WEEKDAY_ALARM_MASK   0x07

//Status Mask
#define AM1805_STATUS_CB_MASK       0x80
#define AM1805_STATUS_BAT_MASK      0x40
#define AM1805_STATUS_WDT_MASK      0x20
#define AM1805_STATUS_BL_MASK       0x10
#define AM1805_STATUS_TIM_MASK      0x08
#define AM1805_STATUS_ALM_MASK      0x04
#define AM1805_STATUS_EX2_MASK      0x02
#define AM1805_STATUS_EX1_MASK      0x01

//Control1 Mask
#define AM1805_CONTROL1_1224_MASK   0x40
#define AM1805_CONTROL1_RSP_MASK    0x08
#define AM1805_CONTROL1_ARST_MASK   0x04
#define AM1805_CONTROL1_PWR2_MASK   0x02
#define AM1805_CONTROL1_WRTC_MASK   0x01

//Control2 Mask
#define AM1805_CONTROL2_RS1E_MASK   0x20
#define AM1805_CONTROL2_OUT2S_MASK  0x1C
#define AM1805_CONTROL2_OUT1S_MASK  0x03

//Interrupt Mask
#define AM1805_INTERRUPT_CEB_MASK   0x80
#define AM1805_INTERRUPT_IM_MASK    0x60
#define AM1805_INTERRUPT_BLIE_MASK  0x10
#define AM1805_INTERRUPT_TIE_MASK   0x08
#define AM1805_INTERRUPT_AIE_MASK   0x04
#define AM1805_INTERRUPT_EX2E_MASK  0x02
#define AM1805_INTERRUPT_EX1E_MASK  0x01

//Sleep Control Mask
#define AM1805_SLEEP_CONTROL_SLP_MASK   0x80
#define AM1805_SLEEP_CONTROL_SLRES_MASK 0x40  //When 1, assert nRST low when the Power Control SM is in the SLEEP state.
#define AM1805_SLEEP_CONTROL_EX2P_MASK  0x20  //When 1, the external interrupt XT2 will trigger on a rising edge of the WDI pin.
#define AM1805_SLEEP_CONTROL_EX1P_MASK  0x10  //When 1, the external interrupt XT1 will trigger on a rising edge of the EXTI pin.
#define AM1805_SLEEP_CONTROL_SLST_MASK  0x08  //Set when the AM18X5 enters Sleep Mode
#define AM1805_SLEEP_CONTROL_SLTO_MASK  0x07  //The number of 7.8 ms periods after SLP is set until the Power Control SM goes into the SLEEP state.

// Timer Register and Countdown Control
#define AM1805_TIMER_CONTROL_TE_MASK    0x80
#define AM1805_TIMER_CONTROL_TM_MASK    0x40
#define AM1805_TIMER_CONTROL_TRPT_MASK  0x20
#define AM1805_TIMER_CONTROL_RPT_MASK   0x1C
#define AM1805_TIMER_CONTROL_TFS_MASK   0x03

//Outcontrol Mask
#define AM1805_OCTRL_WDBM_MASK          0x80
#define AM1805_OCTRL_EXBM_MASK          0x40
#define AM1805_OCTRL_WDDS_MASK          0x20
#define AM1805_OCTRL_EXDS_MASK          0x10
#define AM1805_OCTRL_RSEN_MASK          0x08
#define AM1805_OCTRL_O4EN_MASK          0x04
#define AM1805_OCTRL_O3EN_MASK          0x02
#define AM1805_OCTRL_O1EN_MASK          0x01

//OSC Control Mask
#define AM1805_OSC_CONTROL_OSEL_MASK    0x80
#define AM1805_OSC_CONTROL_ACAL_MASK    0x60
#define AM1805_OSC_CONTROL_AOS_MASK     0x10
#define AM1805_OSC_CONTROL_FOS_MASK     0x08
#define AM1805_OSC_CONTROL_PWGT_MASK    0x04
#define AM1805_OSC_CONTROL_OFIE_MASK    0x02
#define AM1805_OSC_CONTROL_ACIE_MASK    0x01

//WDT Register Mask
#define AM1805_WDT_REGISTER_WDS_MASK    0x80
#define AM1805_WDT_REGISTER_BMB_MASK    0x7C
#define AM1805_WDT_REGISTER_WRB_MASK    0x03

//WDT Register Shifts
#define AM1805_WDT_REGISTER_WDS_SHIFT   (7)
#define AM1805_WDT_REGISTER_BMB_SHIFT   (2)
#define AM1805_WDT_REGISTER_WRB_SHIFT   (0)

//WDT WRB Field Values (frequency to countdown WDT)
#define AM1805_WDT_REGISTER_WRB_QUARTER_HZ   (0x03)
#define AM1805_WDT_REGISTER_WRB_ONE_HZ       (0x02)
#define AM1805_WDT_REGISTER_WRB_FOUR_HZ      (0x01)
#define AM1805_WDT_REGISTER_WRB_SIXTEEN_HZ   (0x00)


typedef enum
{
    AM1805_ALARM_REPEAT_DISABLE = 0,
    AM1805_ALARM_REPEAT_YEAR    = 1,
    AM1805_ALARM_REPEAT_MONTH   = 2,
    AM1805_ALARM_REPEAT_WEEK    = 3,
    AM1805_ALARM_REPEAT_DAY     = 4,
    AM1805_ALARM_REPEAT_HOUR    = 5,
    AM1805_ALARM_REPEAT_MINUTE  = 6,
} am1805_alarm_repeat_t;

typedef struct
{
    uint8_t repeat; // am1805_alarm_repeat_t
    uint8_t month;
    uint8_t weekday;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} am1805_alarm_t;

class AM1805
{
public:
    AM1805(pin_t wdi_pin = AM1805_PIN_INVALID, TwoWire &wire = Wire, uint8_t address = AM1805_I2C_ADDR);
    void begin();

    /**
     * @brief Read register data
     * @param[in] rtc_register : register address
     * @param[in] mask : filter mask, default is 0xFF
     * @return value that read from register
     */
    uint8_t read_register(const uint8_t rtc_register, const uint8_t mask = 0xFF);
    /**
     * @brief Write register data
     * @param[in] rtc_register : register address
     * @param[in] data : the value that will be written to register
     * @return write result, success or not
     */
    uint8_t write_register(const uint8_t rtc_register, const uint8_t data);

    /**
     * @brief Get datetime from AM1805
     * @return datetime in format struct tm
     */
    struct tm get_datetime(void);
    /**
     * @brief Set datetime to AM1805
     * @param[in] t : datetime in struct tm format
     * @return set result: true - success, false - failed
     */
    bool set_datetime(const struct tm *t);

    /**
     * @brief Get time of seconds from AM1805
     * @return datetime of seconds
     */
    time_t get_seconds(void);
    /**
     * @brief Set time of seconds to AM1805
     * @param[in] seconds : datetime of seconds
     * @return set result: true - success, false - failed
     */
    bool set_seconds(time_t seconds);

    /**
     * @brief Get alarm setting frorm AM1805
     * @return alarm settings in AM1805
     */
    am1805_alarm_t get_alarm(void);
    /**
     * @brief Set alarm settings to AM1805
     * @details  convert to alarm setting to register value and write to AM1805
     * @param[in] alarm : alarm settings struct
     * @return set result: true - success, false - failed
     */
    bool set_alarm(const am1805_alarm_t *alarm);

    /**
     * @brief Get status from AM1805
     * @param[in] mask : filter mask, default is 0xFF
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
     * @return value that read from register
     */
    uint8_t get_status(uint8_t mask = 0xFF);
    /**
     * @brief Clean status in AM1805
     * @param[in] mask : filter mask, default is 0xFF
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
     * @return set result: true - success, false - failed
     */
    bool clean_status(uint8_t mask = 0xFF);

    /**
     * @brief Get Control1 register
     * @param[in] mask : filter mask
     *  mask list
     *   - default: 0xFF all status
     *   - AM1805_CONTROL1_1224_MASK
     *   - AM1805_CONTROL1_RSP_MASK
     *   - AM1805_CONTROL1_ARST_MASK
     *   - AM1805_CONTROL1_PWR2_MASK
     *   - AM1805_CONTROL1_WRTC_MASK
     * @return value that read from register
     */
    uint8_t get_control1(uint8_t mask);
    /**
     * @brief Set Control1 register
     * @param[in] mask : filter mask
     *  mask list
     *   - default: 0xFF all status
     *   - AM1805_CONTROL1_1224_MASK
     *   - AM1805_CONTROL1_RSP_MASK
     *   - AM1805_CONTROL1_ARST_MASK
     *   - AM1805_CONTROL1_PWR2_MASK
     *   - AM1805_CONTROL1_WRTC_MASK
     * @param[in] value : new data of the register
     * @return set result: true - success, false - failed
     */
    bool set_control1(uint8_t mask, uint8_t value);

    /**
     * @brief Get Control2 register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_CONTROL2_RS1E_MASK
     *   - AM1805_CONTROL2_OUT2S_MASK
     *   - AM1805_CONTROL2_OUT1S_MASK
     * @param[in] value : new data of the register
     * @return value that read from register
     */
    uint8_t get_control2(uint8_t mask);
    /**
     * @brief Set Control2 register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_CONTROL2_RS1E_MASK
     *   - AM1805_CONTROL2_OUT2S_MASK
     *   - AM1805_CONTROL2_OUT1S_MASK
     * @param[in] value : new data of the register
     * @return set result: true - success, false - failed
     */
    bool set_control2(uint8_t mask, uint8_t value);

    /**
     * @brief Get Interrupt register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_INTERRUPT_CEB_MASK
     *   - AM1805_INTERRUPT_IM_MASK
     *   - AM1805_INTERRUPT_BLIE_MASK
     *   - AM1805_INTERRUPT_TIE_MASK
     *   - AM1805_INTERRUPT_AIE_MASK
     *   - AM1805_INTERRUPT_EX2E_MASK
     *   - AM1805_INTERRUPT_EX1E_MASK
     * @return value that read from register
     */
    uint8_t get_interrupt(uint8_t mask);
    /**
     * @brief Set Interrupt register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_INTERRUPT_CEB_MASK
     *   - AM1805_INTERRUPT_IM_MASK
     *   - AM1805_INTERRUPT_BLIE_MASK
     *   - AM1805_INTERRUPT_TIE_MASK
     *   - AM1805_INTERRUPT_AIE_MASK
     *   - AM1805_INTERRUPT_EX2E_MASK
     *   - AM1805_INTERRUPT_EX1E_MASK
     * @param[in] value : new data of the register
     * @return set result: true - success, false - failed
     */
    bool set_interrupt(uint8_t mask, uint8_t value);

    /**
     * @brief Get Sleep register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_SLEEP_CONTROL_SLP_MASK
     *   - AM1805_SLEEP_CONTROL_SLRES_MASK
     *   - AM1805_SLEEP_CONTROL_EX2P_MASK
     *   - AM1805_SLEEP_CONTROL_EX1P_MASK
     *   - AM1805_SLEEP_CONTROL_SLST_MASK
     *   - AM1805_SLEEP_CONTROL_SLTO_MASK
     * @return value that read from register
     */
    uint8_t get_sleep(uint8_t mask);
    /**
     * @brief Set Sleep register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_SLEEP_CONTROL_SLP_MASK
     *   - AM1805_SLEEP_CONTROL_SLRES_MASK
     *   - AM1805_SLEEP_CONTROL_EX2P_MASK
     *   - AM1805_SLEEP_CONTROL_EX1P_MASK
     *   - AM1805_SLEEP_CONTROL_SLST_MASK
     *   - AM1805_SLEEP_CONTROL_SLTO_MASK
     * @param[in] value : new data of the register
     * @return set result: true - success, false - failed
     */
    bool set_sleep(uint8_t mask, uint8_t value);

    /**
     * @brief Get Timer Control register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_TIMER_CONTROL_TE_MASK
     *   - AM1805_TIMER_CONTROL_TM_MASK
     *   - AM1805_TIMER_CONTROL_TRPT_MASK
     *   - AM1805_TIMER_CONTROL_RPT_MASK
     *   - AM1805_TIMER_CONTROL_TFS_MASK
     * @return value that read from register
     */
    uint8_t get_timer_control(uint8_t mask);
    /**
     * @brief Set Timer Control register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_TIMER_CONTROL_TE_MASK
     *   - AM1805_TIMER_CONTROL_TM_MASK
     *   - AM1805_TIMER_CONTROL_TRPT_MASK
     *   - AM1805_TIMER_CONTROL_RPT_MASK
     *   - AM1805_TIMER_CONTROL_TFS_MASK
     * @param[in] value : new data of the register
     * @return set result: true - success, false - failed
     */
    bool set_timer_control(uint8_t mask, uint8_t value);

    /**
     * @brief Get Out Control register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_OCTRL_WDBM_MASK
     *   - AM1805_OCTRL_EXBM_MASK
     *   - AM1805_OCTRL_WDDS_MASK
     *   - AM1805_OCTRL_EXDS_MASK
     *   - AM1805_OCTRL_RSEN_MASK
     *   - AM1805_OCTRL_O4EN_MASK
     *   - AM1805_OCTRL_O3EN_MASK
     *   - AM1805_OCTRL_O1EN_MASK
     * @return value that read from register
     */
    uint8_t get_out_control(uint8_t mask);
    /**
     * @brief Set Out Control register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_OCTRL_WDBM_MASK
     *   - AM1805_OCTRL_EXBM_MASK
     *   - AM1805_OCTRL_WDDS_MASK
     *   - AM1805_OCTRL_EXDS_MASK
     *   - AM1805_OCTRL_RSEN_MASK
     *   - AM1805_OCTRL_O4EN_MASK
     *   - AM1805_OCTRL_O3EN_MASK
     *   - AM1805_OCTRL_O1EN_MASK
     * @param[in] value : new data of the register
     * @return set result: true - success, false - failed
     */
    bool set_out_control(uint8_t mask, uint8_t value);

    /**
     * @brief Get OSC Control register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_OSC_CONTROL_OSEL_MASK
     *   - AM1805_OSC_CONTROL_ACAL_MASK
     *   - AM1805_OSC_CONTROL_AOS_MASK
     *   - AM1805_OSC_CONTROL_FOS_MASK
     *   - AM1805_OSC_CONTROL_PWGT_MASK
     *   - AM1805_OSC_CONTROL_OFIE_MASK
     *   - AM1805_OSC_CONTROL_ACIE_MASK
     * @return value that read from register
     */
    uint8_t get_osc_control(uint8_t mask);
    /**
     * @brief Set OSC Control register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_OSC_CONTROL_OSEL_MASK
     *   - AM1805_OSC_CONTROL_ACAL_MASK
     *   - AM1805_OSC_CONTROL_AOS_MASK
     *   - AM1805_OSC_CONTROL_FOS_MASK
     *   - AM1805_OSC_CONTROL_PWGT_MASK
     *   - AM1805_OSC_CONTROL_OFIE_MASK
     *   - AM1805_OSC_CONTROL_ACIE_MASK
     * @param[in] value : new data of the register
     * @return set result: true - success, false - failed
     */
    bool set_osc_control(uint8_t mask, uint8_t value);

    /**
     * @brief Get WDT register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_WDT_REGISTER_WDS_MASK
     *   - AM1805_WDT_REGISTER_BMB_MASK
     *   - AM1805_WDT_REGISTER_WRB_MASK
     * @return value that read from register
     */
    uint8_t get_wdt(uint8_t mask);
    /**
     * @brief Set WDT register
     * @param[in] mask : filter mask
     *  mask list
     *   - AM1805_WDT_REGISTER_WDS_MASK
     *   - AM1805_WDT_REGISTER_BMB_MASK
     *   - AM1805_WDT_REGISTER_WRB_MASK
     * @param[in] value : new data of the register
     * @return set result: true - success, false - failed
     */
    bool set_wdt(uint8_t mask, uint8_t value);

    /** Configure WDT
     */
    bool configure_wdt(bool reset, int cycles, int frequency);

    /** Reset WDT
     *  Resets WDT by writing the last configuration.
     */
    bool reset_wdt();

    /** Enable WDT
     *  Enable the watchdog by setting timeout seconds
     */
    bool enable_wdt(uint8_t seconds);

    /** Disable WDT
     *  Disable the watchdog by setting underlying register to 0x00.
     */
    bool disable_wdt();

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
    void sleep_seconds(uint8_t seconds, bool enable_exit = true);

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
    void sleep_minutes(uint8_t minutes, bool enable_exit = true);

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
    void sleep_until(uint8_t hour, uint8_t minute, uint8_t second, bool enable_exit = true);


    /**
     * @brief User app sleep forever
     *
     * @details
     *  - set AM1805 wakeup trigger by EXTI
     *  - set AM1805 to to sleep mode
     *
     * @param[in] enable_exit : enable trigger external interrupt
     */
    void sleep_forever(bool enable_exit = true);
    bool enter_sleep_mode();

private:
    TwoWire &_wire;
    uint8_t _address;
    uint8_t _wdt_reg;
    pin_t _wdi_pin = AM1805_PIN_INVALID;

    //convert func
    uint8_t dec2hex(uint8_t dec);
    uint8_t hex2dec(uint8_t hex);

    //sleep internal functions
    void sleep_init();
    void sleep_exit();

};
