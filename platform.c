/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#include "wiced_bt_dev.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_trace.h"
#include "wiced_platform.h"
#include "wiced_hal_i2c.h"

  #define cr_pad_config_adr0                             0x00320068
  #define cr_pad_config_adr1                             0x0032006c
  #define cr_pad_config_adr2                             0x00320070
  #define cr_pad_config_adr3                             0x00320074
  #define cr_pad_config_adr4                             0x00320078
  #define cr_pad_config_adr5                             0x0032007c
  #define cr_pad_config_adr6                             0x00320080
  #define cr_pad_fcn_ctl_adr0                            0x00320088
  #define cr_pad_fcn_ctl_adr1                            0x0032008c
  #define cr_pad_fcn_ctl_adr2                            0x00320090
  #define cr_pad_fcn_ctl_adr3                            0x003201a8
  #define cr_pad_fcn_ctl_adr4                            0x003201b8
  #define gpiodata_adr                                   0x0032c000
  #define gpiodir_adr                                    0x0032c400

#define NO_PUART_SUPPORT    1
#define REG32(address)  ( *(volatile UINT32*)(address) )

extern BOOL8 debuguart_enabled;
void debuguart_Init( UINT32 baud_rate );

void pinconfig_setup_m2base43012(void)
{
    /* cr_pad_config_adr0 [31:24] = 0x60. Set input disable for BT_GPIO3 */
    REG32(cr_pad_config_adr0) = (REG32(cr_pad_config_adr0) & 0x00ffffff) | (0x60 << 24);

    /* cr_pad_fcn_ctl_adr0 [15:12] = 0x2. Select BT_GPIO3 to DEBUG_TXD */
    REG32(cr_pad_fcn_ctl_adr0) = (REG32(cr_pad_fcn_ctl_adr0) & 0xffff0fff) | (0x2 << 12);
}

void debug_uart_enable(uint32_t baud_rate)
{
    pinconfig_setup_m2base43012();
    debuguart_enabled = TRUE;
    debuguart_Init(baud_rate);
}

void i2s_init_m2base43012(void)
{
    /* BT_PCM_IN cr_pad_config_adr4 [7:0] cr_pad_fcn_ctl_adr1 [15:12] */
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0xffffff00) | 0x00;
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xffff0fff) | (0x7 << 12);

    /* BT_PCM_OUT cr_pad_config_adr4 [15:8]	cr_pad_fcn_ctl_adr1 [19:16] */
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0xffff00ff) | (0x60 << 8);
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xfff0ffff) | (0x7 << 16);

    /* BT_PCM_SYNC cr_pad_config_adr4 [31:24]	cr_pad_fcn_ctl_adr1 [23:20] */
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0x00ffffff) | (0x60 << 24);
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xff0fffff) | (0x7 << 20);

    /* BT_PCM_CLK cr_pad_config_adr4 [23:16]	cr_pad_fcn_ctl_adr1 [27:24] */
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0xff00ffff) | (0x60 << 16);
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xf0ffffff) | (0x7 << 24);
}

void i2c_init_m2base43012(void)
{
    /* BT_GPIO5/SCL cr_pad_config_adr1 [15:8]	cr_pad_fcn_ctl_adr0 [23:20] */
    REG32(cr_pad_config_adr1) = (REG32(cr_pad_config_adr1) & 0xffff00ff) | (0x02 << 8);
    REG32(cr_pad_fcn_ctl_adr0) = (REG32(cr_pad_fcn_ctl_adr0) & 0xff0fffff) | (0xf << 20);

    /* BT_GPIO4/SDA cr_pad_config_adr1 [7:0]	cr_pad_fcn_ctl_adr0 [19:16] */
    REG32(cr_pad_config_adr1) = (REG32(cr_pad_config_adr1) & 0xffffff00) | 0x02;
    REG32(cr_pad_fcn_ctl_adr0) = (REG32(cr_pad_fcn_ctl_adr0) & 0xfff0ffff) | (0xf << 16);

}

#if 0
/* GPIO debug on BT_GPIO2 */
void bt_gpio_init(void)
{
    /* BT_GPIO2 cr_pad_config_adr0 [23:16]	cr_pad_fcn_ctl_adr0 [11:8] */
    REG32(cr_pad_config_adr0) = (REG32(cr_pad_config_adr0) & 0xff00ffff) | (0x60 << 16);
    REG32(cr_pad_fcn_ctl_adr0) = (REG32(cr_pad_fcn_ctl_adr0) & 0xfffff0ff) | (0x0 << 8);
}

void bt_gpio_set(uint8_t gpio, uint8_t value)
{
    REG32(gpiodir_adr) |= 0x1 << gpio;

    if (value)
    {
        REG32(gpiodata_adr + ((0x1 << gpio) << 2)) |= 0x1 << gpio;
    }
    else
    {
        REG32(gpiodata_adr + ((0x1 << gpio) << 2)) &= ~(0x1 << gpio);
    }
}
#endif

void platform_init_43012c0(void)
{
    i2s_init_m2base43012();

    wiced_hal_i2c_init();
    i2c_init_m2base43012();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);
}

