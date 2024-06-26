/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
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

/** @file
*
* Defines peripherals available for use on CYW9M2BASE-43012BT
*
*/

#pragma once

/** \addtogroup Platfrom config - Peripherals pin configuration
*   \ingroup HardwareDrivers
*/
/*! @{ */

/******************************************************
 *                   Enumerations
 ******************************************************/

#include "wiced_bt_types.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_trace.h"

typedef enum
{
    // WICED_PLATFORM_LED_1,
    WICED_PLATFORM_LED_MAX
}wiced_platform_led_t;

#define HCI_UART_DEFAULT_BAUD   3000000   /* default baud rate is 3M, that is max supported baud rate on Mac OS */

/**
 * wiced_platform_transport_rx_data_handler
 *
 * Callback registered by the application to receive the incoming HCI UART data.
 *
 * @param[in] op_code   : operation code for the incoming HCI data (refer to hci_control_api.h)
 * @param[in] p_data    : Pointer to the received data for the op_code
 * @param[in] data_len  : length of the data pointed to by p_data in bytes
 */
typedef void (wiced_platform_transport_rx_data_handler)(uint16_t op_code, uint8_t *p_data, uint32_t data_len);

void debug_uart_enable(uint32_t baud_rate);
void wiced_platform_init(void);

/**
 * wiced_platform_i2c_init
 *
 * Initialize the I2C interface.
 */
void wiced_platform_i2c_init(void);

/**
 * wiced_platform_i2s_init
 *
 * Initialize the I2S interface.
 */
void wiced_platform_i2s_init(void);

/**
 * wiced_platform_transport_init
 *
 * Initialize the WICED HCI Transport interface.
 *
 * @param[in] p_rx_data_handler : user callback for incoming HCI data.
 *
 * @return  WICED_TRUE - success
 *          WICED_FALSE - fail
 */
wiced_bool_t wiced_platform_transport_init(wiced_platform_transport_rx_data_handler *p_rx_data_handler);

extern wiced_debug_uart_types_t wiced_debug_uart;
/* @} */
