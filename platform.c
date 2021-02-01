/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "wiced_bt_dev.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "wiced_hal_i2c.h"
#include "wiced_rtos.h"
#include "wiced_transport.h"
#include "hci_control_api.h"
#include "platform_mem.h"
#include "wiced_hal_nvram.h"

  #define cr_pad_config_adr0                             0x00320068
  #define cr_pad_config_adr1                             0x0032006c
  #define cr_pad_config_adr2                             0x00320070
  #define cr_pad_config_adr3                             0x00320074
  #define cr_pad_config_adr4                             0x00320078
  #define cr_pad_config_adr5                             0x0032007c
  #define cr_pad_config_adr6                             0x00320080
  #define cr_pad_config_adr7                             0x003200e4
  #define cr_pad_config_adr8                             0x00320110
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
extern wiced_bool_t rbg_ready(void);

typedef struct platform_virtual_nvram_t
{
    uint16_t                        data_length;
    struct platform_virtual_nvram_t *p_next;

    struct __attribute__((packed))
    {
        uint16_t                    vs_id;
        uint8_t                     data[0];
    } content;
} platform_virtual_nvram_t;

static struct
{
    struct
    {
        wiced_bool_t init;
        wiced_platform_transport_rx_data_handler *p_app_rx_data_handler;
    } transport;

    platform_virtual_nvram_t *p_virtual_nvram;
} platform_cb = {0};

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
#ifdef BT_I2S_ENABLE // BT_I2S
    // Mux to PCM_IN to I2S_DI and PCM_OUT to I2S_DO
    REG32(cr_pad_fcn_ctl_adr2 ) &= 0xF0F0FFFF;
    REG32(cr_pad_fcn_ctl_adr2 ) |= 0x01010000;  // PCM_IN, PCM_OUT

    // Mux PCM_CLK to I2S_CLK pin and PCM_SYNC to I2S_WS pin
    REG32(cr_pad_fcn_ctl_adr3 ) &= 0xFFFF00FF;
    REG32(cr_pad_fcn_ctl_adr3 ) |= 0x00001100;  // PCM_CLK, PCM_SYNC

    // Set I2S_DO as an output and I2S_DI as an input
    REG32(cr_pad_config_adr7 ) &= 0xFF00FF00;
    REG32(cr_pad_config_adr7 ) |= 0x00090088;

    // Set I2S_WS and I2S_CLK as outputs
    REG32(cr_pad_config_adr8 ) &= 0x8080FFFF;
    REG32(cr_pad_config_adr8 ) |= 0x88880000;
#else // BT_PCM
    /* 1. Set BT_PCM_IN as I2S_DI. */
    /*
     * 1.1. Set Control Pad: cr_pad_config_adr4 [7:0]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0xffffff00) | 0x09;
    /*
     * 1.2. Assign Function: cr_pad_fcn_ctl_adr1 [15:12]
     *      0: A_GPIO[3]
     *      1: PCM_IN
     *      2: PCM_IN
     *      3: HCLK
     *      4: nESMCS
     *      5: PHY_DEBUG_8
     *      6: MTU_DEBUG_3
     *      7: I2S_SSDI/MSDI
     */
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xffff0fff) | (0x07 << 12);

    /* 2. Set BT_PCM_OUT as I2S_DO. */
    /*
     * 2.1. Set Control Pad: cr_pad_config_adr4 [15:8]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0xffff00ff) | (0x88 << 8);
    /*
     * 2.2. Assign Function: cr_pad_fcn_ctl_adr1 [19:16]
     *      0: A_GPIO[2]
     *      1: PCM_OUT
     *      2: -
     *      3: LINK_IND
     *      4: -
     *      5: I2S_MSDO
     *      6: MTU_DEBUG_2
     *      7: I2S_SSDO
     */
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xfff0ffff) | (0x05 << 16);

    /* 3. Set BT_PCM_SYNC as I2S_WS. */
    /*
     * 3.1. Set Control Pad: cr_pad_config_adr4 [31:24]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0x00ffffff) | (0x88 << 24);
    /*
     * 3.2. Assign Function: cr_pad_fcn_ctl_adr1 [23:20]
     *      0: A_GPIO[1]
     *      1: PCM_SYNC
     *      2: PCM_SYNC
     *      3: HCLK
     *      4: INT_LPO
     *      5: I2S_MWS
     *      6: MTU_DEBUG_1
     *      7: I2S_SWS
     */
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xff0fffff) | (0x05 << 20);

    /* 4. Set BT_PCM_CLK as I2S_SCK. */
    /*
     * 4.1. Set Control Pad: cr_pad_config_adr4 [23:16]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0xff00ffff) | (0x88 << 16);
    /*
     * 4.2. Assign Function: cr_pad_fcn_ctl_adr1 [27:24]
     *      0: A_GPIO[0]
     *      1: PCM_CLK
     *      2: PCM_CLK
     *      3: SPI_INT(0)
     *      4: MTU_DEBUT_7
     *      5: I2S_MSCK
     *      6: MTU_DEBUG_0 or SPI_INT(1)
     *      7: I2S_SSCK
     */
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xf0ffffff) | (0x5 << 24);
#endif
}

void i2c_init_m2base43012(void)
{
    /* 1. Set BT_GPIO5 as I2C_SCL. */
    /*
     * 1.1. Set Control Pad: cr_pad_config_adr1 [15:8]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr1) = (REG32(cr_pad_config_adr1) & 0xffff00ff) | (0x02 << 8);
    /*
     * 1.2. Assign Function: cr_pad_fcn_ctl_adr0 [23:20]
     *      0: GPIO[5]
     *      1: HCLK
     *      2: -
     *      3: I2S_MSCK
     *      4: I2S_SSCK
     *      5: PHY_DEBUG_5
     *      6: wlan_clk_req
     *      7: CLK_REQ
     *      8: Super_Mux
     *      9: -
     *      10: -
     *      11: UART2_CTS_N
     *      12: BT_RX_ACTIVE
     *      13: -
     *      14: mia_debug[2]
     *      15: SCL
     */
    REG32(cr_pad_fcn_ctl_adr0) = (REG32(cr_pad_fcn_ctl_adr0) & 0xff0fffff) | (0xf << 20);

    /* 2. Set BT_GPIO4 as I2C_SDA. */
    /*
     * 2.1. Set Control Pad: cr_pad_config_adr1 [7:0]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr1) = (REG32(cr_pad_config_adr1) & 0xffffff00) | 0x02;
    /*
     * 2.2. Assign Function: ccr_pad_fcn_ctl_adr0 [19:16]
     *      0: GPIO[4]
     *      1: LINK_IND
     *      2: DEBUG_RXD
     *      3: I2S_MSDO
     *      4: I2S_SSDO
     *      5: PHY_DEBUG_4
     *      6: wlan_clk_req
     *      7: -
     *      8: Super_Mux
     *      9: CLASS1[1]
     *      10: -
     *      11: UART2_RTS_N
     *      12: coex_out[1]
     *      13: -
     *      14: mia_debug[1]
     *      15: SDA
     */
    REG32(cr_pad_fcn_ctl_adr0) = (REG32(cr_pad_fcn_ctl_adr0) & 0xfff0ffff) | (0xf << 16);
}

void i2c_init_bitbang(void)
{
    /* 1. Set BT_GPIO5 as I2C_SCL. */
    /*
     * 1.1. Set Control Pad: cr_pad_config_adr1 [15:8]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr1) = (REG32(cr_pad_config_adr1) & 0xffff00ff) | (0x02 << 8);
 //   REG32(cr_pad_config_adr1) = REG32(cr_pad_config_adr1) & 0xffff00ff;
    /*
     * 1.2. Assign Function: cr_pad_fcn_ctl_adr0 [23:20]
     *      0: GPIO[5]
     *      1: HCLK
     *      2: -
     *      3: I2S_MSCK
     *      4: I2S_SSCK
     *      5: PHY_DEBUG_5
     *      6: wlan_clk_req
     *      7: CLK_REQ
     *      8: Super_Mux
     *      9: -
     *      10: -
     *      11: UART2_CTS_N
     *      12: BT_RX_ACTIVE
     *      13: -
     *      14: mia_debug[2]
     *      15: SCL
     */
    REG32(cr_pad_fcn_ctl_adr0) = REG32(cr_pad_fcn_ctl_adr0) & 0xff0fffff;

    /* 2. Set BT_GPIO4 as I2C_SDA. */
    /*
     * 2.1. Set Control Pad: cr_pad_config_adr1 [7:0]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr1) = (REG32(cr_pad_config_adr1) & 0xffffff00) | 0x02;
 //   REG32(cr_pad_config_adr1) = REG32(cr_pad_config_adr1) & 0xffffff00;
    /*
     * 2.2. Assign Function: ccr_pad_fcn_ctl_adr0 [19:16]
     *      0: GPIO[4]
     *      1: LINK_IND
     *      2: DEBUG_RXD
     *      3: I2S_MSDO
     *      4: I2S_SSDO
     *      5: PHY_DEBUG_4
     *      6: wlan_clk_req
     *      7: -
     *      8: Super_Mux
     *      9: CLASS1[1]
     *      10: -
     *      11: UART2_RTS_N
     *      12: coex_out[1]
     *      13: -
     *      14: mia_debug[1]
     *      15: SDA
     */
    REG32(cr_pad_fcn_ctl_adr0) = REG32(cr_pad_fcn_ctl_adr0) & 0xfff0ffff;
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

/*
 * wiced_platform_warm_up_rbg200
 *
 * Warm up the Random Block Generator 200
 */
static void wiced_platform_warm_up_rbg200(void)
{
    /* Wait the till the RBG200 (Random Block Generator) has been warmed up. */
    while (1)
    {
        if (rbg_ready())
        {
            break;
        }
        else
        {
            wiced_rtos_delay_milliseconds(100, ALLOW_THREAD_TO_SLEEP);
        }
    }
}

/*
 * wiced_platform_warm_up
 *
 * Warm up the platform-specific modules
 */
static void wiced_platform_warm_up(void)
{
    wiced_platform_warm_up_rbg200();
}

void wiced_platform_init(void)
{
    wiced_platform_warm_up();

    platform_mem_init();
}

/**
 * wiced_platform_i2c_init
 *
 * Initialize the I2C interface.
 */
void wiced_platform_i2c_init(void)
{
    wiced_hal_i2c_init();
    i2c_init_m2base43012();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);
}

/**
 * wiced_platform_i2s_init
 *
 * Initialize the I2S interface.
 */
void wiced_platform_i2s_init(void)
{
    i2s_init_m2base43012();
}

/*
 * platform_transport_status_handler
 */
static void platform_transport_status_handler( wiced_transport_type_t type )
{
    wiced_transport_send_data(HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0);
}

static void platform_transport_rx_data_handler_push_nvram_data(uint8_t *p_data, uint32_t data_len)
{
    uint16_t vs_id;
    uint32_t payload_len;
    platform_virtual_nvram_t *p_new;
    wiced_result_t status;

    /* Check parameter. */
    if ((p_data == NULL) ||
        (data_len == 0))
    {
        return;
    }

    /* Parse information. */
    STREAM_TO_UINT16(vs_id, p_data);
    payload_len = data_len - sizeof(vs_id);

    wiced_platform_nvram_write(vs_id, payload_len, p_data, &status);
    (void) status;
}

/*
 * platform_transport_rx_data_handler
 */
static uint32_t platform_transport_rx_data_handler(uint8_t *p_buffer, uint32_t length)
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint32_t sample_rate = 16000;
    uint8_t wiced_hci_status = 1;
    wiced_result_t status;
    uint8_t param8;

    /* Check parameter. */
    if (p_buffer == NULL)
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    // Expected minimum 4 byte as the wiced header
    if (length < (sizeof(opcode) + sizeof(payload_len)))
    {
        wiced_transport_free_buffer(p_buffer);
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    switch(opcode)
    {
    case HCI_CONTROL_HCI_AUDIO_COMMAND_PUSH_NVRAM_DATA:
        platform_transport_rx_data_handler_push_nvram_data(p_data, payload_len);
        break;
    default:
        if (platform_cb.transport.p_app_rx_data_handler)
        {
            (*platform_cb.transport.p_app_rx_data_handler)(opcode, p_data, payload_len);
        }
        break;
    }

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer(p_buffer);

    return HCI_CONTROL_STATUS_SUCCESS;
}

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
wiced_bool_t wiced_platform_transport_init(wiced_platform_transport_rx_data_handler *p_rx_data_handler)
{
    wiced_transport_cfg_t cfg = {
                                    .type = WICED_TRANSPORT_UART,
                                    .cfg.uart_cfg = {
                                                        .mode = WICED_TRANSPORT_UART_HCI_MODE,
                                                        .baud_rate = HCI_UART_DEFAULT_BAUD,
                                                    },
                                    .rx_buff_pool_cfg = {
                                                            .buffer_size = 0,
                                                            .buffer_count = 0,
                                                        },
                                    .p_status_handler = platform_transport_status_handler,
                                    .p_data_handler = platform_transport_rx_data_handler,
                                    .p_tx_complete_cback = NULL,
                                };
    wiced_result_t result;

    if (platform_cb.transport.init)
    {
        return WICED_FALSE;
    }

    /* Initialize the transport. */
    result = wiced_transport_init(&cfg);

    if (result == WICED_BT_SUCCESS)
    {
        platform_cb.transport.init = WICED_TRUE;
        platform_cb.transport.p_app_rx_data_handler = p_rx_data_handler;

        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/**
 * wiced_platform_nvram_read
 *
 * Reads the data from NVRAM
 *
 * @param[in] vs_id         : Volatile Section Identifier. Application can use the VS IDs from
 *                            WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END
 * @param[in] data_length   : Length of the data to be read from NVRAM
 * @param[out] p_data       : Pointer to the buffer to which data will be copied
 * @param[out] p_status     : Pointer to location where status of the call is returned
 *
 * @return length of data that is read
 */
uint16_t wiced_platform_nvram_read(uint16_t vs_id, uint16_t data_length, uint8_t *p_data, wiced_result_t *p_status)
{
    platform_virtual_nvram_t *p_index;

    /* Check parameter. */
    if (p_status == NULL)
    {
        return 0;
    }

    *p_status = WICED_BADARG;

    if ((data_length == 0) ||
        (p_data == NULL))
    {
        return 0;
    }

    /* Check if the target vs_id exists. */
    p_index = platform_cb.p_virtual_nvram;
    while (p_index)
    {
        if (p_index->content.vs_id == vs_id)
        {
            /* Check the data length. */
            if (data_length < p_index->data_length)
            {
                return 0;
            }

            memcpy((void *) p_data, (void *) &p_index->content.data[0], p_index->data_length);

            break;
        }

        p_index = p_index->p_next;
    }

    if (p_index == NULL)
    {
        return 0;
    }

    *p_status = WICED_SUCCESS;

    return p_index->data_length;
}

/**
 * wiced_platform_nvram_write
 *
 * Reads the data to NVRAM
 *
 * @param[in] vs_id         : Volatile Section Identifier. Application can use the VS IDs from
 *                            WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END
 * @param[in] data_length   : Length of the data to be written to the NVRAM
 * @param[in] p_data        : Pointer to the data to be written to the NVRAM
 * @param[out] p_status     : Pointer to location where status of the call is returned
 *
 * @return number of bytes written, 0 on error
 */
uint16_t wiced_platform_nvram_write(uint16_t vs_id, uint16_t data_length, uint8_t *p_data, wiced_result_t *p_status)
{
    platform_virtual_nvram_t *p_index = NULL;
    wiced_bool_t update = WICED_FALSE;

    /* Check parameter. */
    if (p_status == NULL)
    {
        return 0;
    }

    *p_status = WICED_BADARG;

    if ((data_length == 0) ||
        (p_data == NULL))
    {
        return 0;
    }

    /* Check if the target vs_id exists. */
    p_index = platform_cb.p_virtual_nvram;
    while (p_index)
    {
        if (p_index->content.vs_id == vs_id)
        {
            wiced_result_t result;

            /* Check the data length. */
            if (data_length != p_index->data_length)
            {
                /* Delete this entry. */
                wiced_platform_nvram_delete(vs_id, p_status);

                /* Add a new entry. */
                return wiced_platform_nvram_write(vs_id, data_length, p_data, p_status);
            }

            /* Check if the data shall be updated. */
            if (memcmp((void *) p_data,
                       (void *) &p_index->content.data[0],
                       data_length) == 0)
            {
                *p_status = WICED_SUCCESS;
                return data_length;
            }

            /* Update data content. */
            memcpy((void *) &p_index->content.data[0], (void *) p_data, data_length);

            /* Inform Host device. */
            result = wiced_transport_send_data(HCI_CONTROL_HCI_AUDIO_EVENT_WRITE_NVRAM_DATA,
                                          (uint8_t *) &p_index->content,
                                          sizeof(p_index->content.vs_id) + p_index->data_length);
            if (result != WICED_SUCCESS)
            {
                /* fail, delete entry due to unsync */
                wiced_platform_nvram_delete(vs_id, p_status);

                *p_status = result;
                return 0;
            }

            *p_status = WICED_SUCCESS;
            return p_index->data_length;
        }

        p_index = p_index->p_next;
    }

    /* Acquire memory. */
    p_index = (platform_virtual_nvram_t *) platform_mem_allocate(sizeof(platform_virtual_nvram_t) - sizeof(uint8_t) + data_length);
    if (p_index == NULL)
    {
        *p_status = WICED_NO_MEMORY;
        return 0;
    }

    /* Write information. */
    p_index->content.vs_id = vs_id;
    p_index->data_length = data_length;
    memcpy((void *) &p_index->content.data[0], (void *) p_data, data_length);

    /* Add the new entry to the list. */
    p_index->p_next = platform_cb.p_virtual_nvram;
    platform_cb.p_virtual_nvram = p_index;

    /* Inform Host device. */
    if (wiced_transport_send_data(HCI_CONTROL_HCI_AUDIO_EVENT_WRITE_NVRAM_DATA,
                                  (uint8_t *) &p_index->content,
                                  sizeof(p_index->content.vs_id) + p_index->data_length) != WICED_SUCCESS)
    {
        wiced_platform_nvram_delete(p_index->content.vs_id, p_status);
        *p_status = WICED_NO_MEMORY;
        return 0;
    }

    *p_status = WICED_SUCCESS;
    return p_index->data_length;
}

/**
 * wiced_platform_nvram_delete
 *
 * Deletes data from NVRAM at specified VS id
 *
 * @param vs_id     : Volatile Section Identifier. Application can use the VS IDs from
 *                    WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END
 * @param p_status  : Pointer to location where status of the call is returned
 */
void wiced_platform_nvram_delete(uint16_t vs_id, wiced_result_t *p_status)
{
    platform_virtual_nvram_t *p_index = NULL;
    platform_virtual_nvram_t *p_pre = NULL;

    /* Check parameter. */
    if (p_status == NULL)
    {
        return;
    }

    *p_status = WICED_BADARG;

    p_index = platform_cb.p_virtual_nvram;
    p_pre = NULL;
    while (p_index)
    {
        if (p_index->content.vs_id == vs_id)
        {
            /* Inform Host device. */
            wiced_transport_send_data(HCI_CONTROL_HCI_AUDIO_EVENT_DELETE_NVRAM_DATA,
                                      (uint8_t *) &p_index->content.vs_id,
                                      sizeof(p_index->content.vs_id));

            /* Remove this entry from the list. */
            if (p_pre == NULL)
            {
                platform_cb.p_virtual_nvram = p_index->p_next;
            }
            else
            {
                p_pre->p_next = p_index->p_next;
            }

            platform_mem_free((void *) p_index);

            break;
        }

        p_pre = p_index;
        p_index = p_index->p_next;
    }
}
