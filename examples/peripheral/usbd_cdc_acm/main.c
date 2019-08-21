/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "radio_config.h"
#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "boards.h"
#include "bsp.h"
#include "bsp_cli.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"



/**@file
 * @defgroup usbd_cdc_acm_example main.c
 * @{
 * @ingroup usbd_cdc_acm_example
 * @brief USBD CDC ACM example
 *
 */

#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_0)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_TX      (BSP_BOARD_LED_0)
//#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_1)
//#define LED_CDC_ACM_RX      (BSP_BOARD_LED_2)
//#define LED_CDC_ACM_TX      (BSP_BOARD_LED_3)

#define BTN_CDC_DATA_SEND       1
#define BTN_CDC_NOTIFY_SEND     0

#define BTN_CDC_DATA_KEY_RELEASE        (bsp_event_t)(BSP_EVENT_KEY_LAST + 1)

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1


/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 0;

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   READ_SIZE);
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            bsp_board_led_off(LED_CDC_ACM_OPEN);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            bsp_board_led_invert(LED_CDC_ACM_TX);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;

			do
            {
                /*Get amount of data transfered*/
                app_usbd_cdc_acm_rx_size(p_cdc_acm);

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            m_rx_buffer,
                                            READ_SIZE);
            } while (ret == NRF_SUCCESS);

            bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            app_usbd_start();
            break;
        default:
            break;
    }
}

static void bsp_event_callback(bsp_event_t ev)
{
    switch ((unsigned int)ev)
    {
        case BSP_EVENT_KEY_1:
        {
            break;
        }
        
        case BTN_CDC_DATA_KEY_RELEASE :
        {
            break;
        }

        case BSP_EVENT_KEY_0:
        {
			m_send_flag = 1;
            app_usbd_cdc_acm_serial_state_notify(&m_app_cdc_acm, APP_USBD_CDC_ACM_SERIAL_STATE_BREAK, false);
            break;
        }

        default:
            return; // no implementation needed
    }
}

static void init_bsp(void)
{

    bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);    
	bsp_event_to_button_action_assign(BTN_CDC_DATA_SEND, BSP_BUTTON_ACTION_RELEASE, BTN_CDC_DATA_KEY_RELEASE);
    
    /* Configure LEDs */
    bsp_board_init(BSP_INIT_LEDS);
}





static uint32_t                   packet;              /**< Packet to transmit. */


/**@brief Function for reading packet.
 */
#define RX_STATE_START			0
#define RX_STATE_ENABLED		1
#define RX_STATE_PACKET_END		2
#define RX_STATE_DISABLE		3

static  uint8_t rx_state = RX_STATE_START;

uint32_t read_packet()
{
	static uint32_t result = 0;

	if (rx_state == RX_STATE_START)
	{
		NRF_RADIO->EVENTS_READY = 0U;
		// Enable radio and wait for ready
		NRF_RADIO->TASKS_RXEN = 1U;
		rx_state = RX_STATE_ENABLED;
		bsp_board_led_on(3);
	}

	if (rx_state == RX_STATE_ENABLED)
	{
		//while (NRF_RADIO->EVENTS_READY == 0U)
		if (NRF_RADIO->EVENTS_READY == 0U)
		{
			return 0;
			// wait
		}

		NRF_RADIO->EVENTS_END = 0U;
		// Start listening and wait for address received event
		NRF_RADIO->TASKS_START = 1U;

		rx_state = RX_STATE_PACKET_END;
		bsp_board_led_off(3);
		bsp_board_led_on(2);
	}

	if (rx_state == RX_STATE_PACKET_END)
	{
		// Wait for end of packet or buttons state changed
		//while (NRF_RADIO->EVENTS_END == 0U)
		if (NRF_RADIO->EVENTS_END == 0U)
		{
			return 0;
			// wait
		}

		if (NRF_RADIO->CRCSTATUS == 1U)
		{
			result = packet;
		}

		NRF_RADIO->EVENTS_DISABLED = 0U;
		// Disable radio
		NRF_RADIO->TASKS_DISABLE = 1U;

		rx_state = RX_STATE_DISABLE;

		m_send_flag = 1;

		bsp_board_led_off(2);
		bsp_board_led_on(1);
	}

	if (rx_state == RX_STATE_DISABLE)
	{
		//while (NRF_RADIO->EVENTS_DISABLED == 0U)
		if(NRF_RADIO->EVENTS_DISABLED == 0U)
		{
			return 0;
			// wait
		}

		rx_state = RX_STATE_START;
		return result;
	}

	return 0;
}

static int  frame_counter;
uint32_t received = 0;

int main(void)
{
    ret_code_t ret;

    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };


    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    ret = app_timer_init();

    init_bsp();

    app_usbd_serial_num_generate();
    app_usbd_init(&usbd_config);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    app_usbd_class_append(class_cdc_acm);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
    }
    else
    {
        app_usbd_enable();
        app_usbd_start();
    }

	// Radio Init
	radio_configure();
	NRF_RADIO->PACKETPTR = (uint32_t)&packet;

    while (true)
    {
		frame_counter++;

        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        
        if(m_send_flag)
        {
            size_t size = sprintf(m_tx_buffer, "Hello USB CDC FA demo: %u\r\n", frame_counter);

            ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);
            if (ret == NRF_SUCCESS)
            {
                ++frame_counter;
            }

			m_send_flag = 0;
        }
        
		received = read_packet();
		if (received)
		{
			m_send_flag = 1;
		}

        //UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        /* Sleep CPU only if there was no interrupt since last loop processing */
        //__WFE();

    }
}

/** @} */
