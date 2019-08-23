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

#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "boards.h"
#include "bsp.h"
#include "nrf_cli_uart.h"


#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_0)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_TX      (BSP_BOARD_LED_0)

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


/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer, READ_SIZE);
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
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer, READ_SIZE);
            } while (ret == NRF_SUCCESS);

            bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}

void usbd_user_ev_handler(app_usbd_event_type_t event)
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






#define RX_STATE_START			0
#define RX_STATE_ENABLED		1
#define RX_STATE_PACKET_END		2
#define RX_STATE_DISABLE		3

static uint32_t               packet;              /**< Packet to transmit. */
static  uint8_t rx_state = RX_STATE_START;

/**@brief Function for reading packet.
 */
uint32_t read_packet()
{
	static uint32_t result = 0;

	if (rx_state == RX_STATE_START)
	{
		NRF_RADIO->EVENTS_READY = 0U;
		NRF_RADIO->TASKS_RXEN = 1U;
		rx_state = RX_STATE_ENABLED;
	}

	if (rx_state == RX_STATE_ENABLED)
	{
		if (NRF_RADIO->EVENTS_READY == 0U)
		{
			return 0;
		}

		NRF_RADIO->EVENTS_END = 0U;
		// Start listening and wait for address received event
		NRF_RADIO->TASKS_START = 1U;

		rx_state = RX_STATE_PACKET_END;
	}

	if (rx_state == RX_STATE_PACKET_END)
	{
		// Wait for end of packet or buttons state changed
		if (NRF_RADIO->EVENTS_END == 0U)
		{
			return 0;
		}

		if (NRF_RADIO->CRCSTATUS == 1U)
		{
			result = packet;
		}

		NRF_RADIO->EVENTS_DISABLED = 0U;
		// Disable radio
		NRF_RADIO->TASKS_DISABLE = 1U;

		rx_state = RX_STATE_DISABLE;
	}

	if (rx_state == RX_STATE_DISABLE)
	{
		if(NRF_RADIO->EVENTS_DISABLED == 0U)
		{
			return 0;
		}

		rx_state = RX_STATE_START;
		return result;
	}

	return 0;
}


void clock_initialization()
{
	/* Start 16 MHz crystal oscillator */
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;

	/* Wait for the external oscillator to start up */
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
	{
		// Do nothing.
	}

	/* Start low frequency crystal oscillator for app_timer(used by bsp)*/
	NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;

	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
	{
		// Do nothing.
	}
}

int main(void)
{
	int rx_counter =0;
	uint32_t received = 0;
	bool rx_flag = false;

    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

	clock_initialization();
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

	app_timer_init();
	bsp_init(BSP_INIT_LEDS, NULL);

    app_usbd_serial_num_generate();
    app_usbd_init(&usbd_config);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    app_usbd_class_append(class_cdc_acm);

    if (USBD_POWER_DETECTION)
    {
        app_usbd_power_events_enable();
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
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        
        if(rx_flag)
        {
			rx_flag = false;

            size_t size = sprintf(m_tx_buffer, "Rx Count: %d %x\r\n", rx_counter, (unsigned int)received);
			app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);
        }
        
		received = read_packet();
		if (received)
		{
			bsp_board_led_invert(BSP_BOARD_LED_3);
			rx_flag = true;
			rx_counter++;
		}

    }
}

/** @} */
