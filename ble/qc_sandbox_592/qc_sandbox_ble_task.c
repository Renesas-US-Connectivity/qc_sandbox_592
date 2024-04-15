/**
 ****************************************************************************************
 *
 * @file pxp_reporter_task.c
 *
 * @brief PXP profile application implementation
 *
 * Copyright (C) 2015-2023 Renesas Electronics Corporation and/or its affiliates.
 * All rights reserved. Confidential Information.
 *
 * This software ("Software") is supplied by Renesas Electronics Corporation and/or its
 * affiliates ("Renesas"). Renesas grants you a personal, non-exclusive, non-transferable,
 * revocable, non-sub-licensable right and license to use the Software, solely if used in
 * or together with Renesas products. You may make copies of this Software, provided this
 * copyright notice and disclaimer ("Notice") is included in all such copies. Renesas
 * reserves the right to change or discontinue the Software at any time without notice.
 *
 * THE SOFTWARE IS PROVIDED "AS IS". RENESAS DISCLAIMS ALL WARRANTIES OF ANY KIND,
 * WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. TO THE
 * MAXIMUM EXTENT PERMITTED UNDER LAW, IN NO EVENT SHALL RENESAS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE, EVEN IF RENESAS HAS BEEN ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGES. USE OF THIS SOFTWARE MAY BE SUBJECT TO TERMS AND CONDITIONS CONTAINED IN
 * AN ADDITIONAL AGREEMENT BETWEEN YOU AND RENESAS. IN CASE OF CONFLICT BETWEEN THE TERMS
 * OF THIS NOTICE AND ANY SUCH ADDITIONAL LICENSE AGREEMENT, THE TERMS OF THE AGREEMENT
 * SHALL TAKE PRECEDENCE. BY CONTINUING TO USE THIS SOFTWARE, YOU AGREE TO THE TERMS OF
 * THIS NOTICE.IF YOU DO NOT AGREE TO THESE TERMS, YOU ARE NOT PERMITTED TO USE THIS
 * SOFTWARE.
 *
 ****************************************************************************************
 */

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "osal.h"
#include "ble_att.h"
#include "ble_common.h"
#include "ble_gap.h"
#include "ble_gattc.h"
#include "ble_gatts.h"
#include "ble_l2cap.h"
#include "sdk_list.h"
#include "bas.h"
#include "ias.h"
#include "lls.h"
#include "tps.h"
#include "sys_power_mgr.h"

#include "ad_nvparam.h"
#include "ad_gpadc.h"
#include "hw_gpio.h"

#include "sys_watchdog.h"
#include "platform_devices.h"
#include "qc_svc.h"
#include "gui_cfg.h"
#include "hs300x_task.h"
#include "hw_wkup_v2.h"
#include "hw_pdc.h"

static void handle_read_gui_req(uint8_t conn_idx, uint16_t id, uint8_t const * const data);
static void handle_write_led(uint8_t conn_idx, uint16_t id, uint8_t const * const data);
static void handle_read_version(uint8_t conn_idx, uint16_t id, uint8_t const * const data);
static void handle_read_temperature_param(uint8_t conn_idx, uint16_t id, uint8_t const * const data);
static void handle_read_humidity_param(uint8_t conn_idx, uint16_t id, uint8_t const * const data);

typedef enum
{
	LED_OFF = 0x00,
	LED_ON = 0x01,
} led_state_t;

typedef __PACKED_STRUCT
{
    int16_t temperature;
    int16_t  humidity;
} graph_data_t;

/* Step 1 - Update the device name to something unique
   Note the device name must be of the format
   xxyyy-zzzzzzzzz
   where:
   xx - two-character country code which must be one of the following, US, EU, CN, JP, or TW.
        Use US for this lab
   yyy - three digit numeric code indicating the winning combo number associated with the PoC.
         Use 592 for this lab
   zzz – string representing the name of the PoC, should not be more than 16 characters long.
         Use any unique string identify your EVK
*/
static char *device_name  = "US592-YourDevice"; // QC-Sandbox-592
static int16_t temperature;
static int16_t humidity;
ble_service_t* qc_svc_handle;
static char *version_str  = "1.0.0";

static const qc_svc_request_handlers_t qc_sv_req_handlers[] =
{
	/* Object ID	Read Handler						Write Handler */
	{ 0x0000,		handle_read_gui_req,				NULL					},
	{ 0x0100,		NULL,								handle_write_led		},
	{ 0x0201,		handle_read_temperature_param,		NULL					},
	{ 0x0202,		handle_read_humidity_param,			NULL					},
	{ 0x0301,		handle_read_version,				NULL					},
	{ 0xFFFF,		NULL,								NULL					}
};

static void handle_evt_gap_connected(ble_evt_gap_connected_t *evt)
{
	// Any necessary setup for connection

	// Update transport mtu for connection
	uint16_t mtu;
	ble_gattc_get_mtu(evt->conn_idx, &mtu);
	qc_svc_set_transport_mtu(mtu - 3); // Subtract 3 to account for L2CAP overhead
}

static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt)
{
	// Restart advertising
	ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
}

static void handle_evt_gap_pair_req(ble_evt_gap_pair_req_t *evt)
{
	/* Just accept the pairing request, set bond flag to what peer requested */
	ble_gap_pair_reply(evt->conn_idx, true, evt->bond);
}

static void handle_evt_gattc_mtu_changed(ble_evt_gattc_mtu_changed_t *evt)
{
	// update transport mtu
	qc_svc_set_transport_mtu(evt->mtu - 3); // Subtract 3 to account for L2CAP overhead
}

static void handle_read_gui_req(uint8_t conn_idx, uint16_t id, uint8_t const * const data)
{
    uint16_t len = strlen((char *)gui_cfg);

    if (len > 0)
    {
        qc_svc_send_read_response(conn_idx, QC_SVC_SUCCESS, id, len, (uint8_t *)&gui_cfg[0]);
    }
}

static void handle_read_humidity_param(uint8_t conn_idx, uint16_t id, uint8_t const * const data)
{
    qc_svc_send_read_response(conn_idx, QC_SVC_SUCCESS, id, sizeof(humidity), (uint8_t *)&humidity);
}

static void handle_read_temperature_param(uint8_t conn_idx, uint16_t id, uint8_t const * const data)
{
    qc_svc_send_read_response(conn_idx, QC_SVC_SUCCESS, id, sizeof(temperature), (uint8_t *)&temperature);
}

static void handle_read_version(uint8_t conn_idx, uint16_t id, uint8_t const * const data)
{
    qc_svc_send_read_response(conn_idx, QC_SVC_SUCCESS, id, (uint16_t)strlen(version_str), (uint8_t *)version_str);
}

static void handle_write_led(uint8_t conn_idx, uint16_t id, uint8_t const * const data)
{
	qc_svc_error_t write_status = QC_SVC_SUCCESS;
	if (data[4] == LED_OFF)
	{
		hw_sys_pd_com_enable();
		hw_gpio_set_pin_function(LED1_PORT, LED1_PIN, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
		hw_gpio_pad_latch_enable(LED1_PORT, LED1_PIN);
		hw_gpio_set_inactive(LED1_PORT, LED1_PIN);
		hw_gpio_pad_latch_disable(LED1_PORT, LED1_PIN);
		hw_sys_pd_com_disable();
	}
	else if (data[4] == LED_ON)
	{
		hw_sys_pd_com_enable();
		hw_gpio_set_pin_function(LED1_PORT, LED1_PIN, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
		hw_gpio_pad_latch_enable(LED1_PORT, LED1_PIN);
		hw_gpio_set_active(LED1_PORT, LED1_PIN);
		hw_gpio_pad_latch_disable(LED1_PORT, LED1_PIN);		hw_sys_pd_com_disable();
	}
	else
	{
		write_status = QC_SVC_ERR_INVALID_REQEST;
	}

	qc_svc_send_write_response(conn_idx, write_status, id);
}

static void send_qc_svc_response(uint8_t conn_idx, uint8_t const * const p_data, uint16_t len)
{
    qc_svc_notify(qc_svc_handle, conn_idx, p_data, len);
}
void qc_sandbox_ble_task(void *pvParameters)
{
	OS_QUEUE sample_q = (OS_QUEUE)pvParameters;

	hs300x_task_event_queue_register(OS_GET_CURRENT_TASK());

	int8_t wdog_id;

	/* Register task to be monitored by watchdog */
	wdog_id = sys_watchdog_register(false);

	/*************************************************************************************************\
	* Initialize BLE
	*/
	/* Start BLE device as peripheral */
	ble_peripheral_start();

	/* Register task to BLE framework to receive BLE event notifications */
	ble_register_app();

	// Use a random address to avoid collisions
	own_address_t random_addr = {PRIVATE_RANDOM_RESOLVABLE_ADDRESS};
	ble_gap_address_set(&random_addr, 3600);

	/* Add custom sensor service */
	qc_svc_handle = qc_svc_init(&qc_sv_req_handlers[0], send_qc_svc_response);
	qc_svc_set_transport_mtu(DEFAULT_MTU_BYTES-3); // Subtract 3 to account for L2CAP overhead

	/* Set device name */
	ble_gap_device_name_set(device_name, ATT_PERM_READ);

	/* Define Scan Response object internals dealing with retrieved name */
	gap_adv_ad_struct_t *adv_data = GAP_ADV_AD_STRUCT_DECLARE(GAP_DATA_TYPE_LOCAL_NAME,
										  	  	  	  	  	  strlen(device_name),
															  device_name);

	// Start advertising
	ble_gap_adv_ad_struct_set(1, adv_data, 0 , NULL);
	ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);

	for (;;)
	{
		OS_BASE_TYPE ret __UNUSED;
		uint32_t notif;

		/* Notify watchdog on each loop */
		sys_watchdog_notify(wdog_id);

		/* Suspend watchdog while blocking on OS_TASK_NOTIFY_WAIT() */
		sys_watchdog_suspend(wdog_id);

		/*
		* Wait on any of the notification bits, then clear them all
		*/
		ret = OS_TASK_NOTIFY_WAIT(OS_TASK_NOTIFY_NONE, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
		/* Blocks forever waiting for the task notification. Therefore, the return value must
		* always be OS_OK
		*/
		OS_ASSERT(ret == OS_OK);

		/* Resume watchdog */
		sys_watchdog_notify_and_resume(wdog_id);

		/* Notified from BLE manager? */
		if (notif & BLE_APP_NOTIFY_MASK)
		{
			ble_evt_hdr_t *hdr;
			hdr = ble_get_event(false);
			if (hdr)
			{
				if (!ble_service_handle_event(hdr))
				{
					switch (hdr->evt_code)
					{
						case BLE_EVT_GAP_CONNECTED:
							handle_evt_gap_connected((ble_evt_gap_connected_t *) hdr);
							break;
						case BLE_EVT_GAP_DISCONNECTED:
							handle_evt_gap_disconnected((ble_evt_gap_disconnected_t *) hdr);
							break;
						case BLE_EVT_GAP_PAIR_REQ:
							handle_evt_gap_pair_req((ble_evt_gap_pair_req_t *) hdr);
							break;
						case BLE_EVT_GATTC_MTU_CHANGED:
							handle_evt_gattc_mtu_changed((ble_evt_gattc_mtu_changed_t *) hdr);
							break;
						default:
							ble_handle_event_default(hdr);
							break;
					}
				}
				/* Free event buffer (it's not needed anymore) */
				OS_FREE(hdr);
			}

			/*
			* If there are more events waiting in queue, application should process
			* them now.
			*/
			if (ble_has_event())
			{
				OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(),
							   BLE_APP_NOTIFY_MASK,
							   OS_NOTIFY_SET_BITS);
			}
		}
		/* Notified from HS3001 Task */
		if (notif & HS3001_MEASUREMENT_NOTIFY_MASK)
		{
			// Get a measurement from the queue
			hs300x_data_t sample = {0};
			OS_QUEUE_GET(sample_q, &sample, OS_QUEUE_NO_WAIT);

			humidity = sample.humidity_rh_pct;
			temperature = sample.temp_deg_c;
		}
	}
}

