/*
 * qc_svc.c
 *
 *  Created on: Mar 13, 2024
 *      Author: a5137667
 */
#include "qc_svc.h"
#include "ble_uuid.h"
#include "ble_storage.h"
#include "osal.h"
#include "ble_bufops.h"
#include <stdio.h>

#define QC_SVC_REQUEST_LEN_MIN        3
#define QC_SVC_REQUEST_TYPE_WRITE     0x01
#define QC_SVC_REQUEST_TYPE_READ      0x02
#define QC_SVC_RESPONSE_FRAG_LEN_MAX  256

typedef __PACKED_STRUCT
{
  uint32_t   payload_len;
  uint32_t   offset;
  uint16_t   remaining;
  uint8_t  * payload_data;
  bool       in_progress;
  uint16_t   id;
  uint8_t    type;
  uint8_t    status;
} qc_svc_read_rsp_msg_t;

typedef __PACKED_STRUCT
{
  uint8_t    type;
  uint8_t    status;
  uint16_t   id;
} qc_svc_write_rsp_msg_t;

/* Service Constants */
static const char request_char_user_description[]  = "Request";
static const char resposne_char_user_description[]  = "Response";

static qc_svc_request_handlers_t const * p_req_handlers = NULL;
static qc_svc_response_transmit_cb_t     transmit_cb;
static qc_svc_read_rsp_msg_t             qc_svc_read_rsp_msg;
static qc_svc_write_rsp_msg_t            qc_svc_write_rsp_msg;
static uint16_t                          transport_mtu  = DEFAULT_MTU_BYTES;

static void cleanup(ble_service_t *svc);
static void handle_response_ccc_read(qc_svc_t *qc_svc_handle, const ble_evt_gatts_read_req_t *evt);
static att_error_t handle_response_ccc_write(qc_svc_t *qc_svc_handle, const ble_evt_gatts_write_req_t *evt);
static qc_svc_error_t send_rsp_msg_fragment(uint8_t conn_idx, qc_svc_read_rsp_msg_t * p_rsp);

static void cleanup(ble_service_t *svc)
{
	qc_svc_t *qc_svc_handle = (qc_svc_t *) svc;
	ble_storage_remove_all(qc_svc_handle->response_ccc_h);
	OS_FREE(qc_svc_handle);
}

static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt)
{
	qc_svc_t *qc_svc_handle = (qc_svc_t *) svc;

	/*
	 * Identify which attribute handle the read request has been sent to
	 * and call the appropriate function.
	 */

	if(evt->handle == qc_svc_handle->response_value_h)
	{
		// just confirm the read, all data will be transmitted thru notifications
		uint8_t rsp = 0;
		ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, 0, &rsp);
	}
	else if(evt->handle == qc_svc_handle->response_ccc_h )
	{
		handle_response_ccc_read(qc_svc_handle, evt);
	}
	// Otherwise read operations are not permitted
	else
	{
		ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_READ_NOT_PERMITTED, 0, NULL);
	}

}

static void handle_response_ccc_read(qc_svc_t *qc_svc_handle, const ble_evt_gatts_read_req_t *evt)
{
	uint16_t ccc = 0x0000;

	// Extract the CCC value from the ble storage
	ble_storage_get_u16(evt->conn_idx, qc_svc_handle->response_ccc_h, &ccc);

	// Send a read confirmation with the value from storage
	ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(ccc), &ccc);
}

static att_error_t handle_response_ccc_write(qc_svc_t *qc_svc_handle, const ble_evt_gatts_write_req_t *evt)
{
	att_error_t error = ATT_ERROR_OK;

	// Verify the write request is valid
	if(evt->offset)
	{
		error = ATT_ERROR_ATTRIBUTE_NOT_LONG;
	}
	else if(evt->length != sizeof(uint16_t)) // All CCCs are 2 bytes
	{
		error = ATT_ERROR_INVALID_VALUE_LENGTH;
	}
	else
	{
		uint16_t ccc = get_u16(evt->value);

		// Store the CCC value to ble storage
		ble_storage_put_u32(evt->conn_idx, qc_svc_handle->response_ccc_h, ccc, true);

		// Respond to the write requst
		ble_gatts_write_cfm(evt->conn_idx, qc_svc_handle->response_ccc_h, error);
	}

	return error;
}

static void handle_response_write_cfm(ble_service_t *svc, const ble_evt_gatts_event_sent_t *evt)
{
	if (qc_svc_read_rsp_msg.in_progress == true)
    {
        if (qc_svc_read_rsp_msg.remaining > 0)
        {
            send_rsp_msg_fragment(evt->conn_idx, &qc_svc_read_rsp_msg);
        }
        else
        {
            qc_svc_read_rsp_msg.in_progress = false;
        }
    }
}

static void handle_write_req(ble_service_t *svc, const ble_evt_gatts_write_req_t *evt)
{
	qc_svc_t *qc_svc_handle = (qc_svc_t *) svc;

	ble_gatts_write_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK);

	/*
	 * Identify for which attribute handle the write request has been sent to
	 * and call the appropriate function.
	 */

	if(evt->handle == qc_svc_handle->request_value_h)
	{
		qc_svc_handle_request(qc_svc_handle, evt);
	}
	else if(evt->handle == qc_svc_handle->response_ccc_h)
	{
		handle_response_ccc_write(qc_svc_handle, evt);
	}
}

qc_svc_error_t qc_svc_handle_request(qc_svc_t *qc_svc_handle, const ble_evt_gatts_write_req_t *evt)
{
	uint32_t len = evt->length;

    uint8_t                  i;
    uint8_t                  type;
    uint16_t                 id;
    qc_svc_error_t           err         = QC_SVC_SUCCESS;
    qc_svc_request_handler_t req_handler = NULL;

    type = evt->value[0];
    id  =  (uint16_t)evt->value[1];
    id |=  (uint16_t)((uint16_t)evt->value[2] << 8);



    if (QC_SVC_REQUEST_LEN_MIN <= len)
    {
        switch (type)
        {
            case QC_SVC_REQUEST_TYPE_WRITE:
            case QC_SVC_REQUEST_TYPE_READ:
            {
                if (NULL != p_req_handlers)
                {
                    for (i = 0; p_req_handlers[i].id != 0xFFFF; i++)
                    {
                        if (p_req_handlers[i].id == id)
                        {
                            switch (type)
                            {
                                case QC_SVC_REQUEST_TYPE_READ:
                                    req_handler = p_req_handlers[i].rd_handler;
                                    break;
                                case QC_SVC_REQUEST_TYPE_WRITE:
                                    req_handler = p_req_handlers[i].wr_handler;
                                    break;
                                default:
                                    break;
                            }
                            if (NULL != req_handler)
                            {
                                req_handler(evt->conn_idx, id, &evt->value[3]);
                            }
                            else
                            {
                                err = QC_SVC_ERR_UNKNOWN_ID;
                                qc_svc_send_read_response(evt->conn_idx, QC_SVC_ERR_HANLDERS_NOT_REGISTERED, id, 0, NULL);
                            }
                            break;
                        }
                    }
                }
                else
                {
                    err = QC_SVC_ERR_HANLDERS_NOT_REGISTERED;
                }
            }
            break;

            default:
            {
                /* Unknown request type */
                err = QC_SVC_ERR_UNKNOWN_REQUEST_TYPE;
            }
            break;
        }
    }

    return err;
}

ble_service_t *qc_svc_init(qc_svc_request_handlers_t const * const p_handlers, qc_svc_response_transmit_cb_t tx_cb)
{
	qc_svc_t *qc_svc_handle;

	uint16_t num_attr;
	att_uuid_t uuid;

	// Allocate memory for the service handle
	qc_svc_handle = (qc_svc_t *) OS_MALLOC(sizeof(qc_svc_t));
	memset(qc_svc_handle, 0, sizeof(qc_svc_t));

	// Declare handlers for specific BLE events
	qc_svc_handle->svc.read_req  = handle_read_req;
	qc_svc_handle->svc.write_req = handle_write_req;
	qc_svc_handle->svc.event_sent = handle_response_write_cfm;
	qc_svc_handle->svc.cleanup   = cleanup;

	/*
	 * 0 --> Number of Included Services
	 * 2 --> Number of Characteristic Declarations
	 * 3 --> Number of Descriptors
	 */
	num_attr = ble_gatts_get_num_attr(0, 2, 3);

	// Service declaration
	ble_uuid_from_string(QUICK_CONNECT_SVC_UUID_STR, &uuid);
	ble_gatts_add_service(&uuid, GATT_SERVICE_PRIMARY, num_attr);

	// Characteristic declaration for Request Characteristic
	ble_uuid_from_string(QUICK_CONNECT_REQ_CHAR_UUID_STR, &uuid);
	ble_gatts_add_characteristic(&uuid,
								 GATT_PROP_WRITE,
								 ATT_PERM_WRITE,
	                             32,
	                             0,
	                             NULL,
	                             &qc_svc_handle->request_value_h);

	// Define descriptor of type Characteristic User Description for Request Characteristic
	ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);
	ble_gatts_add_descriptor(&uuid,
                             ATT_PERM_READ,
                             sizeof(request_char_user_description)-1, // -1 to account for NULL char
                             0,
                             &qc_svc_handle->request_user_desc_h);

	// Characteristic declaration for Response Characteristic
	ble_uuid_from_string(QUICK_CONNECT_RESP_CHAR_UUID_STR, &uuid);
	ble_gatts_add_characteristic(&uuid,
								 GATT_PROP_READ | GATT_PROP_NOTIFY,
								 ATT_PERM_READ,
                                 101,
                                 GATTS_FLAG_CHAR_READ_REQ,
                                 NULL,
                                 &qc_svc_handle->response_value_h);

	// Define descriptor of type Characteristic User Description for Response characteristic
	ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);
	ble_gatts_add_descriptor(&uuid,
                             ATT_PERM_READ,
                             sizeof(resposne_char_user_description)-1,  // -1 to account for NULL char
                             0,
                             &qc_svc_handle->response_user_desc_h);

	// Define descriptor of type Cleint Characteristic Configuration Descriptor for Response Characteristic
	ble_uuid_create16(UUID_GATT_CLIENT_CHAR_CONFIGURATION, &uuid);
	ble_gatts_add_descriptor(&uuid,
                             ATT_PERM_RW,
                             2,
                             0,
                             &qc_svc_handle->response_ccc_h);

	/*
	 * Register all the attribute handles so that they can be updated
	 * by the BLE manager automatically.
	 */
	ble_gatts_register_service(&qc_svc_handle->svc.start_h,
                               &qc_svc_handle->request_value_h,
                               &qc_svc_handle->request_user_desc_h,
                               &qc_svc_handle->response_value_h,
                               &qc_svc_handle->response_user_desc_h,
                               &qc_svc_handle->response_ccc_h,
                               0);

	// Calculate the last attribute handle of the BLE service
	qc_svc_handle->svc.end_h = qc_svc_handle->svc.start_h + num_attr;

	// Set default values for User Descriptions
	ble_gatts_set_value(qc_svc_handle->request_user_desc_h,
                        sizeof(request_char_user_description)-1,
						request_char_user_description);

	ble_gatts_set_value(qc_svc_handle->response_user_desc_h,
	                    sizeof(resposne_char_user_description)-1,
						resposne_char_user_description);

	p_req_handlers = p_handlers;
	transmit_cb = tx_cb;

	// Register the BLE service in BLE framework
	ble_service_add(&qc_svc_handle->svc);

	// Return the service handle
	return &qc_svc_handle->svc;
}

void qc_svc_notify(ble_service_t *svc, uint16_t conn_idx, uint8_t const * const p_data, uint16_t len)
{
	qc_svc_t *qc_svc_handle = (qc_svc_t *) svc;

	uint16_t ccc = 0x0000;
	ble_storage_get_u16(conn_idx, qc_svc_handle->response_ccc_h, &ccc);

	if (ccc & GATT_CCC_NOTIFICATIONS)
	{
		ble_gatts_send_event(conn_idx, qc_svc_handle->response_value_h, GATT_EVENT_NOTIFICATION, len, (uint8_t *)p_data);
	}
}

qc_svc_error_t qc_svc_send_read_response(uint8_t conn_idx, qc_svc_error_t status, uint16_t id, uint16_t len, uint8_t * data)
{
    qc_svc_error_t err;

    qc_svc_read_rsp_msg.type         = QC_SVC_REQUEST_TYPE_READ;
    qc_svc_read_rsp_msg.status       = status;
    qc_svc_read_rsp_msg.id           = id;
    qc_svc_read_rsp_msg.payload_len  = len;
    qc_svc_read_rsp_msg.payload_data = data;
    qc_svc_read_rsp_msg.remaining    = len;
    qc_svc_read_rsp_msg.offset       = 0;


    err = send_rsp_msg_fragment(conn_idx, &qc_svc_read_rsp_msg);

    return err;
}

qc_svc_error_t qc_svc_set_transport_mtu(uint16_t mtu)
{
    transport_mtu = mtu;

    return QC_SVC_SUCCESS;
}

qc_svc_error_t qc_svc_send_write_response(uint8_t conn_idx, qc_svc_error_t status, uint16_t id)
{
    qc_svc_write_rsp_msg.type         = QC_SVC_REQUEST_TYPE_WRITE;
    qc_svc_write_rsp_msg.status       = status;
    qc_svc_write_rsp_msg.id           = id;

	printf("qc_svc_send_write_response. sizeof: %d. id: %d\r\n", sizeof(qc_svc_write_rsp_msg), id);

    if (transmit_cb != NULL)
    {
    	transmit_cb(conn_idx, (uint8_t*)&qc_svc_write_rsp_msg, sizeof(qc_svc_write_rsp_msg));
    }

    return QC_SVC_SUCCESS;
}

static qc_svc_error_t send_rsp_msg_fragment(uint8_t conn_idx, qc_svc_read_rsp_msg_t * p_rsp)
{
    /* Determine size of header */
    uint16_t  hdr_len = 0;
    uint16_t frag_len = 0;
    uint16_t ix       = 0;
    uint8_t  buffer[QC_SVC_RESPONSE_FRAG_LEN_MAX];

    if (p_rsp->payload_len > 0)
    {
        hdr_len = sizeof(p_rsp->type) + sizeof(p_rsp->status) +
                  sizeof(p_rsp->id)   + sizeof(p_rsp->payload_len);
    }
    else
    {
        hdr_len = sizeof(p_rsp->type) + sizeof(p_rsp->status) +
                  sizeof(p_rsp->id);
    }

    /* Determine size of fragment to be transmitted */
    if ((hdr_len + p_rsp->remaining) <= transport_mtu)
    {
        frag_len = hdr_len + p_rsp->remaining;
    }
    else
    {
        frag_len = transport_mtu;
    }

    /* Add packet fragment header */
    buffer[ix++] = p_rsp->type;
    buffer[ix++] = p_rsp->status;
    memcpy(&buffer[ix], (uint8_t *)&p_rsp->id, sizeof(p_rsp->id));
    ix += sizeof(p_rsp->id);

    memcpy(&buffer[ix], (uint8_t *)&p_rsp->payload_len, sizeof(p_rsp->payload_len));
    ix += sizeof(p_rsp->payload_len);

    /* Add payload data, if present */
    if (p_rsp->payload_len > 0)
    {
        if (p_rsp->payload_data != NULL)
        {
            memcpy(&buffer[ix], (uint8_t *)&p_rsp->payload_data[p_rsp->offset], (frag_len - hdr_len));
        }
        p_rsp->offset += (frag_len - hdr_len);
        ix += ((uint16_t)(frag_len - hdr_len));
    }
    p_rsp->in_progress = true;
    p_rsp->remaining  -= ((uint16_t)(frag_len - hdr_len));

    if (transmit_cb != NULL)
    {
        transmit_cb(conn_idx, buffer, ix);
    }

    return QC_SVC_SUCCESS;
}

