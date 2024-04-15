/*
 * qc_svc.h
 *
 *  Created on: Mar 13, 2024
 *      Author: a5137667
 */

#ifndef QC_SVC_H_
#define QC_SVC_H_

#include <stdint.h>
#include <ble_service.h>

#define QUICK_CONNECT_SVC_UUID_STR				"18424398-7cbc-11e9-8f9e-2a86e4085a59"
#define QUICK_CONNECT_REQ_CHAR_UUID_STR         "2d86686a-53dc-25b3-0c4a-f0e10c8dee20"
#define QUICK_CONNECT_RESP_CHAR_UUID_STR        "2d86686a-53dc-25b3-0c4a-f0e10c8dee22"
#define DEFAULT_MTU_BYTES	 					23

typedef enum e_qc_svc_err
{
    QC_SVC_SUCCESS                     = 0,
    QC_SVC_ERR_INVALID_REQEST          = 1,
    QC_SVC_ERR_UNKNOWN_REQUEST_TYPE    = 2,
    QC_SVC_ERR_ALREADY_REGISTERED      = 3,
    QC_SVC_ERR_HANLDERS_NOT_REGISTERED = 4,
    QC_SVC_ERR_UNKNOWN_ID              = 5,
} qc_svc_error_t;

typedef void (* qc_svc_request_handler_t)(uint8_t conn_idx, uint16_t id, uint8_t const * const data);
typedef void (* qc_svc_response_transmit_cb_t)(uint8_t conn_idx, uint8_t const * const data, uint16_t len);

typedef struct
{
    uint16_t                 id;
    qc_svc_request_handler_t rd_handler;
    qc_svc_request_handler_t wr_handler;
} qc_svc_request_handlers_t;

typedef struct {
        ble_service_t svc;

        // Attribute handles of BLE service
        uint16_t request_value_h;			// Request Value
        uint16_t request_user_desc_h;		// Request User Description
        uint16_t response_value_h;			// Response Value
        uint16_t response_user_desc_h;	    // Response User Description
        uint16_t response_ccc_h;		    // Response Client Characteristic Configuration Descriptor. Used for notifications
} qc_svc_t;

ble_service_t *qc_svc_init(qc_svc_request_handlers_t const * const p_handlers, qc_svc_response_transmit_cb_t tx_cb);
qc_svc_error_t qc_svc_handle_request(qc_svc_t *qc_svc_handle, const ble_evt_gatts_write_req_t *evt);
void qc_svc_notify(ble_service_t *svc, uint16_t conn_idx, uint8_t const * const p_data, uint16_t len);
qc_svc_error_t qc_svc_send_read_response(uint8_t conn_idx, qc_svc_error_t status, uint16_t id, uint16_t len, uint8_t * data);
qc_svc_error_t qc_svc_send_write_response(uint8_t conn_idx, qc_svc_error_t status, uint16_t id);
qc_svc_error_t qc_svc_set_transport_mtu(uint16_t mtu);

#endif /* QC_SVC_H_ */
