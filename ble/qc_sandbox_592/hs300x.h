/*
 * hs300x.h
 *
 *  Created on: Aug 17, 2022
 *      Author: a5137667
 */

#ifndef HS300x_H_
#define HS300x_H_

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <ad_i2c.h>
#include <hw_gpio.h>
#include <hw_clk.h>

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/* Definitions of Mask Data for A/D data */
#define HS300x_MASK_HUMIDITY_UPPER_0X3F           (0x3F)
#define HS300x_MASK_TEMPERATURE_LOWER_0XFC        (0xFC)
#define HS300x_MASK_STATUS_0XC0                   (0xC0)

/* Definitions for Status Bits of A/D Data */
#define HS300x_DATA_STATUS_VALID                  (0x00)
#define HS300x_DATA_STATUS_STALE                  (0x01)

/* Definitions for Calculation */
#define HS300x_CALC_14BIT_MAX                     (((1 << 14) - 1))
#define HS300x_CALC_HUMD_VALUE_100                (100.0F)
#define HS300x_CALC_TEMP_C_VALUE_165              (165.0F)
#define HS300x_CALC_TEMP_C_VALUE_40               (40.0F)


/* Definitions for Programming mode */
#define HS300x_PROGRAMMING_MODE_ENTER                   (0xA0)
#define HS300x_PROGRAMMING_MODE_EXIT                    (0x80)
#define HS300x_PROGRAMMING_MODE_SUCCESS_STATUS          (0x81)
#define HS300x_REGISTER_HUMIDITY_RESOLUTION_READ        (0x06)
#define HS300x_REGISTER_HUMIDITY_RESOLUTION_WRITE       (0x46)
#define HS300x_REGISETER_TEMPERATURE_RESOLUTION_READ    (0x11)
#define HS300x_REGISTER_TEMPERATURE_RESOLUTION_WRITE    (0x51)
#define HS300x_REGISTER_SENSOR_ID_UPPER                 (0x1E)
#define HS300x_REGISTER_SENSOR_ID_LOWER                 (0x1F)

/* Definitions of Wait Time */
#define HS300x_DEALY_100_us                             (100)
#define HS300x_DELAY_120_us                             (120)
#define HS300x_DELAY_14_ms                              (14)

#define HS300x_MEASUREMENT_LENGTH_HUMIDITY_AND_TEMP     4
#define HS300x_MEASUREMENT_LENGTH_HUMIDITY_ONLY         2
#define HS300x_READ_REGISTER_RESPONSE_LENGTH            3

#define HS300x_UNKNOWN_SENSOR_ID                        0xFFFFFFFF

#define HS300x_MEASUREMENT_TIME_MARGIN_ms                5
#define HS300x_POWER_UP_DOWN_TIME_MARGIN_ms              2

static const uint8_t enter_programming_mode_cmd[] =     {HS300x_PROGRAMMING_MODE_ENTER, 0, 0};
static const uint8_t exit_programming_mode_cmd[] =      {HS300x_PROGRAMMING_MODE_EXIT, 0, 0};
static const uint8_t read_humidity_resolution_cmd[] =   {HS300x_REGISTER_HUMIDITY_RESOLUTION_READ, 0, 0};
static const uint8_t read_temp_resolution_cmd[] =       {HS300x_REGISETER_TEMPERATURE_RESOLUTION_READ, 0, 0};
static const uint8_t read_sensor_id_upper_cmd[] =       {HS300x_REGISTER_SENSOR_ID_UPPER, 0, 0};
static const uint8_t read_sensor_id_lower_cmd[] =       {HS300x_REGISTER_SENSOR_ID_LOWER, 0, 0};

typedef enum
{
    HS300x_RESOLUTION_8_BITS = 0,
    HS300x_RESOLUTION_10_BITS = 1,
    HS300x_RESOLUTION_12_BITS = 2,
    HS300x_RESOLUTION_14_BITS = 3,
    HS300x_RESOLUTSION_UNKNOWN = 4,
    HS300x_RESOLUTION_MAX = 0xFF,
} hs300x_resolution_t;

typedef enum
{
    HS300x_RESOLUTION_TYPE_HUMIDITY = 0,
    HS300x_RESOLUTION_TYPE_TEMPERATURE = 1,
    HS300x_RESOLUTION_TYPE_MAX = 0xFF,
} hs300x_resolution_type_t;

typedef struct
{
    float humidity_rh_pct;
    float temp_deg_c;
} hs300x_data_t;

typedef struct
{
    ad_i2c_handle_t i2c_handle;          /**< I2C handle for the sensor*/
    gpio_config *power_enable;           /**< GPIO providing power to sensor */
    hs300x_resolution_t humidity_res;    /**< Humidity resolution of sensor */
    hs300x_resolution_t temp_res;        /**< Temperature resolution of sensor*/
} hs300x_handle_t;

/*
 * Error codes. This driver makes use of the I2C adapter and I2C driver. The error codes below map I2C errors to hs300x errors
 */
typedef enum
{
    HS300x_ERROR_I2C_IO_CONFIG_INVALID = AD_I2C_ERROR_IO_CFG_INVALID,
    HS300x_ERROR_I2C_ERROR_CONTROLLER_BUSY     = AD_I2C_ERROR_CONTROLLER_BUSY,
    HS300x_ERROR_I2C_ERROR_DRIVER_CONF_INVALID = AD_I2C_ERROR_DRIVER_CONF_INVALID,
    HS300x_ERROR_I2C_ERROR_HANDLE_INVALID      = AD_I2C_ERROR_HANDLE_INVALID,
    HS300x_ERROR_NONE = AD_I2C_ERROR_NONE,
    HS300x_ERROR_I2C_ABORT_7B_ADDR_NO_ACK = HW_I2C_ABORT_7B_ADDR_NO_ACK,                     /**< address byte of 7-bit address was not acknowledged by any slave */
    HS300x_ERROR_I2C_ABORT_10B_ADDR1_NO_ACK = HW_I2C_ABORT_10B_ADDR1_NO_ACK,                 /**< 1st address byte of the 10-bit address was not acknowledged by any slave */
    HS300x_ERROR_I2C_ABORT_10B_ADDR2_NO_ACK = HW_I2C_ABORT_10B_ADDR2_NO_ACK,                 /**< 2nd address byte of the 10-bit address was not acknowledged by any slave */
    HS300x_ERROR_I2C_ABORT_TX_DATA_NO_ACK = HW_I2C_ABORT_TX_DATA_NO_ACK,                     /**< data were not acknowledged by slave */
    HS300x_ERROR_I2C_ABORT_GENERAL_CALL_NO_ACK = HW_I2C_ABORT_GENERAL_CALL_NO_ACK,           /**< General Call sent but no slave acknowledged */
    HS300x_ERROR_I2C_ABORT_GENERAL_CALL_READ = HW_I2C_ABORT_GENERAL_CALL_READ,               /**< trying to read from bus after General Call */
    HS300x_ERROR_I2C_ABORT_START_BYTE_ACK = HW_I2C_ABORT_START_BYTE_ACK,                     /**< START condition acknowledged by slave */
    HS300x_ERROR_I2C_ABORT_10B_READ_NO_RESTART = HW_I2C_ABORT_10B_READ_NO_RESTART,           /**< read command in 10-bit addressing mode with RESTART disabled */
    HS300x_ERROR_I2C_ABORT_MASTER_DISABLED = HW_I2C_ABORT_MASTER_DISABLED,                   /**< master operation initiated with master mode disabled */
    HS300x_ERROR_I2C_ABORT_ARBITRATION_LOST = HW_I2C_ABORT_ARBITRATION_LOST,                 /**< bus arbitration lost */
    HS300x_ERROR_I2C_ABORT_SLAVE_FLUSH_TX_FIFO = HW_I2C_ABORT_SLAVE_FLUSH_TX_FIFO,           /**< (slave mode) request for data with data already in TX FIFO - used to flush data in TX FIFO */
    HS300x_ERROR_I2C_ABORT_SLAVE_ARBITRATION_LOST = HW_I2C_ABORT_SLAVE_ARBITRATION_LOST,     /**< (slave mode) bus lost when transmitting to master */
    HS300x_ERROR_I2C_ABORT_SLAVE_IN_TX = HW_I2C_ABORT_SLAVE_IN_TX,                           /**< (slave mode) request for data replied with read request */
    HS300x_ERROR_I2C_ABORT_SW_ERROR = HW_I2C_ABORT_SW_ERROR,
	HS300x_ERROR_DATA_ACCESS_FAIL,
} hs300x_error_t;

void hs300x_close(hs300x_handle_t* hs300x_handle);
hs300x_error_t hs300x_enter_programming_mode(hs300x_handle_t* hs300x_handle);
hs300x_error_t hs300x_exit_programming_mode(hs300x_handle_t* hs300x_handle);
hs300x_error_t hs300x_get_measurement(hs300x_handle_t* hs300x_handle, bool data_includes_temp, hs300x_data_t *calculated_data);
hs300x_error_t hs300x_get_resolution(hs300x_handle_t* hs300x_handle, hs300x_resolution_type_t type, hs300x_resolution_t *resolution);
hs300x_error_t hs300x_get_sensor_id(hs300x_handle_t* hs300x_handle, uint32_t *id);
ad_i2c_handle_t hs300x_open(const ad_i2c_controller_conf_t *i2c_conf);
void hs300x_power_cycle_sensor(gpio_config power_enable);
hs300x_error_t hs300x_read(hs300x_handle_t* hs300x_handle, uint8_t *response_buffer, size_t response_length);
hs300x_error_t hs300x_set_resolution(hs300x_handle_t* hs300x_handle, hs300x_resolution_t resolution, hs300x_resolution_type_t type);
hs300x_error_t hs300x_start_measurement(hs300x_handle_t* hs300x_handle);
hs300x_error_t hs300x_write(hs300x_handle_t* hs300x_handle, const uint8_t *write_buffer, size_t write_length);

#endif /* HS300x_H_ */
