/*
 * hs300x.c
 *
 *  Created on: Aug 17, 2022
 *      Author: a5137667
 */
#include "hs300x.h"

/* Private function prototypes */
static float calc_measurement_time(hs300x_resolution_t humidity_res, hs300x_resolution_t temp_res);
static void convert_raw_to_humid_temp(uint8_t *raw_data, bool data_includes_temp, hs300x_data_t *calculated_data);
static float measurement_time_from_resolution(hs300x_resolution_t res);
static hs300x_error_t send_programming_mode_enter(hs300x_handle_t* hs300x_handle);

/**
 * \brief Calculate the time for a measurement based on the resolution settings
 *
 * \param[in] humidity_res      resolution of humidity data to be measured
 * \param[in] temp_res          resolution of temperature data to be measured
 *
 * \return                      the measurement time in milliseconds
 *
 *  \note
 *  See Section 6.6: Data Fetch of the datasheet. Be aware the times returned are typical, not maximum
 *  The time returned includes measurement time for both humidty and termperature plus the wakeup time
 *  of the sensor
 */
static float calc_measurement_time(hs300x_resolution_t humidity_res, hs300x_resolution_t temp_res)
{
    return (0.1 + measurement_time_from_resolution(humidity_res) + measurement_time_from_resolution(temp_res));
}

/**
 * \brief Close the I2C controller for the HS300x
 *
 * \param[in] i2c_handle        I2C handle of the controller to close
 *
 * \return void
 *
 */
static void convert_raw_to_humid_temp(uint8_t *raw_data, bool data_includes_temp, hs300x_data_t *calculated_data)
{
    float humidity = ((raw_data[0] & HS300x_MASK_HUMIDITY_UPPER_0X3F) << 8) | raw_data[1];
    calculated_data->humidity_rh_pct = (humidity * HS300x_CALC_HUMD_VALUE_100) / (HS300x_CALC_14BIT_MAX);

    if(data_includes_temp)
    {
        float temp = ((raw_data[2] << 8) | (raw_data[3] & HS300x_MASK_TEMPERATURE_LOWER_0XFC)) >> 2;
        calculated_data->temp_deg_c = (temp * HS300x_CALC_TEMP_C_VALUE_165)/(HS300x_CALC_14BIT_MAX) - HS300x_CALC_TEMP_C_VALUE_40;
    }
}

/**
 * \brief Extract the port/pin from a gpio_config
 *
 * \param[in] config        gpio_config to extract information from
 * \param[out] port         pointer where port information will be placed
 * \param[out] pin        	pointer where pin information will be placed
 *
 * \return void
 *
 * \sa ad_i2c_close()
 *
 */
static void gpio_config_to_port_and_pin(gpio_config config, HW_GPIO_PORT *port, HW_GPIO_PIN *pin)
{
    *port = (config.pin & 0x3F) >> HW_GPIO_PIN_BITS;
    *pin = (config.pin) & ((1 << HW_GPIO_PIN_BITS) - 1);
}

/**
 * \brief Close the I2C controller for the HS300x
 *
 * \param[in] i2c_handle        I2C handle of the controller to close
 *
 * \return void
 *
 * \sa ad_i2c_close()
 *
 */
void hs300x_close(hs300x_handle_t* hs300x_handle)
{
    ASSERT_WARNING(hs300x_handle->i2c_handle)
    hs300x_error_t error = ad_i2c_close(hs300x_handle->i2c_handle, true);
    ASSERT_ERROR(error == HS300x_ERROR_NONE);
}

/**
 * \brief Put the HS300x into programming mode
 *
 * \param[in] i2c_handle        I2C handle of the HS300x
 *
 * \return error indicating status of command
 *
 * \note
 * See section 6.8. The command to enter programming mode must be sent within 10ms of chip power up.
 */
hs300x_error_t hs300x_enter_programming_mode(hs300x_handle_t* hs300x_handle)
{
    hs300x_power_cycle_sensor(hs300x_handle->power_enable[0]);
    return send_programming_mode_enter(hs300x_handle);
}

/**
 * \brief Send the command to take the HS300x out of programming mode
 *
 * \param[in] i2c_handle        I2C handle of the HS300x
 *
 * \return error indicating status of command
 *
 */
hs300x_error_t hs300x_exit_programming_mode(hs300x_handle_t* hs300x_handle)
{
    return  hs300x_write(hs300x_handle, exit_programming_mode_cmd, sizeof(exit_programming_mode_cmd));
}

/**
 * \brief Get a measurement. This function will start the measurement (see Section 6.5), wait for the appropriate
 * amount of time for the measurement to complete (plus some additional margin), and convert the raw value to
 * relative humidity percentage and degrees C per Section 7.
 *
 * \param[in] hs300x_handle             handle of the HS300x
 * \param[in] data_includes_temp        a boolean value indicating if the measurement should include temperature data
 * \param[out] calculated_data          a pointer to a buffer where the data will be placed
 *
 * \return error indicating status of the operation
 *
 */
hs300x_error_t hs300x_get_measurement(hs300x_handle_t* hs300x_handle, bool data_includes_temp, hs300x_data_t *calculated_data)
{
    hs300x_error_t error = hs300x_start_measurement(hs300x_handle);
    if(error == HS300x_ERROR_NONE)
    {
        float measurement_time = calc_measurement_time(hs300x_handle->humidity_res, hs300x_handle->temp_res);
        // Round measurement time up and add some additional delay to account for worst case measurement time
        uint32_t measurement_delay = HS300x_MEASUREMENT_TIME_MARGIN_ms + ((uint32_t)measurement_time + 1);
        OS_DELAY_MS(measurement_delay);

        uint8_t response[HS300x_MEASUREMENT_LENGTH_HUMIDITY_AND_TEMP] = {0};
        // TODO handle 8 bit temperature (e.g. len == 3)
        uint8_t len = data_includes_temp ? HS300x_MEASUREMENT_LENGTH_HUMIDITY_AND_TEMP : HS300x_MEASUREMENT_LENGTH_HUMIDITY_ONLY;
        error = hs300x_read(hs300x_handle, response, len);

        if(error == HS300x_ERROR_NONE)
        {
            convert_raw_to_humid_temp(response, data_includes_temp, calculated_data);
        }
    }

    return error;
}

/**
 * \brief Get the current resolution for humidity or temperature
 *
 * \param[in] i2c_handle        I2C handle of the HS300x
 * \param[in] type              which resolution to get (humidity or temperature)
 * \param[out] resolution       a buffer where the resolution data will be placed
 *
 * \return error indicating status of the operation
 *
 * \note The sensor must be in programming mode to access Non-volatile memory. See section 6.8
 */
hs300x_error_t hs300x_get_resolution(hs300x_handle_t* hs300x_handle, hs300x_resolution_type_t type, hs300x_resolution_t *resolution) // TODO do not need separate resolution. Just update the handle
{
    const uint8_t *cmd = type == HS300x_RESOLUTION_TYPE_HUMIDITY ? read_humidity_resolution_cmd : read_temp_resolution_cmd;
    hs300x_error_t error = hs300x_write(hs300x_handle, cmd, sizeof(read_humidity_resolution_cmd));
    // cmd takes 120us to process
    hw_clk_delay_usec(HS300x_DELAY_120_us);

    if(error == HS300x_ERROR_NONE)
    {
        uint8_t response[3] = {0};
        error = hs300x_read(hs300x_handle, response, sizeof(response));

        if(error == HS300x_ERROR_NONE)
        {
            if(response[0] == HS300x_PROGRAMMING_MODE_SUCCESS_STATUS)
            {
                *resolution = ((response[1] & 0x0C) >> 2); // bits 11 and 10. TODO create masks
            }
            else
            {
            	error = HS300x_ERROR_DATA_ACCESS_FAIL;
            }
        }
    }

    return error;
}

/**
 * \brief Get the sensor ID of the HS300x
 *
 * \param[in] i2c_handle        I2C handle of the HS300x
 * \param[out] id               a buffer where the sensor ID will be placed
 *
 * \return error indicating status of the operation
 *
 * \note The sensor must be in programming mode to access Non-volatile memory. See section 6.8
 */
hs300x_error_t hs300x_get_sensor_id(hs300x_handle_t *hs300x_handle, uint32_t* id)
{
   hs300x_error_t error = hs300x_write(hs300x_handle, read_sensor_id_upper_cmd, sizeof(read_sensor_id_upper_cmd));
   // cmd takes 120us to process
   hw_clk_delay_usec(HS300x_DELAY_120_us);

   if(error == HS300x_ERROR_NONE)
   {
        uint8_t upper_rsp[3] = {0};
        error = hs300x_read(hs300x_handle, upper_rsp, sizeof(upper_rsp));

        if(error == HS300x_ERROR_NONE && upper_rsp[0] == HS300x_PROGRAMMING_MODE_SUCCESS_STATUS)
        {
            error = hs300x_write(hs300x_handle, read_sensor_id_lower_cmd, sizeof(read_sensor_id_lower_cmd));
            // cmd takes 120us to process. See section 6.8
            hw_clk_delay_usec(HS300x_DELAY_120_us);

            if(error == HS300x_ERROR_NONE)
            {
                uint8_t lower_rsp[3] = {0};
                error = hs300x_read(hs300x_handle, lower_rsp, sizeof(lower_rsp));

                if(error == HS300x_ERROR_NONE && lower_rsp[0] == HS300x_PROGRAMMING_MODE_SUCCESS_STATUS)
                {
                    *id = (upper_rsp[1] << 24) | (upper_rsp[2] << 16) | (lower_rsp[1] << 8) | lower_rsp[2];
                }
                else if (error == HS300x_ERROR_NONE)
                {
                	error = HS300x_ERROR_DATA_ACCESS_FAIL;
                }
            }
        }
        else if (error == HS300x_ERROR_NONE)
        {
        	 error = HS300x_ERROR_DATA_ACCESS_FAIL;
        }
    }

    return error;
}

/**
 * \brief Open the I2C controller for the HS300x
 *
 * \param[in] i2c_conf          pointer to the configuration of the I2C controller to open
 *
 * \return >0: non-NULL handle that should be used in subsequent API calls, NULL: error
 *
 * \sa ad_i2c_open()
 *
 */
ad_i2c_handle_t hs300x_open(const ad_i2c_controller_conf_t *i2c_conf)
{
    ad_i2c_handle_t sensor_i2c_handle = ad_i2c_open(i2c_conf);
    ASSERT_ERROR(sensor_i2c_handle);

    return sensor_i2c_handle;
}

/**
 * \brief Power cycle the HS300x
 *
 * \param[in] power_enable      GPIO config for the pin used to power the HS300x
 *
 * \return void
 *
 * \note
 * This function assumes the HS300x is powered from a GPIO which can be toggled power cycle the sensor
 *
 * \sa hs300x_send_programming_mode_enter()
 */
void hs300x_power_cycle_sensor(gpio_config power_enable)
{
	// Get the port,pin for the power_enable pin
    HW_GPIO_PORT port;
    HW_GPIO_PIN pin;
    gpio_config_to_port_and_pin(power_enable, &port, &pin);

    hw_gpio_set_inactive(port, pin);

    // give sensor some time to power off
    OS_DELAY_MS(HS300x_POWER_UP_DOWN_TIME_MARGIN_ms);

    hw_gpio_set_active(port, pin);

    // give sensor some time to power on
    OS_DELAY_MS(HS300x_POWER_UP_DOWN_TIME_MARGIN_ms);
}


/**
 * \brief Read data from the HS300x
 *
 * \param[in] i2c_handle        I2C handle for the HS300x
 * \param[out] response_buffer  pointer to a buffer where the data will be placed
 * \param[in] response_length   number of bytes to read
 *
 * \return error code indicating status of operation
 *
 */
hs300x_error_t hs300x_read(hs300x_handle_t* hs300x_handle, uint8_t *response_buffer, size_t response_length)
{
    return ad_i2c_read(hs300x_handle->i2c_handle, response_buffer, response_length, HW_I2C_F_ADD_STOP);
}

/* \brief Convenience function to convert hs300x_resolution_t to a string
 *
 * \param[in] res       Resolution to convert
 *
 * \return a string with the corresponding resolution
 *
 *\note The sensor must be in programming mode to access Non-volatile memory. See section 6.8
 */
hs300x_error_t hs300x_set_resolution(hs300x_handle_t* hs300x_handle, hs300x_resolution_t resolution, hs300x_resolution_type_t type)
{
    ASSERT_ERROR(resolution <= HS300x_RESOLUTION_14_BITS);

    const uint8_t *cmd = (type == HS300x_RESOLUTION_TYPE_HUMIDITY ? read_humidity_resolution_cmd : read_temp_resolution_cmd);

    hs300x_error_t error = hs300x_write(hs300x_handle, cmd, sizeof(read_humidity_resolution_cmd));
    // command takes 120us to process
    hw_clk_delay_usec(HS300x_DELAY_120_us);

    if(error == HS300x_ERROR_NONE)
    {
        uint8_t response[3] = {0};
        error = hs300x_read(hs300x_handle, response, sizeof(response));

        if(error == HS300x_ERROR_NONE )
        {
            if(response[0] == HS300x_PROGRAMMING_MODE_SUCCESS_STATUS)
            {
            	// The measurement resolution is stored in bits [11:10]. See datasheet section 6.9 Setting the Measurement Resolution, step 3.
            	// note response[1] is the MSB of the register
                response[1] &= ~(0x0C); // clear bits 11,10
                response[1] |= (resolution << 2); // set bits 11,10
                uint8_t res_type = type == HS300x_RESOLUTION_TYPE_HUMIDITY ? HS300x_REGISTER_HUMIDITY_RESOLUTION_WRITE : HS300x_REGISTER_TEMPERATURE_RESOLUTION_WRITE;
                uint8_t new_resolution_cmd[] = {res_type, response[1] , response[2]};
                error = hs300x_write(hs300x_handle, new_resolution_cmd, sizeof(new_resolution_cmd));
                // update takes 14ms, see datasheet section 6.9
                OS_DELAY_MS(HS300x_DELAY_14_ms);

                if(error == HS300x_ERROR_NONE)
                {
                	hs300x_resolution_t* new_resolution = (type == HS300x_RESOLUTION_TYPE_HUMIDITY) ? &hs300x_handle->humidity_res : &hs300x_handle->temp_res;
                	*new_resolution = resolution;
                }
            }
            else
            {
                    error = HS300x_ERROR_DATA_ACCESS_FAIL;
            }
        }
    }

    return error;
}

/**
 * \brief Start a measurement
 *
 * \param[in] i2c_handle I2C handle for the sensor
 *
 * \return error code indicating status of write
 */
hs300x_error_t hs300x_start_measurement(hs300x_handle_t* hs300x_handle)
{
    // See section 6.5
    // need to send i2c address to start a measurement.
    // SDK does will return error if attempt to send 0 bytes over i2c, so instead send a zeroed byte
    uint8_t zero[] = {0};
    return hs300x_write(hs300x_handle, zero, sizeof(zero));
}

/**
 * \brief Write data to HS300x
 *
 * \param[in] i2c_handle        I2C handle for the sensor
 * \param[in] write_buffer      Buffer with data to be written
 * \param[in] write_length      Length of data to be written
 *
 * \return error code indicating status of write
 */
hs300x_error_t hs300x_write(hs300x_handle_t* hs300x_handle, const uint8_t *write_buffer, size_t write_length)
{
    return ad_i2c_write(hs300x_handle->i2c_handle, write_buffer, write_length, HW_I2C_F_ADD_STOP);
}

/**
 * \brief Get time for measurement based on the data resolution
 *
 * \param[in] res       resolution of data to be measured
 *
 * \return the measurement time in milliseconds
 *
 * \note
 *  See datasheet Table 3. Be aware the times returned are typical, not maximum
 */
static float measurement_time_from_resolution(hs300x_resolution_t res)
{
    float time_ms = 0;
    switch(res)
    {
        case HS300x_RESOLUTION_8_BITS:
            time_ms = 0.55;
            break;
        case HS300x_RESOLUTION_10_BITS:
            time_ms = 1.31;
            break;
        case HS300x_RESOLUTION_12_BITS:
            time_ms = 4.5;
            break;
        case HS300x_RESOLUTION_14_BITS:
            time_ms = 16.9;
            break;
        default:
            ASSERT_ERROR(0);
            break;
    }

    return time_ms;
}

/**
 * \brief Send the command to put the HS300x into programming mode
 *
 * \param[in] hs300x_handle             handle of the HS300x
 *
 * \return error indicating status of command
 *
 * \sa hs300x_power_cycle_sensor()
 */
static hs300x_error_t send_programming_mode_enter(hs300x_handle_t* hs300x_handle)
{
    hs300x_error_t error = hs300x_write(hs300x_handle, enter_programming_mode_cmd, sizeof(enter_programming_mode_cmd));
    // cmd takes 120us to process. See section 6.8
    hw_clk_delay_usec(HS300x_DELAY_120_us);

    return error;
}
