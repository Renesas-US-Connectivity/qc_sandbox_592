/*
 * hs3001_task.c
 *
 *  Created on: Aug 15, 2022
 *      Author: a5137667
 */
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "osal.h"
#include "resmgmt.h"
#include "hw_gpio.h"
#include "sys_clock_mgr.h"
#include "sys_power_mgr.h"

#include "hs300x_task.h"
#include <ad_i2c.h>
#include "hs300x.h"
#include "platform_devices.h"

/* Private function prototypes */
static void hs300x_handle_init();
static const char * hs300x_resolution_to_string(hs300x_resolution_t res);
static hs300x_error_t perform_measurement(hs300x_data_t *sample);
static void process_measurement(hs300x_data_t sample);

/* Private variables */
__RETAINED_RW static hs300x_handle_t hs300x_handle = {0};
__RETAINED_RW static uint32_t sensor_id = 0xFFFFFFFF;
__RETAINED_RW static uint32_t sample_rate_ms = 1000;
__RETAINED_RW static OS_MUTEX sample_rate_mutex = NULL;
__RETAINED_RW static OS_QUEUE sample_q = NULL;
__RETAINED_RW static OS_TASK measurement_notification_task = NULL;

// See hs300x_resolution_t for resolution options
hs300x_resolution_t user_humidity_resolution = HS300x_RESOLUTION_14_BITS;
hs300x_resolution_t user_temperature_resolution = HS300x_RESOLUTION_14_BITS;

/**
 * \brief Initialize hs300x_handle
 *
 * \return void
 */
static void hs300x_handle_init()
{
    hs300x_handle.i2c_handle = NULL;
    hs300x_handle.power_enable = hs300x_power_gpio;
    hs300x_handle.humidity_res = user_humidity_resolution;
    hs300x_handle.temp_res = user_temperature_resolution;
}

/**
 * \brief Convenience function to convert hs300x_resolution_t to a string
 *
 * \param[in] res       Resolution to convert
 *
 * \return a string with the corresponding resolution
 *
 */
static const char * hs300x_resolution_to_string(hs300x_resolution_t res)
{
    if(res == HS300x_RESOLUTION_8_BITS)
        return "8 bits";
    else if (res == HS300x_RESOLUTION_10_BITS)
        return "10 bits";
    else if (res == HS300x_RESOLUTION_12_BITS)
        return "12 bits";
    else if (res == HS300x_RESOLUTION_14_BITS)
        return "14 bits";
    else
        return "invalid resolution";
}

/**
 * \brief HS300x sampling task. This task will read the sensor ID, set the measurement
 * resolution for both humidity and temperature to the user defined values set in
 * user_humidity_resolution and user_temperature_resolution respectively. Then a
 * measurement will be taken at a rate of sample_rate_ms. The results will be printed to the terminal
 * and put on queue
 *
 * \param[in] pvParameters      Used to pass in a queue for measurements from the sensor
 *
 * \return void
 */
void hs300x_task(void *pvParameters)
{
    sample_q = (OS_QUEUE)pvParameters;

    printf("Starting HS300x task...\r\n");
    OS_MUTEX_CREATE(sample_rate_mutex);

    // enable power and open the I2C port
    hs300x_power_cycle_sensor(hs300x_handle.power_enable[0]);
    hs300x_handle.i2c_handle = hs300x_open(hs300x_i2c_config);

    // Enter programming mode. Note programming mode must be entered within
    // 10ms of the HS300x powering up. See section 6.8 of the datasheet.
    // Enter programming mode to:
    // 1. Retrieve Sensor ID
    // 2. Set Humidity / Temperature Resolution

    hs300x_error_t  error = hs300x_enter_programming_mode(&hs300x_handle);

    // Get the Sensor ID

    error = hs300x_get_sensor_id(&hs300x_handle, &sensor_id);
    ASSERT_ERROR(error == HS300x_ERROR_NONE);

    printf("HS300x Sensor ID: %08lX\r\n", sensor_id);

    // Set the humidity resolution
    error = hs300x_set_resolution(&hs300x_handle, user_humidity_resolution, HS300x_RESOLUTION_TYPE_HUMIDITY);
    ASSERT_ERROR(error == HS300x_ERROR_NONE);

    // Set the temperature resolution
    error = hs300x_set_resolution(&hs300x_handle, user_temperature_resolution, HS300x_RESOLUTION_TYPE_TEMPERATURE);
    ASSERT_ERROR(error == HS300x_ERROR_NONE);

    // Read back the humidity resolution
    hs300x_resolution_t humidity_resolution;
    error = hs300x_get_resolution(&hs300x_handle, HS300x_RESOLUTION_TYPE_HUMIDITY, &humidity_resolution);
    ASSERT_ERROR(error == HS300x_ERROR_NONE);

    // Read back the temperature resolution
    hs300x_resolution_t temp_resolution;
    error = hs300x_get_resolution(&hs300x_handle, HS300x_RESOLUTION_TYPE_TEMPERATURE,  &temp_resolution);
    ASSERT_ERROR(error == HS300x_ERROR_NONE);

    printf("Humidity Resolution: %s. Temperature Resolution: %s\r\n", hs300x_resolution_to_string(humidity_resolution), hs300x_resolution_to_string(temp_resolution));

    // Exit programming mode
    error = hs300x_exit_programming_mode(&hs300x_handle);
    ASSERT_ERROR(error == HS300x_ERROR_NONE);

    for(;;)
    {
        // Perform a measurement
        hs300x_data_t sample = {0};
        hs300x_error_t error = perform_measurement(&sample);
        if(error == HS300x_ERROR_NONE)
        {
            process_measurement(sample);
        }
        else
        {
            printf("Error performing measurement: error=%d\r\n", error);
        }

        // Delay for sample_rate_ms
        vTaskDelay(OS_MS_2_TICKS(hs300x_task_get_sample_rate()));
    }
}

/**
 * \brief Register for notifications every time a new measurement is taken
 *
 * \param[in] task_handle       handle of the task to be notified
 *
 * \return void
 *
 */
void hs300x_task_event_queue_register(const OS_TASK task_handle)
{
    // Set event queue task handle
    measurement_notification_task = task_handle;
}

/**
 * \brief Get the sample rate for taking a measurement
 *
 * \return sample rate in milliseconds
 *
 */
uint32_t hs300x_task_get_sample_rate()
{
    uint32_t temp = 0;
    OS_MUTEX_GET(sample_rate_mutex, OS_MUTEX_FOREVER);
    temp = sample_rate_ms;
    OS_MUTEX_PUT(sample_rate_mutex);
    return temp;
}

/**
 * \brief Get the Sensor ID
 *
 * \return 4 byte Sensor ID
 *
 */
uint32_t hs300x_task_get_sensor_id()
{
    return sensor_id;
}

/**
 * \brief Set the sample rate for taking a measurement
 *
 * \param[in] rate       new sample rate in milliseconds
 *
 * \return void
 *
 */
void hs300x_task_set_sample_rate(uint32_t rate)
{
    OS_MUTEX_GET(sample_rate_mutex, OS_MUTEX_FOREVER);
    sample_rate_ms = rate;
    OS_MUTEX_PUT(sample_rate_mutex);
}

/**
 * \brief Setup GPIO for interacting with the HS300x
 *
 * \return void
 */
void hs300x_task_setup_hardware()
{
    hs300x_handle_init();

    hw_sys_pd_com_enable();

    ad_i2c_io_config(hs300x_i2c_config->id,
                     hs300x_i2c_config->io, AD_IO_CONF_ON);
    ad_i2c_io_config(hs300x_i2c_config->id,
                     hs300x_i2c_config->io, AD_IO_CONF_ON);

    hw_gpio_configure(hs300x_handle.power_enable);
    hw_gpio_pad_latch_enable(HS300x_POWER_GPIO_PORT,HS300x_POWER_GPIO_PIN);

    hw_sys_pd_com_disable();
}

/**
 * \brief Take a measurement from the HS300x
 *
 * \param[in] sample       buffer to place measurement
 *
 * \return void
 */
static hs300x_error_t perform_measurement(hs300x_data_t *sample)
{
    return hs300x_get_measurement(&hs300x_handle, true, sample);
}

/**
 * \brief Process a measurement from the HS300x.
 *
 * \param[in] sample       sample to process
 *
 * \return void
 */
static void process_measurement(hs300x_data_t sample)
{
    printf("Sample Rate (ms): %ld, Humidity (%%RH): %.3f, Temp (C): %.3f\r\n", hs300x_task_get_sample_rate(), sample.humidity_rh_pct, sample.temp_deg_c);

    OS_QUEUE_PUT(sample_q, &sample, OS_QUEUE_NO_WAIT);
    if(measurement_notification_task)
    	OS_TASK_NOTIFY(measurement_notification_task, HS3001_MEASUREMENT_NOTIFY_MASK, OS_NOTIFY_SET_BITS);
}
