/*
 * platform_devices.c
 *
 *  Created on: Sep 8, 2022
 *      Author: a5137667
 */

#include "platform_devices.h"


/* I2C I/O configuration */
const ad_i2c_io_conf_t i2c_io = {
    .scl = {
        .port = I2C_PORT, .pin = I2C_MASTER_SCL_PIN,
        .on =  { HW_GPIO_MODE_OUTPUT_OPEN_DRAIN, HW_GPIO_FUNC_I2C_SCL, false },
        .off = { HW_GPIO_MODE_INPUT,             HW_GPIO_FUNC_GPIO,    true  }
    },
    .sda = {
        .port = I2C_PORT, .pin = I2C_MASTER_SDA_PIN,
        .on =  { HW_GPIO_MODE_OUTPUT_OPEN_DRAIN, HW_GPIO_FUNC_I2C_SDA, false },
        .off = { HW_GPIO_MODE_INPUT,             HW_GPIO_FUNC_GPIO,    true  }
    },
};

/*
 * PLATFORM PERIPHERALS CONTROLLER CONFIGURATION
 *****************************************************************************************
 */

const ad_i2c_driver_conf_t i2c_driver_config = {
     I2C_DEFAULT_CLK_CFG,
    .i2c.speed = HW_I2C_SPEED_STANDARD,
    .i2c.mode = HW_I2C_MODE_MASTER,
    .i2c.addr_mode = HW_I2C_ADDRESSING_7B,
    .i2c.address = I2C_SLAVE_ADDRESS,
    .i2c.event_cb = NULL,
    //.dma_channel = HW_DMA_CHANNEL_INVALID
    /**
     * I2C master is not configured to use DMA because in case we are connecting the
     * same board in loopback, the DMA controller will block in case there are blocking
     * transactions being handled from both the I2C master and slave controller at the same time.
     *
     * In case we are connecting two boards we can use DMA for the master as well.
     */
};



/* I2C controller configuration */
const ad_i2c_controller_conf_t hs300x_i2c_controller_config = {
    .id = I2C_CTRLR_INSTANCE,
    .io = &i2c_io,
    .drv = &i2c_driver_config
};

/*
 * Output GPIO pins configuration array.
 *
 */
static gpio_config hs300x_gpio[] = {

    HW_GPIO_PINCONFIG(HS300x_POWER_GPIO_PORT, HS300x_POWER_GPIO_PIN, OUTPUT, GPIO, true),

    HW_GPIO_PINCONFIG_END
};

i2c_device hs300x_i2c_config = &hs300x_i2c_controller_config;

gpio_config *hs300x_power_gpio = hs300x_gpio;

