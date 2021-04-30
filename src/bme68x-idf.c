/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>>

#include "esp_system.h"
#include "sdkconfig.h"

#include "driver/i2c.h"
#include "driver/spi_master.h"

#include "i2cdev.h"
#include "bme68x-idf.h"
#include "bme68x.h"

/******************************************************************************/
/*!                 Macro definitions                                         */
#define BME_VCC_GPIO     CONFIG_BME68X_POWER_GPIO
#define BME_VCC_GPIO_SEL (1ULL << CONFIG_BME68X_POWER_GPIO)

#define BME_I2C_PORT CONFIG_BME68X_I2C_PORT_NUM
#define BME_I2C_ADDR CONFIG_BME68X_I2C_ADDR
#define BME_I2C_FREQ (CONFIG_BME68X_I2C_FREQ_KHZ * 1000)
#define BME_I2C_SDA  CONFIG_BME68X_I2C_SDA_GPIO
#define BME_I2C_SCL  CONFIG_BME68X_I2C_SCL_GPIO


/******************************************************************************/
/*!                Static variable definition                                 */
static i2c_dev_t bme_dev;
static const char *TAG = "bme68x";

/******************************************************************************/
/*!                User interface functions                                   */

/**
 * @brief i2c read function map for BSEC 2.0
 *
 * @param reg_addr  register address
 * @param reg_data  where to send data
 * @param len       how many bytes?
 * @param intf_ptr  pointer to i2c_dev_t
 * @return esp_err_t
 */
esp_err_t bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_dev_t *bme_dev = (i2c_dev_t *)intf_ptr;
    I2C_DEV_TAKE_MUTEX(bme_dev);
    I2C_DEV_CHECK(bme_dev, i2c_dev_read_reg(bme_dev, reg_addr, reg_data, len));
    I2C_DEV_GIVE_MUTEX(bme_dev);
    return ESP_OK;
}

/**
 * @brief i2c write function map for BSEC 2.0
 *
 * @param reg_addr  register address
 * @param reg_data  data to write
 * @param len       length of data
 * @param intf_ptr  pointer to i2c_dev_t
 * @return esp_err_t
 */
esp_err_t bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_dev_t *bme_dev = (i2c_dev_t *)intf_ptr;
    I2C_DEV_TAKE_MUTEX(bme_dev);
    I2C_DEV_CHECK(bme_dev, i2c_dev_write_reg(bme_dev, reg_addr, reg_data, len));
    I2C_DEV_GIVE_MUTEX(bme_dev);
    return ESP_OK;
}

/**
 * @brief microsecond (heh) delay map for BSEC 2.0
 *        caveat: is actually rounded up to nearest tick, recommend using 1000Hz tickrate
 *
 * @param period    delay period in microseconds
 * @param intf_ptr  device pointer, for, some reason. ask Bosch.
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    vTaskDelay(pdMS_TO_TICKS((period + 999) / 1000));
}

/**
 * @brief BSEC 2.0 API error code translator/logger
 *
 * @param api_name  string containing API name
 * @param rslt      error code
 */
void bme68x_check_rslt(const char api_name[], int8_t rslt) {
    switch (rslt) {
        case BME68X_OK: break;
        case BME68X_E_NULL_PTR: ESP_LOGE(TAG, "API [%s] Error [%d]: Null pointer exception", api_name, rslt); break;
        case BME68X_E_COM_FAIL: ESP_LOGE(TAG, "API [%s] Error [%d]: Communication failure", api_name, rslt); break;
        case BME68X_E_INVALID_LENGTH: ESP_LOGE(TAG, "API [%s] Error [%d]: Invalid length", api_name, rslt); break;
        case BME68X_E_DEV_NOT_FOUND: ESP_LOGE(TAG, "API [%s] Error [%d]: Device not found", api_name, rslt); break;
        case BME68X_E_SELF_TEST: ESP_LOGE(TAG, "API [%s] Error [%d]: Self test returned error", api_name, rslt); break;
        case BME68X_W_NO_NEW_DATA: ESP_LOGW(TAG, "API [%s] Warning [%d]: No new data found", api_name, rslt); break;
        default: ESP_LOGE(TAG, "API [%s] Error [%d]: Unknown error code", api_name, rslt); break;
    }
}

/**
 * @brief bme68x interface init
 *
 * @param bme           device instance from/for BSEC 2.0
 * @param intf          interface type, this will always be I2C for now because SPI is difficult
 * @return esp_err_t    error code
 */
esp_err_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf) {
    // make sure bme is not null and intf mode is I2C
    if (bme == NULL || intf != BME68X_I2C_INTF) return ESP_ERR_INVALID_ARG;
    esp_err_t err = ESP_OK;

#if CONFIG_BME68X_USE_GPIO_POWER
    // configure and enable VCC_BME
    static const gpio_config_t bme68x_vcc_conf = {
        .pin_bit_mask = BME_VCC_GPIO_SEL,
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&bme68x_vcc_conf));
    gpio_set_level(BME_VCC_GPIO, 1);
    ESP_LOGD(TAG, "VCC GPIO configured, sensor powered on");
#endif

    // allocate some memory for the i2c_dev_t
    i2c_dev_t *dev = heap_caps_malloc(sizeof(i2c_dev_t), MALLOC_CAP_8BIT);
    if (!dev) return ESP_ERR_NO_MEM;
    memset(&dev, 0, sizeof(i2c_dev_t));

    // configure the i2c_dev_t
    dev->port                 = BME_I2C_PORT;
    dev->addr                 = BME_I2C_ADDR;
    dev->cfg.sda_io_num       = BME_I2C_SDA;
    dev->cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    dev->cfg.scl_io_num       = BME_I2C_SCL;
    dev->cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    dev->cfg.master.clk_speed = BME_I2C_FREQ;

    err = i2c_dev_create_mutex(&dev->i2c_dev);
    if (err != ESP_OK) return err;

    // save details in the bme struct
    bme->intf     = BME68X_I2C_INTF;
    bme->intf_ptr = dev;
    bme->delay_us = bme68x_delay_us;
    bme->read     = bme68x_i2c_read;
    bme->write    = bme68x_i2c_write;
    // ambient temp is used to calibrate heater temp, so it doesn't need to be super accurate.
    bme->amb_temp = 25;

    return ESP_OK;
}

/**
 * @brief bme68x interface deinit
 *
 * @param bme   bme68x_dev to clear
 * @return esp_err_t error code
 */
esp_err_t bme68x_interface_deinit(struct bme68x_dev *bme) {
    esp_err_t err = ESP_OK;
    err = i2c_dev_delete_mutex(bme->intf_ptr));
    if (err != ESP_OK) return err;
    free(bme->intf_ptr);

#if CONFIG_BME68X_USE_GPIO_POWER
    err = gpio_set_level(BME_VCC_GPIO, 0);
    ESP_LOGD(TAG, "VCC GPIO , sensor powered off");
#endif

    return err;
}
