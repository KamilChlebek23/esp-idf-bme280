#include "bme280.h"
#include "freertos/timers.h"
#include "esp_check.h"

#define BME280_REG_TEMP_XLSB ((uint8_t)0xFC)
#define BME280_REG_TEMP_LSB  ((uint8_t)0xFB)
#define BME280_REG_TEMP_MSB  ((uint8_t)0xFA)
#define BME280_REG_DIG_T1    ((uint8_t)0x88)
#define BME280_REG_DIG_T2    ((uint8_t)0x8A)
#define BME280_REG_DIG_T3    ((uint8_t)0x8C)

#define BME280_REG_PRESS_XLSB ((uint8_t)0xF9)
#define BME280_REG_PRESS_LSB  ((uint8_t)0xF8)
#define BME280_REG_PRESS_MSB  ((uint8_t)0xF7)
#define BME280_REG_DIG_P1     ((uint8_t)0x8E)
#define BME280_REG_DIG_P2     ((uint8_t)0x90)
#define BME280_REG_DIG_P3     ((uint8_t)0x92)
#define BME280_REG_DIG_P4     ((uint8_t)0x94)
#define BME280_REG_DIG_P5     ((uint8_t)0x96)
#define BME280_REG_DIG_P6     ((uint8_t)0x98)
#define BME280_REG_DIG_P7     ((uint8_t)0x9A)
#define BME280_REG_DIG_P8     ((uint8_t)0x9C)
#define BME280_REG_DIG_P9     ((uint8_t)0x9E)

#define BME280_REG_HUM_LSB ((uint8_t)0xFE)
#define BME280_REG_HUM_MSB ((uint8_t)0xFD)
#define BME280_REG_DIG_H1  ((uint8_t)0xA1)
#define BME280_REG_DIG_H2  ((uint8_t)0xE1)
#define BME280_REG_DIG_H3  ((uint8_t)0xE3)
#define BME280_REG_DIG_H4  ((uint8_t)0xE4)
#define BME280_REG_DIG_H5  ((uint8_t)0xE5)
#define BME280_REG_DIG_H6  ((uint8_t)0xE7)

#define BME280_REG_CONFIG    ((uint8_t)0xF5)
#define BME280_REG_CTRL_MEAS ((uint8_t)0xF4)
#define BME280_REG_STATUS    ((uint8_t)0xF3)
#define BME280_REG_CTRL_HUM  ((uint8_t)0xF2)
#define BME280_REG_RESET     ((uint8_t)0xE0)
#define BME280_REG_ID        ((uint8_t)0xD0)

#define BME280_REG_RESET_MASK ((uint8_t)0xB6)

#define BME280_REG_STATUS_MEASURING_MASK ((uint8_t)0x08)
#define BME280_REG_STATUS_IM_UPDATE_MASK ((uint8_t)0x01)

#define BME280_SPI_IDLE_BYTE ((uint8_t)0x00)
#define BME280_SPI_READ_ADDRESS(ADDRESS) ((ADDRESS) | ((uint8_t)0x80))
#define BME280_SPI_WRITE_ADDRESS(ADDRESS) ((ADDRESS) & ~((uint8_t)0x80))

#define BME280_RESET_TIME_MS (25U)
#define BME280_MEASUREMENT_TIMEOUT_MS (5U)

#define BME280_SPI_DEVICE_INTERFACE_CONFIG(BME280_CONFIG_T) { \
    .clock_speed_hz = 5000000, \
    .mode = 0, \
    .spics_io_num = (BME280_CONFIG_T->spi.cs_gpio_num), \
    .clock_source = (BME280_CONFIG_T->spi.clock_source), \
    .duty_cycle_pos = 128, \
    .queue_size = 5, \
    .cs_ena_posttrans = 0, \
    .input_delay_ns = 10 \
}

#define BME280_SPI_READ_BYTES(BME280_HANDLE_T, SEND_BUFF, RECV_BUFF, BUFF_BYTES) ({ \
    spi_transaction_t spi_trans = { \
        .flags     = 0, \
        .addr      = 0, \
        .length    = (BUFF_BYTES) * 8, \
        .rxlength  = 0, \
        .tx_buffer = ((const void*) &(SEND_BUFF)), \
        .rx_buffer = ((void*) &(RECV_BUFF)) \
    }; \
    spi_device_transmit(BME280_HANDLE_T->spi.spi_handle, &spi_trans); \
})

#define BME280_SPI_WRITE_BYTES(BME280_HANDLE_T, SEND_BUFF, BUFF_BYTES) ({ \
    spi_transaction_t spi_trans = { \
        .flags = 0, \
        .addr  = 0, \
        .length    = (BUFF_BYTES) * 8, \
        .rxlength  = 0, \
        .tx_buffer = ((const void*) &SEND_BUFF), \
        .rx_buffer = NULL \
    }; \
    spi_device_transmit(BME280_HANDLE_T->spi.spi_handle, &spi_trans); \
})

#define BME280_I2C_IDLE_BYTE 0x00

#define BME280_I2C_READ_BYTES(BME280_HANDLE_T, SEND_BUFF, SEND_SIZE, RECV_BUFF, RECV_SIZE) ({ \
    i2c_master_write_read_device(BME280_HANDLE_T->i2c.port, \
                                 BME280_HANDLE_T->i2c.slave_address, \
                                 SEND_BUFF, \
                                 SEND_SIZE, \
                                 RECV_BUFF, \
                                 RECV_SIZE, \
                                 (BME280_HANDLE_T->i2c.rw_timeout_ms) / portTICK_PERIOD_MS); \
})

#define BME280_I2C_WRITE_BYTES(BME280_HANDLE_T, SEND_BUFF, SEND_SIZE) ({ \
    i2c_master_write_to_device(BME280_HANDLE_T->i2c.port, \
                               BME280_HANDLE_T->i2c.slave_address, \
                               SEND_BUFF, \
                               SEND_SIZE, \
                               (BME280_HANDLE_T->i2c.rw_timeout_ms) / portTICK_PERIOD_MS); \
})

#define BME280_LOG_TAG "BME280"
#define BME280_RETURN_ON_ERROR(ERROR) ESP_RETURN_ON_ERROR(ERROR, BME280_LOG_TAG, "")

static esp_err_t bme280_read_u8(bme280_handle_t handle, uint8_t address, uint8_t* value) {
    switch (handle->protocol) {
        case BME280_PROTOCOL_SPI: {
            const uint8_t send_buff[2] = {BME280_SPI_READ_ADDRESS(address), BME280_SPI_IDLE_BYTE};
            uint8_t recv_buff[2] = {0};
            BME280_RETURN_ON_ERROR(BME280_SPI_READ_BYTES(handle, send_buff, recv_buff, sizeof(recv_buff)));
            *value = recv_buff[1]; // return received byte (0 byte is dummy)
            return ESP_OK;
        }
        case BME280_PROTOCOL_I2C: {
            const uint8_t send_buff[1] = {address};
            uint8_t recv_buff[1] = {0};
            BME280_RETURN_ON_ERROR(BME280_I2C_READ_BYTES(handle, send_buff, sizeof(send_buff), recv_buff, sizeof(recv_buff)));
            *value = recv_buff[0];
            return ESP_OK;
        }
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t bme280_read_s8(bme280_handle_t handle, const uint8_t address, int8_t* value) {
    uint8_t temp = 0U;
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, address, &temp));
    *value = (int8_t)temp;
    return ESP_OK;
}

static esp_err_t bme280_write_u8(bme280_handle_t handle, const uint8_t address, const uint8_t value) {
     switch (handle->protocol) {
        case BME280_PROTOCOL_SPI: {
            const uint8_t req_buff[2] = {BME280_SPI_WRITE_ADDRESS(address), value};
            return BME280_SPI_WRITE_BYTES(handle, req_buff, sizeof(req_buff));
        }
        case BME280_PROTOCOL_I2C: {
            const uint8_t req_buff[2] = {address, value};
            return BME280_I2C_WRITE_BYTES(handle, req_buff, sizeof(req_buff));
        }
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t bme280_read_u16(bme280_handle_t handle, const uint8_t address, uint16_t* value) {
     switch (handle->protocol) {
        case BME280_PROTOCOL_SPI: {
            const uint8_t send_buff[3] = {BME280_SPI_READ_ADDRESS(address), BME280_SPI_IDLE_BYTE,
                                          BME280_SPI_IDLE_BYTE};
            uint8_t recv_buff[3] = {0};
            BME280_RETURN_ON_ERROR(BME280_SPI_READ_BYTES(handle, send_buff, recv_buff, sizeof(recv_buff)));
            *value = (((uint16_t)recv_buff[2]) << 8U) | ((uint16_t)recv_buff[1]);
            return ESP_OK;
        }
        case BME280_PROTOCOL_I2C: {
            uint8_t send_buff[2] = {address, BME280_I2C_IDLE_BYTE};
            uint8_t recv_buff[2] = {0};
            BME280_RETURN_ON_ERROR(BME280_I2C_READ_BYTES(handle, send_buff, sizeof(send_buff), recv_buff, sizeof(recv_buff)));
            *value = (((uint16_t)recv_buff[1]) << 8U) | ((uint16_t)recv_buff[0]);
            return ESP_OK;
        }
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t bme280_read_s16(bme280_handle_t handle, const uint8_t address, int16_t* value) {
    uint16_t temp = 0U;
    BME280_RETURN_ON_ERROR(bme280_read_u16(handle, address, &temp));
    *value = (int16_t)temp;
    return ESP_OK;
}

static esp_err_t bme280_write_config(bme280_handle_t handle, const bme280_config_t* const config) {
    /* write 'standby time ms', 'filter coefficient' */
    const uint8_t temp1 = ((config->standby_time_ms << 5) | (config->filter_coefficient << 2));
    BME280_RETURN_ON_ERROR(bme280_write_u8(handle, BME280_REG_CONFIG, temp1));

    /* write 'humidity oversampling' */
    const uint8_t temp2 = (config->humidity_oversampling);
    BME280_RETURN_ON_ERROR(bme280_write_u8(handle, BME280_REG_CTRL_HUM, temp2));

    /* write 'temperature oversampling', 'pressure oversampling', 'mode' */
    switch (config->mode) {
        case BME280_MODE_FORCED: {
            const uint8_t temp3 = ((config->temperature_oversampling << 5) | (config->pressure_oversampling << 2));
            BME280_RETURN_ON_ERROR(bme280_write_u8(handle, BME280_REG_CTRL_MEAS, temp3));
            break;
        }
        case BME280_MODE_NORMAL: {
            const uint8_t temp3 = ((config->temperature_oversampling << 5) | (config->pressure_oversampling << 2) | (config->mode));
            BME280_RETURN_ON_ERROR(bme280_write_u8(handle, BME280_REG_CTRL_MEAS, temp3));
            break;
        }
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}
 
static esp_err_t bme280_read_calibration_data(bme280_handle_t handle) {
    /* read temperature calibration data T1...T3*/
    BME280_RETURN_ON_ERROR(bme280_read_u16(handle, BME280_REG_DIG_T1, &(handle->dig_T1)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_T2, &(handle->dig_T2)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_T3, &(handle->dig_T3)));

    /* read pressure calibration data P1...P9*/
    BME280_RETURN_ON_ERROR(bme280_read_u16(handle, BME280_REG_DIG_P1, &(handle->dig_P1)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_P2, &(handle->dig_P2)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_P3, &(handle->dig_P3)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_P4, &(handle->dig_P4)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_P5, &(handle->dig_P5)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_P6, &(handle->dig_P6)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_P7, &(handle->dig_P7)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_P8, &(handle->dig_P8)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_P9, &(handle->dig_P9)));

    /* read humidity calibration data H1...H3 */
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_DIG_H1, &(handle->dig_H1)));
    BME280_RETURN_ON_ERROR(bme280_read_s16(handle, BME280_REG_DIG_H2, &(handle->dig_H2)));
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_DIG_H3, &(handle->dig_H3)));

    /* read humidity calibration data H4 */
    int8_t dig_H4_lsb = 0;
    int8_t dig_H4_msb = 0;
    BME280_RETURN_ON_ERROR(bme280_read_s8(handle, BME280_REG_DIG_H4, &dig_H4_msb));
    BME280_RETURN_ON_ERROR(bme280_read_s8(handle, (BME280_REG_DIG_H4 + 1), &dig_H4_lsb));
    handle->dig_H4 = (dig_H4_msb << 4) | (dig_H4_lsb & 0x0F);

    /* read humidity calibration data H5 */
    int8_t dig_H5_lsb = 0;
    int8_t dig_H5_msb = 0;
    BME280_RETURN_ON_ERROR(bme280_read_s8(handle, BME280_REG_DIG_H5, &dig_H5_lsb));
    BME280_RETURN_ON_ERROR(bme280_read_s8(handle, (BME280_REG_DIG_H5 + 1), &dig_H5_msb));
    handle->dig_H5 = (dig_H5_msb << 4) | (dig_H5_lsb >> 4);

    /* read humidity calibration data H6 */
    BME280_RETURN_ON_ERROR(bme280_read_s8(handle, BME280_REG_DIG_H6, &(handle->dig_H6)));

    return ESP_OK;
}

static esp_err_t bme280_read_temperature(bme280_handle_t handle, bme280_measurements_t* measurements) {
    uint8_t xlsb_T = 0;
    uint8_t lsb_T = 0;
    uint8_t msb_T = 0;
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_TEMP_XLSB, &xlsb_T));
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_TEMP_LSB, &lsb_T));
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_TEMP_MSB, &msb_T));
    const int32_t adc_T = (((int32_t)msb_T) << 12) | (((int32_t)lsb_T) << 4) | (((int32_t)xlsb_T) >> 4);

    double var1 = (((double)adc_T)/16384.0 - ((double)handle->dig_T1)/1024.0) * ((double)handle->dig_T2);
    double var2 = ((((double)adc_T)/131072.0 - ((double)handle->dig_T1)/8192.0) *
           (((double)adc_T)/131072.0 - ((double)handle->dig_T1)/8192.0)) * ((double)handle->dig_T3);
    handle->fine_T = (int32_t)(var1 + var2);
    measurements->temperature = (var1 + var2) / 5120.0;

    return ESP_OK;
}

static esp_err_t bme280_read_pressure(bme280_handle_t handle, bme280_measurements_t* measurements) {
    uint8_t xlsb_P = 0;
    uint8_t lsb_P = 0;
    uint8_t msb_P = 0;
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_PRESS_XLSB, &xlsb_P));
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_PRESS_LSB, &lsb_P));
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_PRESS_MSB, &msb_P));
    const int32_t adc_P = (((int32_t)msb_P) << 12) | (((int32_t)lsb_P) << 4) | (((int32_t)xlsb_P) >> 4);

    double final_P = 0.0;
    double var1 = ((double)handle->fine_T/2.0) - 64000.0;
    double var2 = var1 * var1 * ((double)handle->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)handle->dig_P5) * 2.0;
    var2 = (var2/4.0)+(((double)handle->dig_P4) * 65536.0);
    var1 = (((double)handle->dig_P3) * var1 * var1 / 524288.0 + ((double)handle->dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0)*((double)handle->dig_P1);
    if (var1 == 0.0) {
        return 0;
    }
    final_P = 1048576.0 - (double)adc_P;
    final_P = (final_P - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)handle->dig_P9) * final_P * final_P / 2147483648.0;
    var2 = final_P * ((double)handle->dig_P8) / 32768.0;
    final_P = final_P + (var1 + var2 + ((double)handle->dig_P7)) / 16.0;
    measurements->pressure = final_P; 

    return ESP_OK;
}

static esp_err_t bme280_read_humidity(bme280_handle_t handle, bme280_measurements_t* measurements) {
    uint8_t lsb_H = 0;
    uint8_t msb_H = 0;
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_HUM_LSB, &lsb_H));
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_HUM_MSB, &msb_H));
    const int32_t adc_H = (((int32_t)msb_H) << 8) | ((int32_t)lsb_H);

    double final_H;
    final_H = (((double)handle->fine_T) - 76800.0);
    final_H = (adc_H - (((double)handle->dig_H4) * 64.0 + ((double)handle->dig_H5) / 16384.0 *
              final_H)) * (((double)handle->dig_H2) / 65536.0 * (1.0 + ((double)handle->dig_H6) /
              67108864.0 * final_H * (1.0 + ((double)handle->dig_H3) / 67108864.0 * final_H)));
    final_H = final_H * (1.0 - ((double)handle->dig_H1) * final_H / 524288.0);
    if (final_H > 100.0) {
        final_H = 100.0;
    }
    else if (final_H < 0.0) {
        final_H = 0.0;
    }
    measurements->humidity = final_H;

    return ESP_OK;
}

static esp_err_t bme280_force_mesurement(bme280_handle_t handle) {
    uint8_t temp = 0;
    BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_CTRL_MEAS, &temp));
    temp &= ~BME280_MODE_NORMAL;
    temp |= BME280_MODE_FORCED;
    return bme280_write_u8(handle, BME280_REG_CTRL_MEAS, temp);
}

static esp_err_t bme280_wait_for_measurement_ready(bme280_handle_t handle) {
    for (uint8_t time_ms = 0U; time_ms < BME280_MEASUREMENT_TIMEOUT_MS; ++time_ms) {
        uint8_t temp = 0;
        BME280_RETURN_ON_ERROR(bme280_read_u8(handle, BME280_REG_STATUS, &temp));

        if (0 == (temp & BME280_REG_STATUS_IM_UPDATE_MASK)) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_FAIL;
}

static esp_err_t bme280_reset(bme280_handle_t handle) {
    BME280_RETURN_ON_ERROR(bme280_write_u8(handle, BME280_REG_RESET, BME280_REG_RESET_MASK));
    vTaskDelay(pdMS_TO_TICKS(BME280_RESET_TIME_MS));
    return ESP_OK;
}

bme280_handle_t bme280_create_device(const bme280_config_t* const config) {
    bme280_handle_t handle = malloc(sizeof(bme280_t));
    if (NULL == handle) {
        return NULL;
    }

    switch (config->protocol) {
        case BME280_PROTOCOL_SPI: {
            spi_device_interface_config_t spi_devcfg = BME280_SPI_DEVICE_INTERFACE_CONFIG(config);
            if (ESP_OK != spi_bus_add_device((config->spi.host_device), &spi_devcfg, &(handle->spi.spi_handle))) {
                free(handle);
                return NULL;
            }
            break;
        }
        case BME280_PROTOCOL_I2C: {
            handle->i2c.port = config->i2c.port;
            handle->i2c.slave_address = config->i2c.slave_address;
            handle->i2c.rw_timeout_ms = config->i2c.rw_timeout_ms;
            break;
        }
        default:
            free(handle);
            return NULL;
    }

    handle->mode = config->mode;
    handle->protocol = config->protocol;

    if (ESP_OK != bme280_reset(handle)) {
        bme280_destroy_device(&handle);
        return NULL;
    }

    if (ESP_OK != bme280_write_config(handle, config)) {
        bme280_destroy_device(&handle);
        return NULL;
    }

    if (ESP_OK != bme280_read_calibration_data(handle)) {
        bme280_destroy_device(&handle);
        return NULL;
    }

    return handle;
}

void bme280_destroy_device(bme280_handle_t* handle) {
    if (NULL == *handle) {
        return;
    }

    if (BME280_PROTOCOL_SPI == (*handle)->protocol) {
        spi_bus_remove_device((*handle)->spi.spi_handle);
    }

    free(*handle);
    *handle = NULL;
}

esp_err_t bme280_read_measurements(bme280_handle_t handle, bme280_measurements_t* measurements) {
    if (NULL == handle) {
        return ESP_FAIL;
    }

    if (BME280_MODE_FORCED == handle->mode) {
        BME280_RETURN_ON_ERROR(bme280_force_mesurement(handle));
    }

    BME280_RETURN_ON_ERROR(bme280_wait_for_measurement_ready(handle));

    BME280_RETURN_ON_ERROR(bme280_read_temperature(handle, measurements));
    BME280_RETURN_ON_ERROR(bme280_read_pressure(handle, measurements));
    BME280_RETURN_ON_ERROR(bme280_read_humidity(handle, measurements));

    return ESP_OK;
}