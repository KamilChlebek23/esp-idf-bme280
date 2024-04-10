# BME280 esp-idf library
This project provides esp-idf library for inferfacing BME280 sensor via SPI 
or I2C.

# Examples

## SPI
```
#include "bme280.h"
#include "assert.h"
#include "esp_check.h"

#define BME280_SPI_HOST          SPI2_HOST
#define BME280_SPI_MISO_GPIO_NUM 23
#define BME280_SPI_MOSI_GPIO_NUM 22
#define BME280_SPI_SCLK_GPIO_NUM 5
#define BME280_SPI_CS_GPIO_NUM   18
#define BME280_SPI_FREQ_HZ       1000000
#define BME280_SPI_MODE          0
#define BME280_SPI_CLK_SRC       SPI_CLK_SRC_DEFAULT

void app_main(void) {
    /* configure SPI */
    spi_bus_config_t spi_buscfg = {
        .miso_io_num = BME280_SPI_MISO_GPIO_NUM,
        .mosi_io_num = BME280_SPI_MOSI_GPIO_NUM,
        .sclk_io_num = BME280_SPI_SCLK_GPIO_NUM,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    assert(ESP_OK == spi_bus_initialize(BME280_SPI_HOST, &spi_buscfg, SPI_DMA_CH1));

    /* create BME280 device with given configuration */
    bme280_config_t config = {
        .standby_time_ms = BME280_STANDBY_TIME_MS_0_5,
        .filter_coefficient = BME280_FILTER_COEFFICIENT_16,
        .temperature_oversampling = BME280_OVERSAMPLING_16,
        .pressure_oversampling = BME280_OVERSAMPLING_16,
        .humidity_oversampling = BME280_OVERSAMPLING_16,
        .mode = BME280_MODE_NORMAL,
        .protocol = BME280_PROTOCOL_SPI,
        .spi = {
            .host_device = BME280_SPI_HOST,
            .clock_source = BME280_SPI_CLK_SRC,
            .cs_gpio_num = BME280_SPI_CS_GPIO_NUM
        }
    };
    bme280_handle_t bme280 = bme280_create_device(&config);
    assert(NULL != bme280);
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* read and display measurements */
    bme280_measurements_t measurements;
    assert(ESP_OK == bme280_read_measurements(bme280, &measurements));
    printf("Temperature: %0.2f [°C]\n", measurements.temperature);
    printf("Pressure: %0.2f [hPa]\n", measurements.pressure / 100.0);
    printf("Humidity: %0.2f [%%]\n", measurements.humidity);

    /* destroy BME280 device */
    bme280_destroy_device(&bme280);
    assert(NULL == bme280);
}
```

## I2C
```
#include "bme280.h"
#include "assert.h"
#include "esp_check.h"

#define BME280_I2C_PORT 0
#define BME280_I2C_SDA 16
#define BME280_I2C_SCL 17
#define BME280_I2C_CLK_SPEED_HZ 400000
#define BME280_I2C_ADDRESS 0x77

void app_main(void) {
    /* configure I2C */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BME280_I2C_SDA,
        .scl_io_num = BME280_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BME280_I2C_CLK_SPEED_HZ
    };
    assert(ESP_OK == i2c_param_config(BME280_I2C_PORT, &conf));
    assert(ESP_OK == i2c_driver_install(BME280_I2C_PORT, conf.mode, 0, 0, 0));

    /* create BME280 device with given configuration */
    bme280_config_t config = {
        .standby_time_ms = BME280_STANDBY_TIME_MS_0_5,
        .filter_coefficient = BME280_FILTER_COEFFICIENT_16,
        .temperature_oversampling = BME280_OVERSAMPLING_16,
        .pressure_oversampling = BME280_OVERSAMPLING_16,
        .humidity_oversampling = BME280_OVERSAMPLING_16,
        .mode = BME280_MODE_NORMAL,
        .protocol = BME280_PROTOCOL_I2C,
        .i2c = {
            .port = BME280_I2C_PORT,
            .slave_address = BME280_I2C_ADDRESS,
            .rw_timeout_ms = 100
        }
    };
    bme280_handle_t bme280 = bme280_create_device(&config);
    assert(NULL != bme280);
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* read and display measurements */
    bme280_measurements_t measurements;
    bme280_read_measurements(bme280, &measurements);
    printf("Temperature: %0.2f [°C]\n", measurements.temperature);
    printf("Pressure: %0.2f [hPa]\n", measurements.pressure / 100.0);
    printf("Humidity: %0.2f [%%]\n", measurements.humidity);

    /* destroy BME280 device */
    bme280_destroy_device(&bme280);
    assert(NULL == bme280);
}
```