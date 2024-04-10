#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>
#include <driver/spi_master.h>
#include "driver/i2c.h"

typedef enum {
    BME280_STANDBY_TIME_MS_0_5 = 0,
    BME280_STANDBY_TIME_MS_62_5,
    BME280_STANDBY_TIME_MS_125,
    BME280_STANDBY_TIME_MS_250,
    BME280_STANDBY_TIME_MS_500,
    BME280_STANDBY_TIME_MS_1000,
    BME280_STANDBY_TIME_MS_10,
    BME280_STANDBY_TIME_MS_20
} bme280_standby_time_ms_t;

typedef enum {
    BME280_FILTER_COEFFICIENT_0 = 0,
    BME280_FILTER_COEFFICIENT_2,
    BME280_FILTER_COEFFICIENT_4,
    BME280_FILTER_COEFFICIENT_8,
    BME280_FILTER_COEFFICIENT_16
} bme280_filter_coefficient_t;

typedef enum {
    BME280_OVERSAMPLING_0 = 0,
    BME280_OVERSAMPLING_1,
    BME280_OVERSAMPLING_2,
    BME280_OVERSAMPLING_4,
    BME280_OVERSAMPLING_8,
    BME280_OVERSAMPLING_16
} bme280_oversampling_t;

typedef enum {
    BME280_MODE_FORCED = 1,
    BME280_MODE_NORMAL = 3,
} bme280_mode_t;

typedef enum {
    BME280_PROTOCOL_SPI = 0,
    BME280_PROTOCOL_I2C
} bme280_protocol_t;

typedef struct {
    bme280_standby_time_ms_t    standby_time_ms;
    bme280_filter_coefficient_t filter_coefficient;
    bme280_oversampling_t       temperature_oversampling;
    bme280_oversampling_t       pressure_oversampling;
    bme280_oversampling_t       humidity_oversampling;
    bme280_mode_t               mode;
    bme280_protocol_t           protocol;
    union {
        struct {
            spi_host_device_t  host_device;
            spi_clock_source_t clock_source;
            uint8_t            cs_gpio_num;
        } spi;
        struct {
            i2c_port_t port;
            uint8_t    slave_address;
            uint64_t   rw_timeout_ms;
        } i2c;
    };
} bme280_config_t;

typedef struct {
    uint16_t            dig_T1;
    int16_t             dig_T2;
    int16_t             dig_T3;
    uint16_t            dig_P1;
    int16_t             dig_P2;
    int16_t             dig_P3;
    int16_t             dig_P4;
    int16_t             dig_P5;
    int16_t             dig_P6;
    int16_t             dig_P7;
    int16_t             dig_P8;
    int16_t             dig_P9;
    uint8_t             dig_H1;
    int16_t             dig_H2;
    uint8_t             dig_H3;
    int16_t             dig_H4;
    int16_t             dig_H5;
    int8_t              dig_H6;
    int32_t             fine_T;
    bme280_mode_t       mode;
    bme280_protocol_t   protocol;
    union {
        struct {
            spi_device_handle_t spi_handle;
        } spi;
        struct {
            i2c_port_t port;
            uint8_t    slave_address;
            uint64_t   rw_timeout_ms;
        } i2c;
    };
} bme280_t;

typedef bme280_t* bme280_handle_t;

typedef struct  {
    double temperature;
    double pressure;
    double humidity;
} bme280_measurements_t;

/**
  * @brief Allocate resources for BME280 device and initialize it
  *
  * @note Allocate ESP32 resources that are required by BME280 device and initialize
  * it with configuration passed to this function.
  *
  * @param [in] config Configuration for BME280 device
  * @return Handle to BME280 device if creation was successful, NULL otherwise
  */
bme280_handle_t bme280_create_device(const bme280_config_t* const config);

/**
  * @brief Deallocate resources for BME280 device
  *
  * @note Deallocate ESP32 resources that are occupied by BME280 device.
  *
  * @param [in] bme280 BME280 device handle pointer
  */
void bme280_destroy_device(bme280_handle_t* bme280);

/**
  * @brief Read measurements from BME280 sensor
  *
  * @note Read temperature, pressure, humidity from BME280 sensor.
  * BME280 device must be firstly created using 'bme280_create_device' function.
  *
  * @param [in] bme280 BME280 device handle
  * @param [out] measurements Measurement results
  * @return
  *         - ESP_OK  - read measurement ok
  *         - other ESP error - read measurement fail
  */
esp_err_t bme280_read_measurements(bme280_handle_t bme280, bme280_measurements_t* measurements);

#endif