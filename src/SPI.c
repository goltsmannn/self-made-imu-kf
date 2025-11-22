#include "SPI.h"

spi_device_handle_t magnetometer_spi_device_handle;
spi_device_handle_t accelerometer_gyroscope_spi_device_handle;

const spi_bus_config_t magnetometer_spi_config = {
    .miso_io_num =  HSPI_MISO_M_PIN,
    .mosi_io_num = HSPI_MOSI_PIN,
    .sclk_io_num = HSPI_SCLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

const spi_bus_config_t accelerometer_gyroscope_spi_config = {
    .miso_io_num =  HSPI_MISO_AG_PIN,
    .mosi_io_num = HSPI_MOSI_PIN,
    .sclk_io_num = HSPI_SCLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

const spi_device_interface_config_t magnetometer_spi_device_config = {
    .mode = 3,
    .clock_speed_hz = 10*1000*1000, // 10MHz
    .input_delay_ns = 50,
    .spics_io_num = HSPI_CS_M_PIN
};

const spi_device_interface_config_t accelerometer_gyroscope_spi_device_config = {
    .mode = 3,
    .clock_speed_hz = 10*1000*1000, // 10MHz
    .input_delay_ns = 50,
    .spics_io_num = HSPI_CS_AG_PIN
};

void imu_spi_init() {
    spi_bus_initialize(HSPI_HOST, &magnetometer_spi_config, SPI_DMA_CH_AUTO);
    spi_bus_add_device(HSPI_HOST, &magnetometer_spi_device_config, &magnetometer_spi_device_handle);

    spi_bus_initialize(HSPI_HOST, &accelerometer_gyroscope_spi_config, SPI_DMA_CH_AUTO);
    spi_bus_add_device(HSPI_HOST, &accelerometer_gyroscope_spi_device_config, &accelerometer_gyroscope_spi_device_handle);
}

uint8_t spi_read_magnetometer(uint8_t address) {
    spi_transaction_t t;
    uint8_t tx[2] = {address, 0x00};
    uint8_t rx[2];
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    esp_err_t ret = spi_device_transmit(magnetometer_spi_device_handle, &t);
    if (ret != ESP_OK) {
        // Handle error
        printf("Error reading magnetometer: %d\r\n", ret);
    }
    return rx[1];
}

void spi_read_accelerometer_gyroscope(uint8_t address) {
    return;
}