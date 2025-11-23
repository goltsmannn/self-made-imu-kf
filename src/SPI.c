#include "SPI.h"

spi_device_handle_t magnetometer_spi_device_handle;
spi_device_handle_t accelerometer_gyroscope_spi_device_handle;


spi_bus_config_t spi_config = {
    .miso_io_num =  HSPI_MISO_PIN,
    .mosi_io_num = HSPI_MOSI_PIN,
    .sclk_io_num = HSPI_SCLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

spi_device_interface_config_t magnetometer_spi_device_config = {
    .mode = 3,
    .clock_speed_hz = 10*1000*1000, // 10MHz
    .input_delay_ns = 50,
    .spics_io_num = HSPI_CS_M_PIN,
    .queue_size = 4,
    .flags = 0,
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0 
};

spi_device_interface_config_t accelerometer_gyroscope_spi_device_config = {
    .mode = 3,
    .clock_speed_hz = 10*1000*1000, // 10MHz
    .input_delay_ns = 50,
    .spics_io_num = HSPI_CS_AG_PIN,
    .queue_size = 4,
    .flags = 0,
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0 
};

void spi_init() {
    spi_bus_initialize(HSPI_HOST, &spi_config, SPI_DMA_CH_AUTO);
    spi_bus_add_device(HSPI_HOST, &magnetometer_spi_device_config, &magnetometer_spi_device_handle);
    spi_bus_add_device(HSPI_HOST, &accelerometer_gyroscope_spi_device_config, &accelerometer_gyroscope_spi_device_handle);
}

void spi_transmit(uint8_t *tx, uint8_t *rx, spi_device_handle_t device, size_t length) {
    spi_transaction_t t = {0};
    t.length = length * 8;

    memcpy(t.tx_data, tx, length); 
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    
    esp_err_t ret = spi_device_transmit(device, &t);
    if (ret != ESP_OK) {
        // Handle error
        printf("Error readin: %d\r\n", ret);
    }
    // for (int i = 0; i < length; i++) {
    //     printf("RX Byte %d: %d\n", i, t.rx_data[i]);
    // }
    memcpy(rx, t.rx_data, 4); // Copy received data to caller's buffer
}

