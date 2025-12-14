#include "ToF.h"
//! DEPRECATED - for reference only
void tof_init(){
    // Initialize GPIO pins for ToF sensor I2C communication
    esp_rom_gpio_pad_select_gpio(I2C_SDA);
    esp_rom_gpio_pad_select_gpio(I2C_SCL);
    gpio_set_direction(I2C_SDA, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(I2C_SCL, GPIO_MODE_INPUT_OUTPUT_OD);
    
    gpio_set_pull_mode(I2C_SDA, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(I2C_SCL, GPIO_PULLUP_ONLY);

    // Then configure the I2C driver (example for master mode)
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void tof_read() {
    uint8_t data[2];

    esp_err_t ret = i2c_master_read_from_device(
        I2C_MASTER_NUM,
        I2C_MASTER_ADDR,
        data,
        sizeof(data),
        1000 / portTICK_PERIOD_MS
    );
    if (ret == ESP_OK) {
        printf("ToF Data: %02x %02x\n", data[0], data[1]);
        // Process the read data
    } else {
        printf("Error reading ToF data: %d\n", ret);

        // Handle error
    }
}
