#include "IMU.h"

// Static member definitions and initializations
imu_data_t IMU::shared_imu_data = {};
converted_sensor_data IMU::calibrated_gyro_offset = {};


QueueHandle_t IMU::batch_data_queue = xQueueCreate(BATCH_SIZE, sizeof(converted_imu_data));
QueueHandle_t IMU::sensor_data_queue = xQueueCreate(10, sizeof(SensorQueueEntity));
EventGroupHandle_t IMU::print_ready_event_group = xEventGroupCreate();
SemaphoreHandle_t IMU::shared_imu_data_mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t IMU::accelGyroDRDYSemaphore = xSemaphoreCreateBinary();
SemaphoreHandle_t IMU::magDRDYSemaphore = xSemaphoreCreateBinary();
SemaphoreHandle_t IMU::batchReadySemaphore = xSemaphoreCreateBinary();
bool IMU::multi_read_enabled = false;

const uint8_t accel_setup_commands[8] = {
    CTRL_REG5_XL, 0b00111000, // enable axis
    CTRL_REG6_XL, 0b01000000, // 50HZ ODR, +-2g, 
    CTRL_REG7_XL, 0b11000000, // high res mode and disable filters.
    CTRL_REG9, 0b00001000 // enable active high data ready signal for both accel and gyro

    // CTRL_REG7_XL, 0b01000000 // ODR/9 BW, High-res mode, use LPF, bypass HPF
};

const uint8_t gyro_setup_commands[6] = {
    CTRL_REG1_G, 0b01000000, // 59.5hz ODR, slow rotation, 
    CTRL_REG4_G, 0b00111000,  // enable axis
    CTRL_REG1_G, 0b01100000 // fast read for calibration (119hz)
};

const uint8_t mag_setup_commands[4] = {
    CTRL_REG1_M, 0b10011000, // 40HZ rate, temp comp, no high performance, 
    CTRL_REG3_M, 0b10000000 // continuous-conversion mode, i2c disable
};

const uint8_t interrupt_setup_commands[2] = {
    INT1_CTRL, 0b00000011, // enable data ready interrupts for accel, gyro, mag on INT1
    // drdy is configured by default
};








void IMU::package_data() {
    SensorQueueEntity queue_entity;
    sensor_data raw_sensor_data;
    converted_sensor_data converted_sensor_data;

    for(;;) {
        if (xQueueReceive(sensor_data_queue, &queue_entity, portMAX_DELAY)) {
            // printf("Received data from queue for device type %d\n", queue_entity.device_type);
            memset(&raw_sensor_data, 0, sizeof(sensor_data));
            memset(&converted_sensor_data, 0, sizeof(sensor_data));
            raw_sensor_data.x = (int16_t)((queue_entity.data[1] << 8) | queue_entity.data[0]);
            raw_sensor_data.y = (int16_t)((queue_entity.data[3] << 8) | queue_entity.data[2]);
            raw_sensor_data.z = (int16_t)((queue_entity.data[5] << 8) | queue_entity.data[4]);

            switch (queue_entity.device_type) {
                case ACCELEROMETER:
                    // printf("Accel Raw: X=%d, Y=%d, Z=%d\r\n", raw_sensor_data.x, raw_sensor_data.y, raw_sensor_data.z);
                    converted_sensor_data.x = RAW_TO_ACCEL_G(raw_sensor_data.x);
                    converted_sensor_data.y = RAW_TO_ACCEL_G(raw_sensor_data.y);
                    converted_sensor_data.z = RAW_TO_ACCEL_G(raw_sensor_data.z);
                    break;
                case GYROSCOPE:
                    // printf("Gyro Raw: X=%d, Y=%d, Z=%d\r\n", raw_sensor_data.x, raw_sensor_data.y, raw_sensor_data.z);
                    converted_sensor_data.x = RAW_TO_GYRO_DPS(raw_sensor_data.x) - IMU::calibrated_gyro_offset.x;
                    converted_sensor_data.y = RAW_TO_GYRO_DPS(raw_sensor_data.y) - IMU::calibrated_gyro_offset.y;
                    converted_sensor_data.z = RAW_TO_GYRO_DPS(raw_sensor_data.z) - IMU::calibrated_gyro_offset.z;
                    break;
                case MAGNETOMETER:  
                    // printf("Mag Raw: X=%d, Y=%d, Z=%d\r\n", raw_sensor_data.x, raw_sensor_data.y, raw_sensor_data.z);     
                    converted_sensor_data.x = RAW_TO_MAG_GAUSS(raw_sensor_data.x);
                    converted_sensor_data.y = RAW_TO_MAG_GAUSS(raw_sensor_data.y);
                    converted_sensor_data.z = RAW_TO_MAG_GAUSS(raw_sensor_data.z);
                    break;  
            }


            xSemaphoreTake(shared_imu_data_mutex, portMAX_DELAY);
            memcpy(((uint8_t *)&(IMU::shared_imu_data.raw_imu_readings)) + queue_entity.device_type * sizeof(sensor_data), &raw_sensor_data, sizeof(sensor_data));
            memcpy(((uint8_t *)&(IMU::shared_imu_data.converted_imu_readings)) + queue_entity.device_type * sizeof(converted_sensor_data), &converted_sensor_data, sizeof(converted_sensor_data));
            xSemaphoreGive(shared_imu_data_mutex);
            xEventGroupSetBits(print_ready_event_group, (1 << queue_entity.device_type));
        }
        
    }    
}



void IMU::spi_transmit_single_command(spi_transmit_single_command_task_s *task_data) {

    uint8_t tx[4];
    spi_device_handle_t device;

    switch (task_data->device_type) {
        case MAGNETOMETER:
            device = magnetometer_spi_device_handle; 
            tx[0] = (task_data->read_active_high << 7) | (task_data->multi_read_active_high << 6) | (task_data->command[0] & 0x3F);
            break;
        case ACCELEROMETER:
        case GYROSCOPE:
            device = accelerometer_gyroscope_spi_device_handle;
            tx[0] = (task_data->read_active_high << 7) | (task_data->command[0] & 0x7F);
            break;
        default:
            device = NULL;
            break;
    }

    for (int i = 1; i < task_data->command_length; i++) {
        tx[i] = task_data->command[i]; 
    }
    for (int i = 0; i < task_data->dummy_length; i++) {
        tx[task_data->command_length + i] = 0x00; 
    }

    // for (int i = 0; i < task_data->command_length + task_data->dummy_length; i++) {
        // printf("TX Byte %d: %d\n", i, tx[i]);
    // }


    spi_transmit(tx, task_data->response, device, task_data->command_length + task_data->dummy_length);
}

void IMU::init() {    
    printf("Calibrated gyro offsets: X=%f, Y=%f, Z=%f\n", IMU::calibrated_gyro_offset.x, IMU::calibrated_gyro_offset.y, IMU::calibrated_gyro_offset.z);
    uint8_t rx_dummy[4];

    // setup gyro
    spi_transmit_single_command_task_s init_task = {
        .command = (uint8_t *)&gyro_setup_commands[4],
        .command_length = 2,
        .dummy_length = 0,
        .device_type = GYROSCOPE,
        .response = rx_dummy,
        .read_active_high = 0,
        .multi_read_active_high = 1 // doesn't matter for gyro
    };

    IMU::spi_transmit_single_command(&init_task); // put gyro in fast mode for calibration

    init_task.command = (uint8_t *)&gyro_setup_commands[2];
    IMU::spi_transmit_single_command(&init_task); // enable axes

    init_task.command = (uint8_t *)gyro_setup_commands;
    IMU::spi_transmit_single_command(&init_task); // normal mode, 59.5hz


    // setup accel
    init_task.command = (uint8_t *)accel_setup_commands;
    init_task.device_type = ACCELEROMETER;

    IMU::spi_transmit_single_command(&init_task); // enable axes

    init_task.command = (uint8_t *)&accel_setup_commands[2];
    IMU::spi_transmit_single_command(&init_task); // set reading frequency 

    init_task.command = (uint8_t *)&accel_setup_commands[4];
    IMU::spi_transmit_single_command(&init_task); // set filters

    init_task.command = (uint8_t *)&accel_setup_commands[6];
    IMU::spi_transmit_single_command(&init_task); // set data ready interrupt

    // setup mag
    init_task.command = (uint8_t *)mag_setup_commands;
    init_task.device_type = MAGNETOMETER;
    IMU::spi_transmit_single_command(&init_task); // mag setup part 1
    
    init_task.command = (uint8_t *)&mag_setup_commands[2];
    IMU::spi_transmit_single_command(&init_task); // mag setup part 2

    // general setup
    IMU::multi_read_enabled = false;
    IMU::toggle_multi_read();
    
    
    IMU::configure_interrupts();
    init_task.command = (uint8_t *)interrupt_setup_commands;
    init_task.device_type = GYROSCOPE; // doesn't matter if gyro or accel
    IMU::spi_transmit_single_command(&init_task); // enable data ready interrupts

    //! Do this only after enabling interrupts
    IMU::calibrate_gyroscope();
    printf("IMU initialization complete.\n");

    // this is done to reset status bits and clear any pending interrupts
    IMU::read_accelerometer();
    IMU::read_gyroscope();
    IMU::read_magnetometer();
}

void IMU::read_gyroscope_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(accelGyroDRDYSemaphore, portMAX_DELAY) == pdTRUE) {
            // printf("Accel/Gyro interrupt triggered\n");
            IMU::read_gyroscope();
            IMU::read_accelerometer();
        }
    }
}
// TODO: THIS IS A TEMPORARY FIX AS I ONLY HAVE ONE WIRE FOR TWO INTERRUPTS (OUT OF JUMPERS)
// void IMU::read_accelerometer_task(void *pvParameters) {
//     while (1) {
//         if (xSemaphoreTake(accelGyroDRDYSemaphore, portMAX_DELAY) == pdTRUE) {
//             IMU::read_accelerometer();
//         }
//     }
// }

void IMU::read_magnetometer_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(magDRDYSemaphore, portMAX_DELAY) == pdTRUE) {
            // printf("Magnetometer interrupt triggered\n");
            IMU::read_magnetometer();
        }
    }
}

void IMU::read_magnetometer() {
    uint8_t mag_data[7];

    spi_transmit_single_command_task_s get_mag_data = {
        .command = (uint8_t[]){OUT_X_L_M},
        .command_length = 1,
        .dummy_length = 6,
        .device_type = MAGNETOMETER,
        .response = mag_data,
        .read_active_high = 1,
        .multi_read_active_high = 1
    };

    IMU::spi_transmit_single_command(&get_mag_data);

    SensorQueueEntity queue_entity = {
        .device_type = MAGNETOMETER,
        .data = {0}
    };
    memcpy(queue_entity.data, mag_data + 1, 6);
    // printf("Sending mag");
    xQueueSend(sensor_data_queue, &queue_entity, 0);
}

void IMU::read_gyroscope() {
    if (!IMU::multi_read_enabled) {
        IMU::toggle_multi_read();
    }
    uint8_t gyro_data[7];

    spi_transmit_single_command_task_s get_gyro_data = {
        .command = (uint8_t[]){OUT_X_L_G},
        .command_length = 1,
        .dummy_length = 6,
        .device_type = GYROSCOPE,
        .response = gyro_data,
        .read_active_high = 1,
        .multi_read_active_high = 1 // doesn't matter for gyro
    };
    IMU::spi_transmit_single_command(&get_gyro_data);

    SensorQueueEntity queue_entity = {
        .device_type = GYROSCOPE,
        .data = {0}
    };
    memcpy(queue_entity.data, gyro_data + 1, 6);

    // printf("Sending gyro");
    xQueueSend(sensor_data_queue, &queue_entity, 0);
}

void IMU::read_accelerometer() {
    if (!IMU::multi_read_enabled) {
        IMU::toggle_multi_read();
    }

    uint8_t accel_data[7];
    spi_transmit_single_command_task_s get_accel_data = {
        .command = (uint8_t[]){OUT_X_L_XL},
        .command_length = 1,
        .dummy_length = 6,
        .device_type = ACCELEROMETER,
        .response = accel_data,
        .read_active_high = 1,
        .multi_read_active_high = 1 // doesn't matter for accel
    };
    IMU::spi_transmit_single_command(&get_accel_data);
    SensorQueueEntity queue_entity = {
        .device_type = ACCELEROMETER,
        .data = {0}
    };
    memcpy(queue_entity.data, accel_data + 1, 6);
    // printf("Sending accel");
    xQueueSend(sensor_data_queue, &queue_entity, 0);
}

void IMU::toggle_multi_read() {
    uint8_t tx[] = { CTRL_REG8, 0x00 }; // enable multiread
    if (!IMU::multi_read_enabled) {
        tx[1] =0b00000100;
    } 

    uint8_t rx_dummy[2];

    spi_transmit_single_command_task_s multi_read = {
        .command = tx,
        .command_length = 2,
        .dummy_length = 0,
        .device_type = GYROSCOPE,
        .response = rx_dummy,
        .read_active_high = 0,
        .multi_read_active_high = 1 // doesn't matter for gyro
    };
    IMU::spi_transmit_single_command(&multi_read);
    IMU::multi_read_enabled = !IMU::multi_read_enabled;
    if (IMU::multi_read_enabled) {
        printf("Multi-read enabled\n");
    } else {
        printf("Multi-read disabled\n");
    }
}

void IMU::print_imu() {
    
    while (1) {
        xEventGroupWaitBits(print_ready_event_group,
            ACCEL_BIT | GYRO_BIT | MAG_BIT,
            pdTRUE, pdTRUE, portMAX_DELAY);
        xSemaphoreTake(shared_imu_data_mutex, portMAX_DELAY);
        printf("Accel: X=%f Y=%f Z=%f | Gyro: X=%f Y=%f Z=%f | Mag: X=%f Y=%f Z=%f\n",
            IMU::shared_imu_data.converted_imu_readings.accel.x, IMU::shared_imu_data.converted_imu_readings.accel.y, IMU::shared_imu_data.converted_imu_readings.accel.z,
            IMU::shared_imu_data.converted_imu_readings.gyro.x, IMU::shared_imu_data.converted_imu_readings.gyro.y, IMU::shared_imu_data.converted_imu_readings.gyro.z,
            IMU::shared_imu_data.converted_imu_readings.mag.x, IMU::shared_imu_data.converted_imu_readings.mag.y, IMU::shared_imu_data.converted_imu_readings.mag.z);
        // printf("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d | Mag: X=%d Y=%d Z=%d\n",
        //     IMU::shared_imu_data.raw_imu_readings.accel.x, IMU::shared_imu_data.raw_imu_readings.accel.y, IMU::shared_imu_data.raw_imu_readings.accel.z,
        //     IMU::shared_imu_data.raw_imu_readings.gyro.x, IMU::shared_imu_data.raw_imu_readings.gyro.y, IMU::shared_imu_data.raw_imu_readings.gyro.z,
        //     IMU::shared_imu_data.raw_imu_readings.mag.x, IMU::shared_imu_data.raw_imu_readings.mag.y, IMU::shared_imu_data.raw_imu_readings.mag.z); 
        
        //TODO: batch data logging
            // xQueueSend(IMU::batch_data_queue, &IMU::shared_imu_data.converted_imu_readings, 0);
        xSemaphoreGive(shared_imu_data_mutex);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void IMU::calibrate_gyroscope() {
    constexpr int num_samples = 100;
    float gyro_x_offset = 0.0f;
    float gyro_y_offset = 0.0f;
    float gyro_z_offset = 0.0f;

    for (int i = 0; i < num_samples; i++) {
        IMU::read_gyroscope(); // bypass interrupts for calibration
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to allow data to be read
        xSemaphoreTake(shared_imu_data_mutex, portMAX_DELAY);
        gyro_x_offset += IMU::shared_imu_data.converted_imu_readings.gyro.x;
        gyro_y_offset += IMU::shared_imu_data.converted_imu_readings.gyro.y;
        gyro_z_offset += IMU::shared_imu_data.converted_imu_readings.gyro.z;
        xSemaphoreGive(shared_imu_data_mutex);
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between samples
    }

    gyro_x_offset /= num_samples;
    gyro_y_offset /= num_samples;
    gyro_z_offset /= num_samples;

    IMU::calibrated_gyro_offset.x = gyro_x_offset;
    IMU::calibrated_gyro_offset.y = gyro_y_offset;
    IMU::calibrated_gyro_offset.z = gyro_z_offset;

    // Store offsets for later use (not implemented in this snippet)
    printf("Gyroscope calibrated. Offsets - X: %f, Y: %f, Z: %f\n", gyro_x_offset, gyro_y_offset, gyro_z_offset);
}

void IMU::configure_interrupts() {
    gpio_install_isr_service(0);

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1 << ACCEL_GYRO_INTERRUPT_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1 << MAG_INTERRUPT_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf);

    gpio_isr_handler_add(ACCEL_GYRO_INTERRUPT_PIN, [](void*){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(IMU::accelGyroDRDYSemaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }, NULL);

    gpio_isr_handler_add(MAG_INTERRUPT_PIN, [](void*){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(IMU::magDRDYSemaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }, NULL);
}

void IMU::package_data_task(void *pvParameters) {
    IMU::instance().package_data();
}

void IMU::print_imu_task(void *pvParameters) {
    IMU::instance().print_imu();
}








void IMU::spi_transmit_single_command_task(void *pvParameters) {
    uint8_t rx[4];
    spi_transmit_single_command_task_s *task_data = (spi_transmit_single_command_task_s *)pvParameters;
    task_data->response = rx;


    while (1) {
        IMU::spi_transmit_single_command(task_data);
        vTaskDelay(pdMS_TO_TICKS(80)); // Delay for 80 milliseconds
        printf("IMU SPI transfer task running...\n");
        for (int i = 0; i < task_data->command_length + task_data->dummy_length; i++) {
            printf("Byte %d: %d\n", i, task_data->response[i]);
        }
    }
}