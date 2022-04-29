#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"


//I2C SENSOR STUFF//
#define I2C_MASTER_SCL_IO           33      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           34     /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define SENSOR_ADDR                 0x92        /*!< Slave address of the temp sensor  1001 001*/


uint8_t myPow (uint8_t base, uint8_t exponent) {

    uint8_t result = 1;

    for (uint8_t i = 0; i < exponent; i++) {
        result *= base;
    }

    return result;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Read a sequence of bytes from a temp sensor registers
 */
static esp_err_t temp_register_read(uint8_t *data, size_t len)
{   
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_ADDR + 1, ACK_CHECK_EN); //0x93 is the slave address hardcoded (the 1 in the LSB tells that it's a read)
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

float convertTemp (uint8_t data[]) {

    float tempData = 0.0;
    uint8_t temp;

    if (data[0] < 128 ) { //MSB is 0 (positive temp)

        temp = data[0]; 
        for (uint8_t i = 0; i < 8; i++) {
            tempData += (temp % 2) * myPow(2,i);
            temp /= 2;
        }

        temp = data[1];
        temp /= 32; //Turns byte into most significant 3 bits
        if ((temp % 2) == 1) { //0.125
            tempData += 0.125;
        }
        temp /= 2;
        if ((temp % 2) == 1) { //0.25
            tempData += 0.25;
        }
        temp /= 2;
        if ((temp % 2) == 1) { //0.5
            tempData += 0.5;
        }

    } else { //LSB is 1 (negative temp)
        int16_t bits = 0xff00 | data[0]; //Sign Extended the temp in binary
        bits = bits << 3; //Shift over by 4 bits to allow for decimal of Temp
        temp = data[1];
        temp /= 32; //Turns byte into most significant 3 bits
        if ((temp % 2) == 1) { //0.125
            bits |= 1 << 0;
        }
        temp /= 2;
        if ((temp % 2) == 1) { //0.25
            bits |= 1 << 1;
        }
        temp /= 2;
        if ((temp % 2) == 1) { //0.5
            bits |= 1 << 2;
        }
        tempData = bits;
        tempData /= 8; 
    }
    return tempData;
}

/*
 *  The IC defaults to 9-bit resoultion, MSB is the sign and the leftover 8 are temp data
 *  With 9-bit resolution, the coversion is 0.5˚C. To get the actual temp, read the value from
 *  the binary representation and multiply by 0.5 (for MSB = 0). If MSB = 1, then twos complement
 *  will be used to convert to the value read 
*/

void app_main() {
    i2c_master_init();

    uint8_t IC_data[2];
    float IC_Temp;

    //Continuously samples
    while (1) {
        ESP_ERROR_CHECK(temp_register_read(IC_data, 2)); 
        printf("IC Temp = %d + %d\n", IC_data[0], IC_data[1]);
        IC_Temp = convertTemp(IC_data); //FIX NEGATIVE
        printf("IC Temp = %f ˚C\n",IC_Temp);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}