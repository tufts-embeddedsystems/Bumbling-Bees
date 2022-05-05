// Bumbling Bees temperature sensor code
// Embedded Systems, spring 2022

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "minimal_wifi.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"

// Address of MQTT server
#define BROKER_URI "mqtt://en1-pi.eecs.tufts.edu"
#define NODE_PATH "nodes/bumbling-bees/node1"

// How long to sleep between temp reads in seconds
#define READ_INTERVAL 30

//I2C SENSOR//
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
#define SENSOR_ADDR                 0x93        /*!< Slave address of the temp sensor  1001 001*/

//THERMISTOR ADC//
#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 64 //Taking 64 samples per reading
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_4;
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
#define T0 298.15 //Room Temp in Kelvin
#define R0 10000 //Resitor used in voltage divider
#define ThermBeta 3380.0 //Beta Constant 

//BATTERY ADC//
/*
static const adc_channel_t batt_channel = ADC_CHANNEL_5;
#define BATT_MAX_VOLTAGE 3.6
*/

// Data packet
struct packet {
	int64_t time; // Epoch time
	int32_t sensor_temp; // Temperature in C, multiplied by 1000
	int32_t thermistor_temp; // Temp in degrees C, -2^31 is unused
	uint8_t battery; // Percentage of battery capacity, 0 to 100, 255 means no battery or no measurement
	uint16_t data_len; // 0 for us
	//uint8_t teamdata[2];
} __attribute__((packed));

const int PACKET_SIZE = 8 + 4 + 4 + 1 + 2;

// Function declarations
void wifi_init(void);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
uint8_t myPow(uint8_t base, uint8_t exponent);
static esp_err_t i2c_master_init(void);
static esp_err_t temp_register_read(uint8_t *data, size_t len);
float convertTemp(uint8_t data[]);
void config_ADC(void);


void app_main() {
    // Start I2C and ADC
    i2c_master_init();
    config_ADC();

    // Start wifi
    wifi_init();

    // Initialize MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = BROKER_URI,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);

    // Start the client
    esp_mqtt_client_start(client);

}


// Does some magic to make the wifi work
void wifi_init(void) {
    // `ESP_ERROR_CHECK` is a macro which checks that the return value of a function is
    // `ESP_OK`.  If not, it prints some debug information and aborts the program.

    // Enable Flash (aka non-volatile storage, NVS)
    // The WiFi stack uses this to store some persistent information
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize the network software stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Initialize the event loop, a separate thread which checks for events
    // (like WiFi being connected or receiving an MQTT message) and dispatches
    // functions to handle them.
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Now connect to WiFi
    ESP_ERROR_CHECK(example_connect());
}


// Decides what to do when an MQTT event happens
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    const char *TAG = "MQTT_HANDLE";
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {

    // Send message once connected
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        // Get IC temperature readings
        uint8_t IC_data[2];
        float IC_Temp;
        ESP_ERROR_CHECK(temp_register_read(IC_data, 2)); 
        printf("IC Temp = %d + %d\n", IC_data[0], IC_data[1]);
        IC_Temp = convertTemp(IC_data);
        printf("IC Temp = %f ˚C\n",IC_Temp);
        int ic_convert_temp = IC_Temp * 1000;
        // make sure to change the variable back

        // Get ADC temperature readings
        float ThermRes = 0;
        float ThermTempK = 0;
        float ThermTempC = 0;
        uint32_t thermReading = 0;

        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            thermReading += adc1_get_raw((adc1_channel_t)channel);
        }
        thermReading /= NO_OF_SAMPLES;

        // Convert adc_reading to voltage in mV
        float voltage = esp_adc_cal_raw_to_voltage(thermReading, adc_chars);
        ThermRes = R0 * ((3300/voltage)-1);

        // Test #1 (Approx Temperature)
        ThermTempK = 1/((1/T0) + ((1/ThermBeta) * log(ThermRes/R0)));
        ThermTempC = ThermTempK - 273.15;
        printf("Temp Approx: %f K = %f ˚C\n", ThermTempK, ThermTempC);
        int therm_convert_temp = ThermTempC * 1000;

        // Get battery voltage
        /*
        int32_t battReading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            battReading += adc1_get_raw((adc1_channel_t)batt_channel);
        }
        battReading /= NO_OF_SAMPLES;
        float batt_voltage = esp_adc_cal_raw_to_voltage(battReading, adc_chars);
        batt_voltage = ((batt_voltage * 2) / BATT_MAX_VOLTAGE) * 100; // Adjust for divider and convert to percentage
        uint8_t batt_life = batt_voltage;
        */

        // Construct packet
        struct packet data_packet = {1, ic_convert_temp, therm_convert_temp, 100, 0};
        void *data_ptr = &data_packet;

        // Send the message
        msg_id = esp_mqtt_client_publish(client, NODE_PATH, (char *)data_ptr, PACKET_SIZE, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        break;

    // Disconnect + go back to sleep once acknowledgement of publish recieved
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);

        // Disconnect from wifi and MQTT client
        ESP_ERROR_CHECK(esp_mqtt_client_disconnect(client));
        ESP_ERROR_CHECK(example_disconnect());

        // Sleep
        esp_sleep_enable_timer_wakeup(READ_INTERVAL * 1000000);
        esp_deep_sleep_start();
        break;

    // Log event before connect to prevent suspicious event message
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
        break;

    // Handle other events
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

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
    i2c_master_write_byte(cmd, SENSOR_ADDR, ACK_CHECK_EN); //0x93 is the slave address hardcoded (the 1 in the LSB tells that it's a read)
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

void config_ADC(void) {
    //Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
}