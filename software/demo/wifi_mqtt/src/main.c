// Bumbling Bees temperature sensor code
// Embedded Systems, spring 2022

#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // Used for timer delay
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "minimal_wifi.h"
#include "esp_sleep.h"

#define BROKER_URI "mqtt://en1-pi.eecs.tufts.edu"

// How long between temp reads in seconds
#define READ_INTERVAL 20


struct packet {
	time_t time; // Epoch time
	int32_t sensor_temp; // Temperature in C, multiplied by 1000
	int32_t thermistor_temp; // Temp in degrees C, -2^31 is unused
	uint8_t battery; // Percentage of battery capacity, 0 to 100, 255 means no battery or no measurement
	uint16_t data_len;
	//uint8_t teamdata[data_len];
};

void wifi_init(void);


void app_main() {
    // Start wifi
    wifi_init();

    // Initialize MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = BROKER_URI,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    // Check time and temperature here eventually and stick the data in the packet
    struct packet data_packet = {0, 20, 21, 50, 0};

    // Send MQTT packet
    esp_mqtt_client_publish(client, "/nodes/bumbling-bees", (void *)(&data_packet), 0, 0, 0);

    // Disconnect from wifi and MQTT client
    ESP_ERROR_CHECK(example_disconnect());
    ESP_ERROR_CHECK(esp_mqtt_client_disconnect(client));

    // Sleep
    esp_sleep_enable_timer_wakeup(READ_INTERVAL * 1000000);
    esp_deep_sleep_start();
}



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