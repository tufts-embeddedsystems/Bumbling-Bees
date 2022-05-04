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

// Address of MQTT server
#define BROKER_URI "mqtt://en1-pi.eecs.tufts.edu"

// How long to sleep between temp reads in seconds
#define READ_INTERVAL 30

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


void app_main() {
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

        // Construct packet
        struct packet data_packet = {1650992374, 20000, 21000, 13, 0};
        void *data_ptr = &data_packet;

        /*
        // Print hex data
        char *test = data_ptr;
        for (int i = 0; i < PACKET_SIZE; i++) {
            printf("%x ", test[i]);
        }
        printf("\n");
        */

        // Send the message
        msg_id = esp_mqtt_client_publish(client, "nodes/bumbling-bees/test3", (char *)data_ptr, PACKET_SIZE, 1, 0);
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
