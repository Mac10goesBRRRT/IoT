#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
// Wifi stuff
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
//WIFi@HOME
#include "wifi_config.h"

#define I2C_MASTER_PORT 0         // port number
#define I2C_MASTER_SDA_IO 21      // GPIO data
#define I2C_MASTER_SCL_IO 22      // GPIO clock
#define I2C_MASTER_FREQ_HZ 400000 // I2C frequency


static const char *TAG = "wifi station";
bool wifi_established;

uint8_t lux_read_sensor_id();
u_int16_t lux_read_sensor_value(u_int8_t reg, u_int8_t addr);

void read_write_i2c(void *pvParameters);

void tcpip_adapter_init();

static void wifi_event_handler(
    void* arg, 
    esp_event_base_t event_base, 
    int32_t event_id, 
    void* event_data);

void app_main()
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    tcpip_adapter_init();
    /*TaskHandle_t xtask_main = NULL;
    xTaskCreate(&read_write_i2c, "readWritei2c", 4000, NULL, tskIDLE_PRIORITY, &xtask_main);
    configASSERT(xtask_main);*/
}

void read_write_i2c(void *pvParameters)
{
    for (;;)
    {
        printf("beep boop hell hier: %d\n", lux_read_sensor_value(0x29, 0));
        vTaskDelay(200); // 2 Sekunden
    }
}


void tcpip_adapter_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = false;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}


static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
    {
        if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
            wifi_established =false;
            ESP_LOGI(TAG, "connect to the AP");
            esp_wifi_connect();
        } else if(event_base == WIFI_EVENT && event_id == 
            WIFI_EVENT_STA_DISCONNECTED) {
            wifi_established = false;
            ESP_LOGI(TAG, "retry to connect to the AP");
            esp_wifi_connect();
        } else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            char* buf[20];
            ESP_LOGI(TAG, "got ip:%s", esp_ip4addr_ntoa(&event->ip_info.ip, buf, sizeof(buf)));
            wifi_established = true;
        } else {
            ESP_LOGI(TAG, "unhandled event (%s) with ID %d", event_base, (int)event_id);
        }
    }