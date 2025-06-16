#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "wifi_manager.h"

static const char *TAG = "WIFI_MANAGER";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define LED_GPIO 21
#define LED_BLINK_INTERVAL_MS 500

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static char s_ssid[32];
static char s_password[64];
static bool s_wifi_connected = false;
static TaskHandle_t s_led_task_handle = NULL;

static void led_task(void *pvParameters)
{
    bool led_state = false;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t blink_interval = pdMS_TO_TICKS(LED_BLINK_INTERVAL_MS);

    while (1) {
        if (s_wifi_connected) {
            // WiFi已连接，LED常亮
            gpio_set_level(LED_GPIO, 0);
        } else {
            // WiFi未连接，LED慢闪
            led_state = !led_state;
            gpio_set_level(LED_GPIO, led_state);
        }
        vTaskDelayUntil(&last_wake_time, blink_interval);
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        s_wifi_connected = false;
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_manager_init(const char* ssid, const char* password)
{
    // 配置LED GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // 创建LED控制任务
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, &s_led_task_handle);

    // 保存WiFi凭据
    strncpy(s_ssid, ssid, sizeof(s_ssid) - 1);
    strncpy(s_password, password, sizeof(s_password) - 1);

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 创建事件组
    s_wifi_event_group = xEventGroupCreate();

    // 初始化TCP/IP适配器
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册事件处理程序
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    return ESP_OK;
}

esp_err_t wifi_manager_start(void)
{
    // 配置WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
        },
    };
    strncpy((char*)wifi_config.sta.ssid, s_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, s_password, sizeof(wifi_config.sta.password));

    // 设置WiFi模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_manager_start finished.");

    return ESP_OK;
}

esp_err_t wifi_manager_stop(void)
{
    ESP_ERROR_CHECK(esp_wifi_stop());
    s_wifi_connected = false;
    return ESP_OK;
}

esp_err_t wifi_manager_deinit(void)
{
    // 停止LED任务
    if (s_led_task_handle != NULL) {
        vTaskDelete(s_led_task_handle);
        s_led_task_handle = NULL;
    }

    // 注销事件处理程序
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));

    // 删除事件组
    vEventGroupDelete(s_wifi_event_group);

    // 停止WiFi
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());

    return ESP_OK;
} 