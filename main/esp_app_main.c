#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dc_motor.h"
#include "adc_read_pos.h"

static const char *TAG = "MOTOR_TEST";

void motor_test_task(void *pvParameters)
{
    // Wait for 1 second after power on
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Starting motor test...");
    
    while (1) {
        // Forward direction
        dc_motor_set_speed(70);
        ESP_LOGI(TAG, "Motor running forward at 70%% speed");
        vTaskDelay(pdMS_TO_TICKS(300));
        
        // Stop briefly
        dc_motor_stop();
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // Reverse direction
        dc_motor_set_speed(-70);
        ESP_LOGI(TAG, "Motor running reverse at 70%% speed");
        vTaskDelay(pdMS_TO_TICKS(300));
        
        // Stop briefly
        dc_motor_stop();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void adc_pos_test_task(void *pvParameters)
{
    uint32_t pos;
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Starting ADC position test...");
    
    while (1) {
        ret = adc_pos_get_value(&pos);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "ADC Position: %lu (0x%lx)", pos, pos);
        } else {
            ESP_LOGE(TAG, "Failed to get ADC position: %d", ret);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 每10ms读取一次
    }
}

void app_main(void)
{
    // Initialize motor driver and ADC
    dc_motor_init();
    adc_pos_init();
    
    // Create tasks
    xTaskCreate(motor_test_task, "motor_test", 4096, NULL, 5, NULL);
    xTaskCreate(adc_pos_test_task, "adc_test", 4096, NULL, 5, NULL);
} 
