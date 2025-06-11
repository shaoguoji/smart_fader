#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dc_motor.h"

static const char *TAG = "MOTOR_TEST";

void motor_test_task(void *pvParameters)
{
    // Wait for 1 second after power on
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Starting motor test...");
    
    // Run motor at 10% speed
    dc_motor_set_speed(10);
    ESP_LOGI(TAG, "Motor running at 10%% speed");
    
    // Run for 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Stop motor
    dc_motor_stop();
    ESP_LOGI(TAG, "Motor test completed");
    
    // Delete the task
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize motor driver
    dc_motor_init();
    
    // Create motor test task
    xTaskCreate(motor_test_task, "motor_test", 2048, NULL, 5, NULL);
} 