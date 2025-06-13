#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"
#include "adc_read_pos.h"
#include "user_pid.h"
#include "dc_motor.h"

static const char *TAG = "MOTOR_TEST";

// 测试位置序列
#define TEST_POSITIONS_COUNT 4
static const float test_positions[TEST_POSITIONS_COUNT] = {
    0.0f,      // 起始位置
    1000.0f,   // 第一个目标位置
    2000.0f,   // 第二个目标位置
    0.0f       // 返回起始位置
};

// 位置环PID参数
#define POS_PID_KP 1.0f
#define POS_PID_KI 0.1f
#define POS_PID_KD 0.01f

// 速度环PID参数
#define SPEED_PID_KP 0.5f
#define SPEED_PID_KI 0.05f
#define SPEED_PID_KD 0.001f

// 位置停留时间（毫秒）
#define POSITION_HOLD_TIME_MS 2000

// 电机控制测试任务
static void motor_test_task(void *pvParameters)
{
    esp_err_t ret;
    int pos_index = 0;

    // 初始化电机控制系统
    ret = motor_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motor control system");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Motor control system initialized");

    // 设置PID参数
    ret = motor_control_set_pos_pid_params(POS_PID_KP, POS_PID_KI, POS_PID_KD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set position PID parameters");
        motor_control_deinit();
        vTaskDelete(NULL);
        return;
    }

    ret = motor_control_set_speed_pid_params(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set speed PID parameters");
        motor_control_deinit();
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "PID parameters set");

    // 启动电机控制
    ret = motor_control_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start motor control");
        motor_control_deinit();
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Motor control started");

    // 测试循环
    while (1) {
        // 设置目标位置
        float target_pos = test_positions[pos_index];
        ESP_LOGI(TAG, "Moving to position: %.1f", target_pos);
        
        ret = motor_control_set_pos(target_pos);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set target position");
            break;
        }

        // 等待到达目标位置并保持一段时间
        vTaskDelay(pdMS_TO_TICKS(POSITION_HOLD_TIME_MS));

        // 更新位置索引
        pos_index = (pos_index + 1) % TEST_POSITIONS_COUNT;
    }

    // 停止电机控制
    motor_control_stop();
    ESP_LOGI(TAG, "Motor control stopped");

    // 清理资源
    motor_control_deinit();
    ESP_LOGI(TAG, "Motor control system deinitialized");

    vTaskDelete(NULL);
}

void app_main(void)
{
    // 创建电机测试任务
    // xTaskCreate(motor_test_task, "motor_test", 4096, NULL, 5, NULL);
    // ESP_LOGI(TAG, "Motor test task created");

    motor_control_init();
    motor_control_start();

    // motor_control_set_pos(2000);

    vTaskDelay(pdMS_TO_TICKS(5000));

    motor_control_stop();
    
} 
