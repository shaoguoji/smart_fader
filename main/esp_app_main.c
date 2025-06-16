#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"
#include "adc_read_pos.h"
#include "user_pid.h"
#include "dc_motor.h"
#include "wifi_manager.h"
#include "websocket_manager.h"

static const char *TAG = "MOTOR_TEST";

// 测试位置序列
#define TEST_POSITIONS_COUNT 4
static const float test_positions[TEST_POSITIONS_COUNT] = {
    0.0f,      // 起始位置
    4000.0f,   // 第一个目标位置
    0.0f,      // 第二个目标位置
    2000.0f    // 第三个目标位置
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

// WebSocket 消息回调函数
static void ws_message_handler(ws_msg_type_t type, void* data)
{
    switch (type) {
        case WS_MSG_TYPE_FADER: {
            int position = *(int*)data;
            // 更新滑块位置
            motor_control_set_pos(position);
            break;
        }
        case WS_MSG_TYPE_PID_SPEED: {
            pid_params_t* params = (pid_params_t*)data;
            // 更新速度 PID 参数
            motor_control_set_speed_pid_params(params->kp, params->ki, params->kd);
            break;
        }
        case WS_MSG_TYPE_PID_POS: {
            pid_params_t* params = (pid_params_t*)data;
            // 更新位置 PID 参数
            motor_control_set_pos_pid_params(params->kp, params->ki, params->kd);
            break;
        }
    }
}

// 状态更新任务
static void status_update_task(void *pvParameters)
{
    while (1) {
        // 获取当前滑块位置
        float current_position = motor_control_get_current_pos();
        int pos_int = (int)current_position;
        // 广播位置更新
        websocket_manager_broadcast(WS_MSG_TYPE_FADER, &pos_int, sizeof(pos_int));
        
        // 获取当前 PID 参数
        pid_params_t speed_pid, pos_pid;
        motor_control_get_speed_pid(&speed_pid.kp, &speed_pid.ki, &speed_pid.kd);
        motor_control_get_position_pid(&pos_pid.kp, &pos_pid.ki, &pos_pid.kd);
        
        // 广播 PID 参数更新
        websocket_manager_broadcast(WS_MSG_TYPE_PID_SPEED, &speed_pid, sizeof(speed_pid));
        websocket_manager_broadcast(WS_MSG_TYPE_PID_POS, &pos_pid, sizeof(pos_pid));
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms 更新一次
    }
}

// 电机控制测试任务
static void motor_test_task(void *pvParameters)
{
    esp_err_t ret;
    int pos_index = 0;

    motor_control_start();

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
    // 初始化 WiFi
    ESP_ERROR_CHECK(wifi_manager_init("阿国的iPhone", "88888888"));
    ESP_ERROR_CHECK(wifi_manager_start());

    // 初始化 WebSocket 服务器，端口改为 1234
    ESP_ERROR_CHECK(websocket_manager_init(1234, ws_message_handler));
    ESP_ERROR_CHECK(websocket_manager_start());

    // 初始化电机控制
    motor_control_init();

    // 创建状态更新任务
    xTaskCreate(status_update_task, "status_update", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Status update task created");

    // 创建电机测试任务（可选）
    // xTaskCreate(motor_test_task, "motor_test", 4096, NULL, 5, NULL);
    // ESP_LOGI(TAG, "Motor test task created");
} 
