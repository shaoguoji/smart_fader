#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/timer.h"
#include "soc/rtc.h"
#include "motor_control.h"
#include "user_pid.h"
#include "adc_read_pos.h"
#include "dc_motor.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"

static const char *TAG = "MOTOR_CTRL";

#define MOTOR_CTRL_TIMER_GROUP        TIMER_GROUP_0
#define MOTOR_CTRL_TIMER_ID           TIMER_0
#define MOTOR_CTRL_TIMER_DIVIDER      16
#define MOTOR_CTRL_TIMER_FREQ_HZ      20000  // 20kHz控制频率
#define TIMER_BASE_CLK                (APB_CLK_FREQ)  // 80MHz

// 位置环PID参数
#define POS_PID_KP                   1.0f
#define POS_PID_KI                   0.0f
#define POS_PID_KD                   0.1f
#define POS_PID_MAX_OUTPUT          4095.0f  // 最大速度限制
#define POS_PID_MIN_OUTPUT          -4095.0f

// 速度环PID参数
#define SPEED_PID_KP                5.0f
#define SPEED_PID_KI                0.0001f
#define SPEED_PID_KD                0.01f
#define SPEED_PID_MAX_OUTPUT        3000.0f  // 最大PWM输出
#define SPEED_PID_MIN_OUTPUT        -3000.0f

// 速度计算参数
#define SPEED_FILTER_FACTOR         0.5f    // 速度滤波系数

static bool s_motor_ctrl_running = false;
static float s_last_position = 0.0f;
static float s_current_speed = 0.0f;
static user_pid_handle_t s_pos_pid = NULL;    // 位置环PID控制器
static user_pid_handle_t s_speed_pid = NULL;  // 速度环PID控制器

static esp_timer_handle_t periodic_timer;

static void periodic_timer_callback(void* arg);
static float calculate_speed(float current_pos);
static void motor_control_cal(void);

void periodic_timer_callback(void* arg)
{
    if (s_motor_ctrl_running) {
        motor_control_cal();
    }
    // ESP_LOGI(TAG, "periodic_timer_callback");
    // ESP_LOGI(TAG, "Current speed: %.2f", s_current_speed);
}

// 计算当前速度（单位：位置/秒）
static float calculate_speed(float current_pos)
{
    static bool is_first_call = true;
    float speed;
    
    if (is_first_call) {
        speed = 0.0f;
        s_current_speed = 0.0f;
        is_first_call = false;
    } else {
        speed = (current_pos - s_last_position) * MOTOR_CTRL_TIMER_FREQ_HZ;
        // 低通滤波
        s_current_speed = s_current_speed * (1.0f - SPEED_FILTER_FACTOR) + speed * SPEED_FILTER_FACTOR;
    }
    
    s_last_position = current_pos;
    return s_current_speed;
}

static void motor_control_cal(void)
{
    uint32_t current_pos;
    float pos_pid_output, speed_pid_output;
    esp_err_t ret;

    ret = adc_pos_get_value(&current_pos);
    if (ret != ESP_OK) {
        dc_motor_set_speed(0);  // 出错时停止电机
        return;
    }

    // 计算当前速度
    float current_speed = calculate_speed((float)current_pos);
    
    // user_pid_set_target(s_pos_pid, 2000);

    // 位置环PID计算，输出为速度指令
    pos_pid_output = user_pid_calculate(s_pos_pid, (float)current_pos);
    
    // 速度环PID计算，输出为PWM值
    user_pid_set_target(s_speed_pid, pos_pid_output);
    speed_pid_output = user_pid_calculate(s_speed_pid, current_speed);

    // 应用电机输出
    dc_motor_set_speed((int)speed_pid_output);
}

// 初始化定时器
static esp_err_t init_timer(void)
{
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic"
    };

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    
    return ESP_OK;
}

esp_err_t motor_control_init(void)
{
    esp_err_t ret;

    // 初始化位置环PID控制器
    user_pid_init(&s_pos_pid, POS_PID_KP, POS_PID_KI, POS_PID_KD, 
                       POS_PID_MAX_OUTPUT, POS_PID_MIN_OUTPUT);

    // 初始化速度环PID控制器
    user_pid_init(&s_speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
                       SPEED_PID_MAX_OUTPUT, SPEED_PID_MIN_OUTPUT);

    // 初始化ADC位置读取
    adc_pos_init();
    // 初始化电机控制
    dc_motor_init();
    // 初始化定时器
    init_timer();

    return ESP_OK;
}

esp_err_t motor_control_start(void)
{
    if (s_motor_ctrl_running) {
        return ESP_ERR_INVALID_STATE;
    }

    s_motor_ctrl_running = true;

    user_pid_reset(s_pos_pid);
    user_pid_reset(s_speed_pid);

    // 启动定时器
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000/MOTOR_CTRL_TIMER_FREQ_HZ));

    return ESP_OK;
}

esp_err_t motor_control_stop(void)
{
    s_motor_ctrl_running = false;

    // 停止定时器
    ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));

    // 停止电机
    dc_motor_set_speed(0);

    return ESP_OK;
}

esp_err_t motor_control_set_pos(float target_position)
{
    // if (!s_motor_ctrl_running) {
        // return ESP_ERR_INVALID_STATE;
    // }

    return user_pid_set_target(s_pos_pid, target_position);
}

esp_err_t motor_control_set_pos_pid_params(float kp, float ki, float kd)
{
    // if (!s_motor_ctrl_running) {
    //     return ESP_ERR_INVALID_STATE;
    // }

    return user_pid_set_params(s_pos_pid, kp, ki, kd);
}

esp_err_t motor_control_set_speed_pid_params(float kp, float ki, float kd)
{
    // if (!s_motor_ctrl_running) {
    //     return ESP_ERR_INVALID_STATE;
    // }

    return user_pid_set_params(s_speed_pid, kp, ki, kd);
}

float motor_control_get_current_pos(void)
{
    uint32_t pos;
    adc_pos_get_value(&pos);
    return (float)pos;
}

esp_err_t motor_control_deinit(void)
{
    // 停止控制
    motor_control_stop();

    // 删除PID控制器
    user_pid_deinit(s_pos_pid);
    user_pid_deinit(s_speed_pid);

    // 删除电机控制
    dc_motor_deinit();

    // 删除ADC位置读取
    adc_pos_deinit();

    // 删除定时器
    esp_timer_delete(periodic_timer);

    return ESP_OK;
}

esp_err_t motor_control_get_speed_pid(float *kp, float *ki, float *kd)
{
    if (!s_speed_pid) {
        return ESP_ERR_INVALID_STATE;
    }

    return user_pid_get_params(s_speed_pid, kp, ki, kd);
}

esp_err_t motor_control_get_position_pid(float *kp, float *ki, float *kd)
{
    if (!s_pos_pid) {
        return ESP_ERR_INVALID_STATE;
    }

    return user_pid_get_params(s_pos_pid, kp, ki, kd);
}
