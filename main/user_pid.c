#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "user_pid.h"

static const char *TAG = "USER_PID";

typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float max_output;   // 输出上限
    float min_output;   // 输出下限
    float target;       // 目标值
    float last_error;   // 上次误差
    float integral;     // 积分项
} user_pid_t;

esp_err_t user_pid_init(user_pid_handle_t *handle, float kp, float ki, float kd, float max_output, float min_output)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    user_pid_t *pid = malloc(sizeof(user_pid_t));
    if (pid == NULL) {
        return ESP_ERR_NO_MEM;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->min_output = min_output;
    pid->target = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;

    *handle = (user_pid_handle_t)pid;
    return ESP_OK;
}

esp_err_t user_pid_set_params(user_pid_handle_t handle, float kp, float ki, float kd)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    user_pid_t *pid = (user_pid_t *)handle;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    return ESP_OK;
}

esp_err_t user_pid_set_target(user_pid_handle_t handle, float target)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    user_pid_t *pid = (user_pid_t *)handle;
    pid->target = target;
    return ESP_OK;
}

esp_err_t user_pid_get_params(user_pid_handle_t handle, float *kp, float *ki, float *kd)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    user_pid_t *pid = (user_pid_t *)handle;
    *kp = pid->kp;
    *ki = pid->ki;
    *kd = pid->kd;

    return ESP_OK;
}

float user_pid_calculate(user_pid_handle_t handle, float current_value)
{
    if (handle == NULL) {
        return 0.0f;
    }

    user_pid_t *pid = (user_pid_t *)handle;
    float error = pid->target - current_value;
    float derivative = error - pid->last_error;
    
    // 计算积分项
    pid->integral += error;
    
    // 计算PID输出
    float output = pid->kp * error + 
                  pid->ki * pid->integral + 
                  pid->kd * derivative;
    
    // 输出限幅
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < pid->min_output) {
        output = pid->min_output;
    }
    
    pid->last_error = error;
    return output;
}

esp_err_t user_pid_reset(user_pid_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    user_pid_t *pid = (user_pid_t *)handle;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    return ESP_OK;
}

esp_err_t user_pid_deinit(user_pid_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    free(handle);
    return ESP_OK;
} 