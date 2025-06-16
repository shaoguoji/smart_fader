#pragma once

#include "esp_err.h"

typedef struct user_pid_handle_t *user_pid_handle_t;

/**
 * @brief 初始化PID控制器
 * 
 * @param handle PID控制器句柄指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_output 输出上限
 * @param min_output 输出下限
 * @return esp_err_t 
 */
esp_err_t user_pid_init(user_pid_handle_t *handle, float kp, float ki, float kd, float max_output, float min_output);

/**
 * @brief 设置PID参数
 * 
 * @param handle PID控制器句柄
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @return esp_err_t 
 */
esp_err_t user_pid_set_params(user_pid_handle_t handle, float kp, float ki, float kd);

/**
 * @brief 获取PID参数
 * 
 * @param handle PID控制器句柄
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
esp_err_t user_pid_get_params(user_pid_handle_t handle, float *kp, float *ki, float *kd);

/**
 * @brief 设置目标值
 * 
 * @param handle PID控制器句柄
 * @param target 目标值
 * @return esp_err_t 
 */
esp_err_t user_pid_set_target(user_pid_handle_t handle, float target);

/**
 * @brief 计算PID输出
 * 
 * @param handle PID控制器句柄
 * @param current_value 当前值
 * @return float PID输出值
 */
float user_pid_calculate(user_pid_handle_t handle, float current_value);

/**
 * @brief 重置PID控制器状态
 * 
 * @param handle PID控制器句柄
 * @return esp_err_t 
 */
esp_err_t user_pid_reset(user_pid_handle_t handle);

/**
 * @brief 反初始化PID控制器
 * 
 * @param handle PID控制器句柄
 * @return esp_err_t 
 */
esp_err_t user_pid_deinit(user_pid_handle_t handle); 