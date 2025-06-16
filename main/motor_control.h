#pragma once

#include "esp_err.h"

/**
 * @brief 初始化电机控制系统
 * 
 * 初始化包括：
 * 1. 创建互斥锁
 * 2. 初始化ADC位置读取
 * 3. 初始化电机控制
 * 4. 初始化位置环和速度环PID控制器
 * 
 * @return esp_err_t 
 *     - ESP_OK: 初始化成功
 *     - ESP_ERR_NO_MEM: 内存分配失败
 *     - 其他错误码: 初始化失败
 */
esp_err_t motor_control_init(void);

/**
 * @brief 启动电机控制
 * 
 * 启动包括：
 * 1. 重置PID控制器
 * 2. 初始化定时器
 * 3. 启动控制循环
 * 
 * @return esp_err_t 
 *     - ESP_OK: 启动成功
 *     - ESP_ERR_INVALID_STATE: 控制已经在运行
 *     - ESP_ERR_TIMEOUT: 获取互斥锁超时
 *     - 其他错误码: 启动失败
 */
esp_err_t motor_control_start(void);

/**
 * @brief 停止电机控制
 * 
 * 停止包括：
 * 1. 停止定时器
 * 2. 停止电机
 * 
 * @return esp_err_t 
 *     - ESP_OK: 停止成功
 *     - ESP_ERR_TIMEOUT: 获取互斥锁超时
 */
esp_err_t motor_control_stop(void);

/**
 * @brief 设置电机目标位置
 * 
 * @param target_position 目标位置值
 * @return esp_err_t 
 *     - ESP_OK: 设置成功
 *     - ESP_ERR_INVALID_STATE: 控制未启动
 */
esp_err_t motor_control_set_pos(float target_position);

/**
 * @brief 设置位置环PID参数
 * 
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @return esp_err_t 
 *     - ESP_OK: 设置成功
 *     - ESP_ERR_INVALID_STATE: 控制未启动
 */
esp_err_t motor_control_set_pos_pid_params(float kp, float ki, float kd);

/**
 * @brief 设置速度环PID参数
 * 
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @return esp_err_t 
 *     - ESP_OK: 设置成功
 *     - ESP_ERR_INVALID_STATE: 控制未启动
 */
esp_err_t motor_control_set_speed_pid_params(float kp, float ki, float kd);

esp_err_t motor_control_get_speed_pid(float *kp, float *ki, float *kd);

esp_err_t motor_control_get_position_pid(float *kp, float *ki, float *kd);

float motor_control_get_current_pos(void);

/**
 * @brief 反初始化电机控制系统
 * 
 * 清理包括：
 * 1. 停止控制
 * 2. 释放定时器资源
 * 3. 释放PID控制器资源
 * 4. 释放电机控制资源
 * 5. 释放ADC资源
 * 6. 释放互斥锁
 * 
 * @return esp_err_t 
 *     - ESP_OK: 清理成功
 */
esp_err_t motor_control_deinit(void); 