#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化WiFi管理器
 * 
 * @param ssid WiFi的SSID
 * @param password WiFi的密码
 * @return esp_err_t 
 */
esp_err_t wifi_manager_init(const char* ssid, const char* password);

/**
 * @brief 启动WiFi连接
 * 
 * @return esp_err_t 
 */
esp_err_t wifi_manager_start(void);

/**
 * @brief 停止WiFi连接
 * 
 * @return esp_err_t 
 */
esp_err_t wifi_manager_stop(void);

/**
 * @brief 反初始化WiFi管理器
 * 
 * @return esp_err_t 
 */
esp_err_t wifi_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */ 