#ifndef WEBSOCKET_MANAGER_H
#define WEBSOCKET_MANAGER_H

#include "esp_err.h"

// WebSocket 消息类型
typedef enum {
    WS_MSG_TYPE_FADER = 0,
    WS_MSG_TYPE_PID_SPEED,
    WS_MSG_TYPE_PID_POS
} ws_msg_type_t;

// PID 参数结构体
typedef struct {
    float kp;
    float ki;
    float kd;
} pid_params_t;

// WebSocket 消息回调函数类型
typedef void (*ws_msg_callback_t)(ws_msg_type_t type, void* data);

/**
 * @brief 初始化 WebSocket 服务器
 * 
 * @param port WebSocket 服务器端口号
 * @param callback 消息回调函数
 * @return esp_err_t 
 */
esp_err_t websocket_manager_init(uint16_t port, ws_msg_callback_t callback);

/**
 * @brief 启动 WebSocket 服务器
 * 
 * @return esp_err_t 
 */
esp_err_t websocket_manager_start(void);

/**
 * @brief 停止 WebSocket 服务器
 * 
 * @return esp_err_t 
 */
esp_err_t websocket_manager_stop(void);

/**
 * @brief 反初始化 WebSocket 服务器
 * 
 * @return esp_err_t 
 */
esp_err_t websocket_manager_deinit(void);

/**
 * @brief 广播消息给所有连接的客户端
 * 
 * @param type 消息类型
 * @param data 消息数据
 * @param len 数据长度
 * @return esp_err_t 
 */
esp_err_t websocket_manager_broadcast(ws_msg_type_t type, const void* data, size_t len);

#endif /* WEBSOCKET_MANAGER_H */ 