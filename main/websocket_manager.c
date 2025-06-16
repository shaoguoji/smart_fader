#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "websocket_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"

static const char *TAG = "WEBSOCKET_MANAGER";

#define MAX_CLIENTS 4
#define MAX_MSG_SIZE 1024

static httpd_handle_t server = NULL;
static ws_msg_callback_t msg_callback = NULL;
static int client_sockets[MAX_CLIENTS] = {-1, -1, -1, -1};
static int client_count = 0;
static uint16_t server_port = 0;
static bool server_initialized = false;

// 添加新的客户端
static void add_client(int sock)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_sockets[i] == -1) {
            client_sockets[i] = sock;
            client_count++;
            ESP_LOGI(TAG, "Client connected. Socket: %d, Total clients: %d", sock, client_count);
            break;
        }
    }
}

// 移除客户端
static void remove_client(int sock)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_sockets[i] == sock) {
            client_sockets[i] = -1;
            client_count--;
            ESP_LOGI(TAG, "Client disconnected. Socket: %d, Total clients: %d", sock, client_count);
            break;
        }
    }
}

// 广播消息给所有客户端
static void broadcast_message(const char* message, size_t len)
{
    ESP_LOGD(TAG, "Broadcasting message to %d clients", client_count);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_sockets[i] != -1) {
            httpd_ws_frame_t ws_pkt;
            ws_pkt.payload = (uint8_t*)message;
            ws_pkt.len = len;
            ws_pkt.type = HTTPD_WS_TYPE_TEXT;
            esp_err_t ret = httpd_ws_send_frame_async(server, client_sockets[i], &ws_pkt);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send message to client %d: %s", client_sockets[i], esp_err_to_name(ret));
            } else {
                ESP_LOGD(TAG, "Message sent to client %d", client_sockets[i]);
            }
        }
    }
}

// WebSocket 握手处理
static esp_err_t ws_handshake(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Handshake failed: Invalid method");
    return ESP_FAIL;
}

// WebSocket 消息处理
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        return ws_handshake(req);
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d: %s", ret, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Received packet type: %d, length: %d", ws_pkt.type, ws_pkt.len);

    if (ws_pkt.len) {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d: %s", ret, esp_err_to_name(ret));
            free(buf);
            return ret;
        }
        ESP_LOGD(TAG, "Received message: %.*s", ws_pkt.len, (char*)ws_pkt.payload);
    }

    // 解析 JSON 消息
    cJSON *root = cJSON_Parse((char*)ws_pkt.payload);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        free(buf);
        return ESP_FAIL;
    }

    cJSON *type = cJSON_GetObjectItem(root, "type");
    if (type == NULL || !cJSON_IsString(type)) {
        ESP_LOGE(TAG, "Invalid message type");
        cJSON_Delete(root);
        free(buf);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Processing message type: %s", type->valuestring);

    // 处理不同类型的消息
    if (strcmp(type->valuestring, "fader") == 0) {
        cJSON *position = cJSON_GetObjectItem(root, "position");
        if (position && cJSON_IsNumber(position)) {
            int pos = position->valueint;
            ESP_LOGI(TAG, "Received fader position: %d", pos);
            if (msg_callback) {
                msg_callback(WS_MSG_TYPE_FADER, &pos);
            }
        }
    } else if (strcmp(type->valuestring, "pid") == 0) {
        cJSON *target = cJSON_GetObjectItem(root, "target");
        cJSON *params = cJSON_GetObjectItem(root, "params");
        if (target && params) {
            pid_params_t pid_params;
            cJSON *kp = cJSON_GetObjectItem(params, "kp");
            cJSON *ki = cJSON_GetObjectItem(params, "ki");
            cJSON *kd = cJSON_GetObjectItem(params, "kd");
            
            if (kp && ki && kd) {
                pid_params.kp = kp->valuedouble;
                pid_params.ki = ki->valuedouble;
                pid_params.kd = kd->valuedouble;
                
                ESP_LOGI(TAG, "Received PID parameters for %s: kp=%.2f, ki=%.2f, kd=%.2f",
                        target->valuestring, pid_params.kp, pid_params.ki, pid_params.kd);
                
                if (strcmp(target->valuestring, "speed") == 0) {
                    if (msg_callback) {
                        msg_callback(WS_MSG_TYPE_PID_SPEED, &pid_params);
                    }
                } else if (strcmp(target->valuestring, "Pos") == 0) {
                    if (msg_callback) {
                        msg_callback(WS_MSG_TYPE_PID_POS, &pid_params);
                    }
                }
            }
        }
    }

    cJSON_Delete(root);
    free(buf);
    return ESP_OK;
}

// 处理客户端连接
static esp_err_t ws_connect_handler(httpd_req_t *req)
{
    int sock = httpd_req_to_sockfd(req);
    ESP_LOGI(TAG, "New client connection request from socket: %d", sock);
    add_client(sock);
    return ESP_OK;
}

// 处理客户端断开连接
static esp_err_t ws_close_handler(httpd_req_t *req)
{
    int sock = httpd_req_to_sockfd(req);
    ESP_LOGI(TAG, "Client disconnection request from socket: %d", sock);
    remove_client(sock);
    return ESP_OK;
}

// WiFi 事件处理函数
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
        // 如果服务器已经初始化但未启动，则启动服务器
        if (server_initialized && server == NULL) {
            ESP_LOGI(TAG, "Starting WebSocket server after WiFi connection");
            websocket_manager_start();
        }
    }
}

esp_err_t websocket_manager_init(uint16_t port, ws_msg_callback_t callback)
{
    ESP_LOGI(TAG, "Initializing WebSocket server on port %d", port);
    msg_callback = callback;
    server_port = port;
    server_initialized = false;

    // 注册 WiFi 事件处理函数
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;
    config.stack_size = 8192;
    config.server_port = port;

    ESP_LOGI(TAG, "Starting HTTP Server");
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }

    httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL
    };

    ESP_LOGI(TAG, "Registering WebSocket URI handler");
    httpd_register_uri_handler(server, &ws);
    httpd_register_uri_handler(server, &(httpd_uri_t){
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_connect_handler,
        .user_ctx = NULL
    });

    server_initialized = true;
    ESP_LOGI(TAG, "WebSocket server initialized successfully");
    return ESP_OK;
}

esp_err_t websocket_manager_start(void)
{
    if (!server_initialized) {
        ESP_LOGE(TAG, "WebSocket server not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (server != NULL) {
        ESP_LOGW(TAG, "WebSocket server already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting WebSocket server");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;
    config.stack_size = 8192;
    config.server_port = server_port;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }

    httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(server, &ws);
    httpd_register_uri_handler(server, &(httpd_uri_t){
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_connect_handler,
        .user_ctx = NULL
    });

    ESP_LOGI(TAG, "WebSocket server started successfully");
    return ESP_OK;
}

esp_err_t websocket_manager_stop(void)
{
    if (server) {
        ESP_LOGI(TAG, "Stopping WebSocket server");
        httpd_stop(server);
        server = NULL;
    }
    return ESP_OK;
}

esp_err_t websocket_manager_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing WebSocket server");
    websocket_manager_stop();
    
    // 注销 WiFi 事件处理函数
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler));
    
    msg_callback = NULL;
    client_count = 0;
    memset(client_sockets, -1, sizeof(client_sockets));
    server_initialized = false;
    return ESP_OK;
}

esp_err_t websocket_manager_broadcast(ws_msg_type_t type, const void* data, size_t len)
{
    char* message = NULL;
    cJSON *root = cJSON_CreateObject();
    
    switch (type) {
        case WS_MSG_TYPE_FADER: {
            int position = *(int*)data;
            ESP_LOGD(TAG, "Broadcasting fader position: %d", position);
            cJSON_AddStringToObject(root, "type", "fader");
            cJSON_AddNumberToObject(root, "position", position);
            break;
        }
            
        case WS_MSG_TYPE_PID_SPEED:
        case WS_MSG_TYPE_PID_POS: {
            pid_params_t* params = (pid_params_t*)data;
            const char* target = type == WS_MSG_TYPE_PID_SPEED ? "speed" : "Pos";
            ESP_LOGD(TAG, "Broadcasting %s PID parameters: kp=%.2f, ki=%.2f, kd=%.2f",
                    target, params->kp, params->ki, params->kd);
            cJSON_AddStringToObject(root, "type", "pid");
            cJSON_AddStringToObject(root, "target", target);
            
            cJSON *params_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(params_obj, "kp", params->kp);
            cJSON_AddNumberToObject(params_obj, "ki", params->ki);
            cJSON_AddNumberToObject(params_obj, "kd", params->kd);
            cJSON_AddItemToObject(root, "params", params_obj);
            break;
        }
    }

    message = cJSON_PrintUnformatted(root);
    if (message) {
        broadcast_message(message, strlen(message));
        free(message);
    }
    
    cJSON_Delete(root);
    return ESP_OK;
} 