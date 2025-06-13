#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "esp_err.h"

#define MOTOR_IN1_GPIO    7    // IN1 pin connected to GPIO7
#define MOTOR_IN2_GPIO    8    // IN2 pin connected to GPIO8

#define PWM_FREQ_HZ         20000  // 20kHz PWM frequency
#define MOTOR_SPEED_MAX      4095    // Maximum duty cycle percentage
#define MOTOR_SPEED_MIN     -4095    // Minimum duty cycle percentage

static const char *TAG = "DC_MOTOR";

// Initialize the motor driver
esp_err_t dc_motor_init(void)
{
    ESP_LOGI(TAG, "Initializing DC motor driver...");

    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_IN1_GPIO) | (1ULL << MOTOR_IN2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO pins: %d", ret);
        return ret;
    }

    // Configure MCPWM
    mcpwm_config_t pwm_config = {
        .frequency = PWM_FREQ_HZ,
        .cmpr_a = 0,    // Initial duty cycle of PWMxA = 0
        .cmpr_b = 0,    // Initial duty cycle of PWMxB = 0
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };

    // Initialize MCPWM for motor control
    ret = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCPWM: %d", ret);
        return ret;
    }

    ret = mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_IN1_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCPWM GPIO A: %d", ret);
        return ret;
    }

    ret = mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_IN2_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCPWM GPIO B: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "DC motor driver initialized successfully");
    return ESP_OK;
}

// Set motor speed (-4095 to 4095)
void dc_motor_set_speed(int speed)
{
    // 限幅处理
    if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;
    if (speed < MOTOR_SPEED_MIN) speed = MOTOR_SPEED_MIN;

    // 将速度值转换为PWM占空比（0-100%）
    float duty_cycle;
    if (speed >= 0) {
        // 正向
        duty_cycle = (float)speed * 100.0f / MOTOR_SPEED_MAX;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    } else {
        // 反向
        duty_cycle = (float)(-speed) * 100.0f / MOTOR_SPEED_MAX;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);
    }

    // Update duty cycle
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

// Stop the motor
void dc_motor_stop(void)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

// Deinitialize the motor driver
esp_err_t dc_motor_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing DC motor driver...");

    // Stop the motor first
    dc_motor_stop();

    // Stop MCPWM timer
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

    // Reset GPIO pins to default state
    gpio_reset_pin(MOTOR_IN1_GPIO);
    gpio_reset_pin(MOTOR_IN2_GPIO);

    ESP_LOGI(TAG, "DC motor driver deinitialized successfully");
    return ESP_OK;
} 