#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"

#define MOTOR_IN1_GPIO    7    // IN1 pin connected to GPIO7
#define MOTOR_IN2_GPIO    8    // IN2 pin connected to GPIO8

#define PWM_FREQ_HZ       20000  // 20kHz PWM frequency
#define PWM_DUTY_MAX      100    // Maximum duty cycle percentage

static const char *TAG = "DC_MOTOR";

// Initialize the motor driver
void dc_motor_init(void)
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
    gpio_config(&io_conf);

    // Configure MCPWM
    mcpwm_config_t pwm_config = {
        .frequency = PWM_FREQ_HZ,
        .cmpr_a = 0,    // Initial duty cycle of PWMxA = 0
        .cmpr_b = 0,    // Initial duty cycle of PWMxB = 0
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };

    // Initialize MCPWM for motor control
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_IN1_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_IN2_GPIO);

    ESP_LOGI(TAG, "DC motor driver initialized successfully");
}

// Set motor speed (-100 to 100)
void dc_motor_set_speed(int speed)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    if (speed >= 0) {
        // Forward direction
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    } else {
        // Reverse direction
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, -speed);
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