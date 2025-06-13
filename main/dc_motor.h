#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the DC motor driver
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t dc_motor_init(void);

/**
 * @brief Set motor speed
 * @param speed Speed value from -4095 to 4095
 *              Positive values for forward direction
 *              Negative values for reverse direction
 *              The speed value will be automatically converted to PWM duty cycle (0-100%)
 */
void dc_motor_set_speed(int speed);

/**
 * @brief Stop the motor
 */
void dc_motor_stop(void);

/**
 * @brief Deinitialize the DC motor driver
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t dc_motor_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_H */ 