#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the DC motor driver
 */
void dc_motor_init(void);

/**
 * @brief Set motor speed
 * @param speed Speed value from -100 to 100
 *              Positive values for forward direction
 *              Negative values for reverse direction
 */
void dc_motor_set_speed(int speed);

/**
 * @brief Stop the motor
 */
void dc_motor_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_H */ 