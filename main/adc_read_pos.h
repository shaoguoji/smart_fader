#ifndef ADC_READ_POS_H
#define ADC_READ_POS_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the ADC position reading module
 * 
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t adc_pos_init(void);

/**
 * @brief Get the current filtered position value (12-bit ADC)
 * 
 * @param pos Pointer to store the position value (0-4095)
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t adc_pos_get_value(uint32_t *pos);

/**
 * @brief Deinitialize the ADC position reading module
 * 
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t adc_pos_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_READ_POS_H */ 