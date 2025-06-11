/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"

#define ADC_POS_UNIT                    ADC_UNIT_1
#define _ADC_POS_UNIT_STR(unit)         #unit
#define ADC_POS_UNIT_STR(unit)          _ADC_POS_UNIT_STR(unit)
#define ADC_POS_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define ADC_POS_ATTEN                   ADC_ATTEN_DB_11
#define ADC_POS_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#define ADC_POS_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_POS_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define ADC_POS_GET_DATA(p_data)        ((p_data)->type2.data)

#define ADC_POS_READ_LEN                    16
#define FILTER_WINDOW_SIZE                  8

static adc_channel_t channel[1] = {ADC_CHANNEL_0};

static TaskHandle_t s_task_handle;
static const char *TAG = "ADC_POS";
static SemaphoreHandle_t s_adc_sem = NULL;

static uint32_t filter_buffer[FILTER_WINDOW_SIZE] = {0};
static uint8_t filter_index = 0;

static uint8_t result[ADC_POS_READ_LEN] = {0};
static adc_continuous_handle_t s_handle = NULL;

static uint32_t filtered_data = 0;

static uint32_t apply_filter(uint32_t new_value) {
    filter_buffer[filter_index] = new_value;
    filter_index = (filter_index + 1) % FILTER_WINDOW_SIZE;
    
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        sum += filter_buffer[i];
    }
    return sum / FILTER_WINDOW_SIZE;
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    xSemaphoreGiveFromISR(s_adc_sem, &mustYield);
    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num)
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 32,
        .conv_frame_size = ADC_POS_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &s_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 10 * 1000,
        .conv_mode = ADC_POS_CONV_MODE,
        .format = ADC_POS_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_POS_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_POS_UNIT;
        adc_pattern[i].bit_width = ADC_POS_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(s_handle, &dig_cfg));
}

void adc_pos_update_task(void *pvParameters)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[ADC_POS_READ_LEN] = {0};
    memset(result, 0xcc, ADC_POS_READ_LEN);

    while (1) {
        xSemaphoreTake(s_adc_sem, portMAX_DELAY);
        char unit[] = ADC_POS_UNIT_STR(ADC_POS_UNIT);
    
        while (1) {
            ret = adc_continuous_read(s_handle, result, ADC_POS_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = ADC_POS_GET_CHANNEL(p);
                    uint32_t data = ADC_POS_GET_DATA(p);
                    if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_POS_UNIT)) {
                        filtered_data = apply_filter(data);
                    }
                }
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(s_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(s_handle));
}

esp_err_t adc_pos_init(void)
{
    s_adc_sem = xSemaphoreCreateBinary();
    if (s_adc_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create ADC semaphore");
        return ESP_ERR_NO_MEM;
    }

    s_task_handle = xTaskGetCurrentTaskHandle();
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };

    xTaskCreate(adc_pos_update_task, "adc_pos_update_task", 2048, NULL, 5, NULL);

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(s_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(s_handle));

    return ESP_OK;
}


esp_err_t adc_pos_get_value(uint32_t *pos)
{
    if (pos == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *pos = filtered_data;
    return ESP_OK;
}

esp_err_t adc_pos_deinit(void)
{
    if (s_handle) {
        ESP_ERROR_CHECK(adc_continuous_stop(s_handle));
        ESP_ERROR_CHECK(adc_continuous_deinit(s_handle));
        s_handle = NULL;
    }
    
    if (s_adc_sem) {
        vSemaphoreDelete(s_adc_sem);
        s_adc_sem = NULL;
    }
    
    return ESP_OK;
}
