/*
 * analog_input.c
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#include "acquisition/input_analog.h"
#include <stdio.h>

/* Tableau mis a jour automatiquement par le DMA a chaque TIM1_TRGO */
static volatile uint16_t adc_dma_buf[ADC_DMA_BUF_SIZE] = {0};

static int32_t adc_to_current_mA(uint16_t raw)
{
    int32_t v_mv = (int32_t)(((uint32_t)raw * ADC_VREF_MV) / ADC_FULL_SCALE);
    return (v_mv - CUR_OFFSET_MV) * CUR_GAIN_MA_PER_MV;
}

int input_analog_init(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_DMA_BUF_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }

    return shell_add(&hshell1, "current", sh_current, "Mesure courant DMA (mA)");
}

/* Impulsion sur PA5 (USR_LED) a chaque fin de conversion DMA pour verif oscillo */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
    }
}

uint16_t get_adc_u_raw(void)
{
    return adc_dma_buf[ADC_RANK_U_IDX];
}

uint16_t get_adc_bus_raw(void)
{
    return adc_dma_buf[ADC_RANK_BUS_IDX];
}

int32_t get_current_u_mA(void)
{
    return adc_to_current_mA(adc_dma_buf[ADC_RANK_U_IDX]);
}

int32_t get_current_bus_mA(void)
{
    return adc_to_current_mA(adc_dma_buf[ADC_RANK_BUS_IDX]);
}

int sh_current(h_shell_t *h_shell, int argc, char **argv)
{
    uint16_t raw_u = adc_dma_buf[ADC_RANK_U_IDX];
    uint16_t raw_bus = adc_dma_buf[ADC_RANK_BUS_IDX];
    int32_t i_u = adc_to_current_mA(raw_u);
    int32_t i_bus = adc_to_current_mA(raw_bus);

    int size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "I_U=%ld mA (ADC=%u)\r\n",
                        (long)i_u, (unsigned int)raw_u);
    h_shell->drv.transmit(h_shell->print_buffer, size);

    size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                    "I_bus=%ld mA (ADC=%u)\r\n",
                    (long)i_bus, (unsigned int)raw_bus);
    h_shell->drv.transmit(h_shell->print_buffer, size);

    return HAL_OK;
}
