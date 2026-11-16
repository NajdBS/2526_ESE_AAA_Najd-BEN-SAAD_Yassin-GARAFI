/*
 * input_analog.h
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#ifndef INC_INPUT_ANALOG_H_
#define INC_INPUT_ANALOG_H_

#include "stm32g4xx_hal.h"
#include "adc.h"
#include "user_interface/shell.h"

/* Buffer DMA : Rank 1 = Bus_Imes (PC2 / CH8), Rank 2 = U_Imes (PA1 / CH2) */
#define ADC_DMA_BUF_SIZE     2U
#define ADC_RANK_BUS_IDX     0U
#define ADC_RANK_U_IDX       1U

/* Parametres ADC 12 bits et capteur LEM GO 10-SME/SP3 (offset 1.65 V, 50 mV/A) */
#define ADC_FULL_SCALE       4095UL
#define ADC_VREF_MV          3300UL
#define CUR_OFFSET_MV        1650L
#define CUR_GAIN_MA_PER_MV   20L

/* Calibration ADC1, demarrage DMA (sur TIM1_TRGO) et commande shell */
int input_analog_init(void);

/* Valeurs brutes et courants convertis en mA depuis le buffer DMA */
uint16_t get_adc_u_raw(void);
uint16_t get_adc_bus_raw(void);
int32_t get_current_u_mA(void);
int32_t get_current_bus_mA(void);

/* Commande shell : current */
int sh_current(h_shell_t *h_shell, int argc, char **argv);

#endif /* INC_INPUT_ANALOG_H_ */
