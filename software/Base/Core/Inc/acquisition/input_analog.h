/*
 * input_analog.h
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#ifndef INC_INPUT_ANALOG_H_
#define INC_INPUT_ANALOG_H_

#include "stm32g4xx_hal.h"

/* Capteurs de courant LEM GO 10-SME/SP3 : U_out = U_ref + G * I
 * G = 50 mV/A, U_ref = 1,65 V environ : le zero reel est mesure a courant nul */
#define ANALOG_SENSITIVITY_V_PER_A  0.050f
#define ANALOG_ADC_VREF_V           3.3f
#define ANALOG_ADC_MAX              4095.0f     /* 12 bits : 1 LSB = 0,8 mV = 16 mA */
#define ANALOG_MILLI_PER_UNIT       1000.0f

/* 47,5 + 12,5 cycles a 42,5 MHz = 1,41 us par conversion */
#define ANALOG_SAMPLING_TIME        ADC_SAMPLETIME_47CYCLES_5

/* Moyenne de plusieurs conversions : en polling, l'instant de mesure n'est pas
 * synchronise a la PWM et chaque valeur tombe ailleurs sur l'ondulation du courant */
#define ANALOG_POLLING_SAMPLES      64U
#define ANALOG_POLLING_TIMEOUT_MS   10U

/* Calibre l'ADC1, mesure le zero des capteurs (PWM coupees) et ajoute les commandes shell */
int input_analog_init(void);

#endif /* INC_INPUT_ANALOG_H_ */
