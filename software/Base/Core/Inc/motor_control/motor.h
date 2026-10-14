/*
 * motor.h
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#ifndef INC_MOTOR_CONTROL_MOTOR_H_
#define INC_MOTOR_CONTROL_MOTOR_H_

#include "stm32g4xx_hal.h"
#include "user_interface/shell.h"
#include "tim.h"

/* TIM1 centre-aligné : f_PWM = 170 MHz / (2 * 4250) = 20 kHz
 * ARR = 4249, dead-time = 34 (~200 ns) */
#define MOTOR_ARR           4249U
#define MOTOR_CCR_MIN       0U
#define MOTOR_CCR_MAX       MOTOR_ARR
#define MOTOR_CCR_DEFAULT   2550U       /* ~60% pour les tests */
#define MOTOR_DECALAGE_MAX  MOTOR_ARR

/* Initialisation : démarre les 4 PWM et enregistre la commande shell SET_CCR */
int motor_init(void);

/* Commande shell SET_CCR <ccr> [decalage] */
int sh_motor_set_ccr(h_shell_t *h_shell, int argc, char **argv);

#endif /* INC_MOTOR_CONTROL_MOTOR_H_ */
