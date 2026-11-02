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

/* TIM1 centre-aligne : f_PWM = 170 MHz / (2 * 4250) = 20 kHz
 * ARR = 4249, dead-time = 34 (~200 ns) */
#define MOTOR_ARR           4249U
#define MOTOR_CCR_MIN       0U
#define MOTOR_CCR_MAX       MOTOR_ARR
#define MOTOR_CCR_DEFAULT   2125U       /* 50% = arret */

/* Parametres de la rampe d'acceleration :
 * +10 ticks toutes les 10 ms => ~2 secondes pour parcourir 50% -> 100% */
#define MOTOR_RAMP_STEP     10U
#define MOTOR_RAMP_DELAY_MS 10U

/* Initialisation du module moteur */
int motor_init(void);

/* Fonction a appeler dans la fonction loop() de app.c pour gerer la rampe */
void motor_update(void);

/* Commandes shell */
int sh_motor_start(h_shell_t *h_shell, int argc, char **argv);
int sh_motor_stop(h_shell_t *h_shell, int argc, char **argv);
int sh_motor_speed(h_shell_t *h_shell, int argc, char **argv);

#endif /* INC_MOTOR_CONTROL_MOTOR_H_ */
