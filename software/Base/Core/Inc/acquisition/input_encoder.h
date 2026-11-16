/*
 * input_encoder.h
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#ifndef INC_ACQUISITION_INPUT_ENCODER_H_
#define INC_ACQUISITION_INPUT_ENCODER_H_

#include "stm32g4xx_hal.h"
#include "tim.h"
#include "user_interface/shell.h"

/* Codeur sur TIM3 (mode quadrature TI12 = x4) et echantillonnage TIM7 a 100 Hz (10 ms) */
#define ENCODER_PPR           1024L
#define ENCODER_QUAD_FACTOR   4L
#define ENCODER_TICKS_PER_REV (ENCODER_PPR * ENCODER_QUAD_FACTOR)
#define ENCODER_MIDPOINT      32768U  /* Point milieu du compteur 16 bits (65536 / 2) */
#define SPEED_SAMPLE_FREQ_HZ  100L
#define SECONDS_PER_MINUTE    60L

/* Demarre TIM3 en mode codeur, TIM7 en IT (10 ms) et enregistre les commandes shell */
int input_encoder_init(void);

/* Mise a jour de la vitesse (appelee toutes les 10 ms dans l'IT de TIM7) */
void input_encoder_update(void);

/* Accesseurs de vitesse et de variation de ticks */
int32_t get_motor_speed_rpm(void);
int16_t get_encoder_delta(void);

/* Commande shell : vitesse / enc */
int sh_encoder_speed(h_shell_t *h_shell, int argc, char **argv);

#endif /* INC_ACQUISITION_INPUT_ENCODER_H_ */
