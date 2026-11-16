/*
 * asserv.h
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#ifndef INC_MOTOR_CONTROL_ASSERV_H_
#define INC_MOTOR_CONTROL_ASSERV_H_

#include "stm32g4xx_hal.h"
#include "user_interface/shell.h"

/* Periode d'echantillonnage TIM7 (10 ms = 100 Hz) et limites de commande */
#define ASSERV_DT_S           0.01f
#define ASSERV_MAX_DELTA_CCR  2000.0f
#define ASSERV_MAX_CURRENT_MA 4000.0f
#define PID_GAIN_SCALE        1000.0f

/* Gains PID par defaut */
#define SPEED_KP_DEFAULT      1.5f
#define SPEED_KI_DEFAULT      0.5f
#define SPEED_KD_DEFAULT      0.0f

#define CURRENT_KP_DEFAULT    0.3f
#define CURRENT_KI_DEFAULT    2.0f
#define CURRENT_KD_DEFAULT    0.0f

typedef enum {
    ASSERV_MODE_OFF = 0,
    ASSERV_MODE_SPEED,
    ASSERV_MODE_CURRENT,
    ASSERV_MODE_CASCADE
} asserv_mode_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} pid_controller_t;

/* Initialisation du module d'asservissement et des commandes shell */
int asserv_init(void);

/* Calcul PID appele a 100 Hz dans l'interruption TIM7 */
void asserv_update(void);

/* Arret de l'asservissement et reinitialisation des correcteurs */
void asserv_stop(void);

/* Commandes shell : asserv et pid */
int sh_asserv(h_shell_t *h_shell, int argc, char **argv);
int sh_pid_tune(h_shell_t *h_shell, int argc, char **argv);

#endif /* INC_MOTOR_CONTROL_ASSERV_H_ */
