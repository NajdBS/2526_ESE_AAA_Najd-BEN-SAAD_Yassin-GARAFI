/*
 * motor.c
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#include "motor_control/motor.h"
#include <stdlib.h>
#include <string.h>

/* Etat du moteur */
static uint8_t motor_running       = 0;
static uint32_t motor_ccr_target   = MOTOR_CCR_DEFAULT;
static uint32_t motor_ccr_current  = MOTOR_CCR_DEFAULT;
static uint32_t last_update_tick   = 0;

/* applique la commande complementaire decalee aux 4 transistors :
 * Bras U (CH1/CH1N) : CCR1 = motor_ccr_current
 * Bras V (CH2/CH2N) : CCR2 = MOTOR_ARR - motor_ccr_current */
static void motor_apply(void)
{
    uint32_t ccr_u = motor_ccr_current;
    uint32_t ccr_v = MOTOR_ARR - motor_ccr_current;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_v);
}

int motor_init(void)
{
    motor_running = 0;
    motor_ccr_current = MOTOR_CCR_DEFAULT;
    motor_ccr_target = MOTOR_CCR_DEFAULT;
    motor_apply();

    /* enregistrement des commandes shell */
    shell_add(&hshell1, "start", sh_motor_start, "Demarre les PWM a 50%");
    shell_add(&hshell1, "stop", sh_motor_stop, "Arrete les PWM");
    shell_add(&hshell1, "speed", sh_motor_speed, "Consigne vitesse : speed <valeur ou %>");
    shell_add(&hshell1, "SET_CCR", sh_motor_speed, "Consigne vitesse : SET_CCR <valeur ou %>");

    return 0;
}

/* Commande start : active les 4 PWM a 50% */
int sh_motor_start(h_shell_t *h_shell, int argc, char **argv)
{
    motor_ccr_current = MOTOR_CCR_DEFAULT;
    motor_ccr_target = MOTOR_CCR_DEFAULT;
    motor_apply();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

    motor_running = 1;
    last_update_tick = HAL_GetTick();

    int size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "OK: PWM demarrees a 50%% (CCR=%u, vitesse nulle)\r\n",
                        MOTOR_CCR_DEFAULT);
    h_shell->drv.transmit(h_shell->print_buffer, size);
    return HAL_OK;
}

/* Commande stop : desactive les 4 PWM */
int sh_motor_stop(h_shell_t *h_shell, int argc, char **argv)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

    motor_running = 0;
    motor_ccr_current = MOTOR_CCR_DEFAULT;
    motor_ccr_target = MOTOR_CCR_DEFAULT;
    motor_apply();

    int size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "OK: PWM arretees (moteur desactive)\r\n");
    h_shell->drv.transmit(h_shell->print_buffer, size);
    return HAL_OK;
}

/* Commande speed / SET_CCR : regle la cible (en ticks ou en %) et lance la rampe */
int sh_motor_speed(h_shell_t *h_shell, int argc, char **argv)
{
    int size;

    if (argc != 2) {
        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Usage: speed <valeur (0-%lu) ou %% (0-100%%)>\r\n",
                        (unsigned long)MOTOR_CCR_MAX);
        h_shell->drv.transmit(h_shell->print_buffer, size);
        return HAL_ERROR;
    }

    int is_pct = (strchr(argv[1], '%') != NULL);
    int raw_val = atoi(argv[1]);

    if (is_pct) {
        /* Interprete en pourcentage : necessite obligatoirement le signe % */
        if (raw_val < 0 || raw_val > 100) {
            size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                            "ERREUR: pourcentage %d%% hors limites (0-100%%) !\r\n",
                            raw_val);
            h_shell->drv.transmit(h_shell->print_buffer, size);
            return HAL_ERROR;
        }
        motor_ccr_target = ((uint32_t)raw_val * MOTOR_ARR) / 100U;
    } else {
        /* Pas de signe % : valeur brute directe en ticks (0 a 4249) */
        if (raw_val < 0 || raw_val > (int)MOTOR_CCR_MAX) {
            size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                            "ERREUR: valeur %d hors limites (0-%lu) !\r\n",
                            raw_val,
                            (unsigned long)MOTOR_CCR_MAX);
            h_shell->drv.transmit(h_shell->print_buffer, size);
            return HAL_ERROR;
        }
        motor_ccr_target = (uint32_t)raw_val;
    }

    uint32_t target_pct = (motor_ccr_target * 100U) / MOTOR_ARR;
    uint32_t current_pct = (motor_ccr_current * 100U) / MOTOR_ARR;

    if (!motor_running) {
        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Cible=%lu (%lu%%), tapez 'start' pour activer les PWM\r\n",
                        (unsigned long)motor_ccr_target,
                        (unsigned long)target_pct);
        h_shell->drv.transmit(h_shell->print_buffer, size);
    } else {
        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "OK: Cible=%lu (%lu%%) [actuel=%lu (%lu%%)], rampe...\r\n",
                        (unsigned long)motor_ccr_target,
                        (unsigned long)target_pct,
                        (unsigned long)motor_ccr_current,
                        (unsigned long)current_pct);
        h_shell->drv.transmit(h_shell->print_buffer, size);
    }

    return HAL_OK;
}

/* Gestion de la rampe progressive d'acceleration/deceleration */
void motor_update(void)
{
    if (!motor_running) return;

    uint32_t now = HAL_GetTick();
    if ((now - last_update_tick) < MOTOR_RAMP_DELAY_MS) return;
    last_update_tick = now;

    if (motor_ccr_current < motor_ccr_target) {
        if ((motor_ccr_target - motor_ccr_current) > MOTOR_RAMP_STEP) {
            motor_ccr_current += MOTOR_RAMP_STEP;
        } else {
            motor_ccr_current = motor_ccr_target;
        }
        motor_apply();
    } else if (motor_ccr_current > motor_ccr_target) {
        if ((motor_ccr_current - motor_ccr_target) > MOTOR_RAMP_STEP) {
            motor_ccr_current -= MOTOR_RAMP_STEP;
        } else {
            motor_ccr_current = motor_ccr_target;
        }
        motor_apply();
    }
}
