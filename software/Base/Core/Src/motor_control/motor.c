/*
 * motor.c
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#include "motor_control/motor.h"
#include <stdlib.h>

/* valeurs courantes */
static uint32_t motor_ccr      = MOTOR_CCR_DEFAULT;
static uint32_t motor_decalage = 0U;

/* applique CCR aux deux bras : U=CH1, V=CH2 avec décalage */
static void motor_apply(void)
{
    uint32_t ccr_v = motor_ccr + motor_decalage;
    if (ccr_v > MOTOR_CCR_MAX)
        ccr_v = MOTOR_CCR_MAX;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_ccr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_v);
}

int motor_init(void)
{
    /* démarrage des 4 PWM complémentaires bras U et V */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)    return HAL_ERROR;
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) return HAL_ERROR;
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)    return HAL_ERROR;
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) return HAL_ERROR;

    motor_apply();

    return shell_add(&hshell1, "SET_CCR", sh_motor_set_ccr,
                     "Commande vitesse : SET_CCR <ccr> [decalage]");
}

/* SET_CCR <ccr (0-4249)> [decalage]
 * ccr      : rapport cyclique bras U
 * decalage : décalage de phase bras V (optionnel, conservé si absent) */
int sh_motor_set_ccr(h_shell_t *h_shell, int argc, char **argv)
{
    int size;

    if (argc < 2 || argc > 3) {
        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Usage: SET_CCR <ccr (0-%lu)> [decalage]\r\n",
                        (unsigned long)MOTOR_CCR_MAX);
        h_shell->drv.transmit(h_shell->print_buffer, size);
        return HAL_ERROR;
    }

    /* lecture et saturation du CCR */
    int raw_ccr = atoi(argv[1]);
    if (raw_ccr < (int)MOTOR_CCR_MIN) raw_ccr = (int)MOTOR_CCR_MIN;
    if (raw_ccr > (int)MOTOR_CCR_MAX) raw_ccr = (int)MOTOR_CCR_MAX;
    motor_ccr = (uint32_t)raw_ccr;

    /* décalage optionnel */
    if (argc == 3) {
        int raw_dec = atoi(argv[2]);
        if (raw_dec < 0)                       raw_dec = 0;
        if (raw_dec > (int)MOTOR_DECALAGE_MAX) raw_dec = (int)MOTOR_DECALAGE_MAX;
        motor_decalage = (uint32_t)raw_dec;
    }

    motor_apply();

    uint32_t ccr_v = motor_ccr + motor_decalage;
    if (ccr_v > MOTOR_CCR_MAX) ccr_v = MOTOR_CCR_MAX;

    size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                    "OK: CCR_U=%lu  CCR_V=%lu  decalage=%lu\r\n",
                    (unsigned long)motor_ccr,
                    (unsigned long)ccr_v,
                    (unsigned long)motor_decalage);
    h_shell->drv.transmit(h_shell->print_buffer, size);

    return HAL_OK;
}
