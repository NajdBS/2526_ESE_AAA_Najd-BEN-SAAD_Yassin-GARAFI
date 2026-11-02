/*
 * motor.c
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#include "motor_control/motor.h"
#include <stdlib.h>

/* valeur courante du CCR bras U */
static uint32_t motor_ccr = MOTOR_CCR_DEFAULT;

/* applique la commande complémentaire décalée aux 4 transistors :
 * Bras U (CH1/CH1N) : CCR1 = motor_ccr
 * Bras V (CH2/CH2N) : CCR2 = MOTOR_ARR - motor_ccr */
static void motor_apply(void)
{
    uint32_t ccr_u = motor_ccr;
    uint32_t ccr_v = MOTOR_ARR - motor_ccr;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_u);
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
                     "Commande vitesse : SET_CCR <valeur>");
}

/* SET_CCR <valeur> (de 0 a MOTOR_ARR = 4249)
 * 2125 = 50% = arret
 * > 2125 = avance
 * < 2125 = recule */
int sh_motor_set_ccr(h_shell_t *h_shell, int argc, char **argv)
{
    int size;

    if (argc != 2) {
        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Usage: SET_CCR <valeur (0-%lu)>\r\n",
                        (unsigned long)MOTOR_CCR_MAX);
        h_shell->drv.transmit(h_shell->print_buffer, size);
        return HAL_ERROR;
    }

    /* conversion en entier */
    int raw_ccr = atoi(argv[1]);

    /* verification et saturation au maximum autorise */
    if (raw_ccr < (int)MOTOR_CCR_MIN) raw_ccr = (int)MOTOR_CCR_MIN;
    if (raw_ccr > (int)MOTOR_CCR_MAX) raw_ccr = (int)MOTOR_CCR_MAX;
    motor_ccr = (uint32_t)raw_ccr;

    /* application aux 4 transistors */
    motor_apply();

    uint32_t ccr_v = MOTOR_ARR - motor_ccr;
    size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                    "OK: CCR1=%lu  CCR2=%lu (max=%lu)\r\n",
                    (unsigned long)motor_ccr,
                    (unsigned long)ccr_v,
                    (unsigned long)MOTOR_CCR_MAX);
    h_shell->drv.transmit(h_shell->print_buffer, size);

    return HAL_OK;
}
