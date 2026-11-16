/*
 * input_encoder.c
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#include "acquisition/input_encoder.h"
#include <stdio.h>

static volatile int16_t last_delta = 0;
static volatile int32_t motor_speed_rpm = 0;

int input_encoder_init(void)
{
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
        return HAL_ERROR;
    }

    __HAL_TIM_SET_COUNTER(&htim3, ENCODER_MIDPOINT);

    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
        return HAL_ERROR;
    }

    if (shell_add(&hshell1, "vitesse", sh_encoder_speed, "Affiche la vitesse codeur (RPM)") != 0) {
        return HAL_ERROR;
    }

    return shell_add(&hshell1, "enc", sh_encoder_speed, "Affiche la vitesse codeur (RPM)");
}

void input_encoder_update(void)
{
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    /* Reset au point milieu (32768) pour eviter tout debordement sur 10 ms */
    __HAL_TIM_SET_COUNTER(&htim3, ENCODER_MIDPOINT);

    int16_t delta = (int16_t)((int32_t)cnt - (int32_t)ENCODER_MIDPOINT);
    last_delta = delta;
    motor_speed_rpm = ((int32_t)delta * SPEED_SAMPLE_FREQ_HZ * SECONDS_PER_MINUTE) / ENCODER_TICKS_PER_REV;
}

int32_t get_motor_speed_rpm(void)
{
    return motor_speed_rpm;
}

int16_t get_encoder_delta(void)
{
    return last_delta;
}

int sh_encoder_speed(h_shell_t *h_shell, int argc, char **argv)
{
    int32_t rpm = motor_speed_rpm;
    int16_t delta = last_delta;
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    int size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "V=%ld tr/min | delta=%d | CNT=%u\r\n",
                        (long)rpm, (int)delta, (unsigned int)cnt);
    h_shell->drv.transmit(h_shell->print_buffer, size);

    return HAL_OK;
}
