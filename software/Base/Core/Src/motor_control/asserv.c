/*
 * asserv.c
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#include "motor_control/asserv.h"
#include "motor_control/motor.h"
#include "acquisition/input_analog.h"
#include "acquisition/input_encoder.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static pid_controller_t pid_speed;
static pid_controller_t pid_current;

static volatile asserv_mode_t asserv_mode = ASSERV_MODE_OFF;
static volatile float asserv_setpoint = 0.0f;

static float clampf(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static float pid_compute(pid_controller_t *pid, float setpoint, float measure, float dt)
{
    float error = setpoint - measure;
    float p_term = pid->kp * error;
    float d_term = pid->kd * ((error - pid->prev_error) / dt);
    pid->prev_error = error;

    /* Anti-windup : blocage de l'integrale en cas de saturation dans le meme sens */
    float new_integral = pid->integral + (pid->ki * error * dt);
    float raw_out = p_term + new_integral + d_term;

    if ((raw_out >= pid->out_min && raw_out <= pid->out_max) ||
        (raw_out > pid->out_max && error < 0.0f) ||
        (raw_out < pid->out_min && error > 0.0f)) {
        pid->integral = clampf(new_integral, pid->out_min, pid->out_max);
    }

    return clampf(p_term + pid->integral + d_term, pid->out_min, pid->out_max);
}

static const char *asserv_mode_str(asserv_mode_t mode)
{
    switch (mode) {
    case ASSERV_MODE_SPEED:   return "speed";
    case ASSERV_MODE_CURRENT: return "current";
    case ASSERV_MODE_CASCADE: return "cascade";
    default:                  return "off";
    }
}

static void asserv_set_mode(asserv_mode_t new_mode, float setpoint)
{
    asserv_mode = ASSERV_MODE_OFF;
    pid_reset(&pid_speed);
    pid_reset(&pid_current);

    if (new_mode == ASSERV_MODE_CASCADE) {
        pid_speed.out_min = -ASSERV_MAX_CURRENT_MA;
        pid_speed.out_max = ASSERV_MAX_CURRENT_MA;
    } else {
        pid_speed.out_min = -ASSERV_MAX_DELTA_CCR;
        pid_speed.out_max = ASSERV_MAX_DELTA_CCR;
    }

    asserv_setpoint = setpoint;
    asserv_mode = new_mode;
}

int asserv_init(void)
{
    pid_init(&pid_speed, SPEED_KP_DEFAULT, SPEED_KI_DEFAULT, SPEED_KD_DEFAULT,
             -ASSERV_MAX_DELTA_CCR, ASSERV_MAX_DELTA_CCR);
    pid_init(&pid_current, CURRENT_KP_DEFAULT, CURRENT_KI_DEFAULT, CURRENT_KD_DEFAULT,
             -ASSERV_MAX_DELTA_CCR, ASSERV_MAX_DELTA_CCR);
    asserv_mode = ASSERV_MODE_OFF;
    asserv_setpoint = 0.0f;

    if (shell_add(&hshell1, "asserv", sh_asserv,
                  "PID: asserv <off|speed|current|cascade> [val]") != 0) {
        return HAL_ERROR;
    }

    return shell_add(&hshell1, "pid", sh_pid_tune,
                     "Gains PID: pid <speed|current> <kp_m> <ki_m> <kd_m>");
}

void asserv_stop(void)
{
    asserv_mode = ASSERV_MODE_OFF;
    pid_reset(&pid_speed);
    pid_reset(&pid_current);
}

void asserv_update(void)
{
    if (asserv_mode == ASSERV_MODE_OFF || !motor_is_running()) {
        return;
    }

    float mes_rpm = (float)get_motor_speed_rpm();
    float mes_ma = (float)get_current_u_mA();
    float delta_ccr = 0.0f;

    switch (asserv_mode) {
    case ASSERV_MODE_SPEED:
        delta_ccr = pid_compute(&pid_speed, asserv_setpoint, mes_rpm, ASSERV_DT_S);
        break;

    case ASSERV_MODE_CURRENT:
        delta_ccr = pid_compute(&pid_current, asserv_setpoint, mes_ma, ASSERV_DT_S);
        break;

    case ASSERV_MODE_CASCADE: {
        float i_ref_ma = pid_compute(&pid_speed, asserv_setpoint, mes_rpm, ASSERV_DT_S);
        delta_ccr = pid_compute(&pid_current, i_ref_ma, mes_ma, ASSERV_DT_S);
        break;
    }

    default:
        return;
    }

    int32_t ccr = (int32_t)MOTOR_CCR_DEFAULT + (int32_t)delta_ccr;
    if (ccr < (int32_t)MOTOR_CCR_MIN) ccr = (int32_t)MOTOR_CCR_MIN;
    if (ccr > (int32_t)MOTOR_CCR_MAX) ccr = (int32_t)MOTOR_CCR_MAX;

    motor_set_ccr_direct((uint32_t)ccr);
}

int sh_asserv(h_shell_t *h_shell, int argc, char **argv)
{
    int size;

    if (argc == 1 || (argc == 2 && strcmp(argv[1], "status") == 0)) {
        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Mode=%s cons=%ld | CCR=%lu\r\n",
                        asserv_mode_str(asserv_mode),
                        (long)asserv_setpoint,
                        (unsigned long)motor_get_ccr());
        h_shell->drv.transmit(h_shell->print_buffer, size);

        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Mes: V=%ld tr/min | I=%ld mA\r\n",
                        (long)get_motor_speed_rpm(),
                        (long)get_current_u_mA());
        h_shell->drv.transmit(h_shell->print_buffer, size);
        return HAL_OK;
    }

    if (argc == 2 && strcmp(argv[1], "off") == 0) {
        asserv_stop();
        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "OK: Asservissement desactive\r\n");
        h_shell->drv.transmit(h_shell->print_buffer, size);
        return HAL_OK;
    }

    if (argc == 3) {
        asserv_mode_t new_mode;
        if (strcmp(argv[1], "speed") == 0) {
            new_mode = ASSERV_MODE_SPEED;
        } else if (strcmp(argv[1], "current") == 0) {
            new_mode = ASSERV_MODE_CURRENT;
        } else if (strcmp(argv[1], "cascade") == 0) {
            new_mode = ASSERV_MODE_CASCADE;
        } else {
            size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                            "Usage: asserv <off|speed|current|cascade> [val]\r\n");
            h_shell->drv.transmit(h_shell->print_buffer, size);
            return HAL_ERROR;
        }

        int32_t target = (int32_t)atoi(argv[2]);
        asserv_set_mode(new_mode, (float)target);

        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "OK: Mode=%s cible=%ld\r\n",
                        asserv_mode_str(new_mode), (long)target);
        h_shell->drv.transmit(h_shell->print_buffer, size);

        if (!motor_is_running()) {
            size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                            "Attention: tapez 'start' pour activer PWM\r\n");
            h_shell->drv.transmit(h_shell->print_buffer, size);
        }
        return HAL_OK;
    }

    size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                    "Usage: asserv <off|speed|current|cascade> [val]\r\n");
    h_shell->drv.transmit(h_shell->print_buffer, size);
    return HAL_ERROR;
}

int sh_pid_tune(h_shell_t *h_shell, int argc, char **argv)
{
    int size;

    if (argc != 5) {
        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Usage: pid <speed|current> <kp_m> <ki_m> <kd_m>\r\n");
        h_shell->drv.transmit(h_shell->print_buffer, size);
        return HAL_ERROR;
    }

    pid_controller_t *pid = NULL;
    if (strcmp(argv[1], "speed") == 0) {
        pid = &pid_speed;
    } else if (strcmp(argv[1], "current") == 0) {
        pid = &pid_current;
    } else {
        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Usage: pid <speed|current> <kp_m> <ki_m> <kd_m>\r\n");
        h_shell->drv.transmit(h_shell->print_buffer, size);
        return HAL_ERROR;
    }

    long kp_m = atol(argv[2]);
    long ki_m = atol(argv[3]);
    long kd_m = atol(argv[4]);

    pid->kp = (float)kp_m / PID_GAIN_SCALE;
    pid->ki = (float)ki_m / PID_GAIN_SCALE;
    pid->kd = (float)kd_m / PID_GAIN_SCALE;
    pid_reset(pid);

    size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                    "OK: PID %s mis a jour (milli):\r\n", argv[1]);
    h_shell->drv.transmit(h_shell->print_buffer, size);

    size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                    "Kp=%ld Ki=%ld Kd=%ld\r\n", kp_m, ki_m, kd_m);
    h_shell->drv.transmit(h_shell->print_buffer, size);

    return HAL_OK;
}
