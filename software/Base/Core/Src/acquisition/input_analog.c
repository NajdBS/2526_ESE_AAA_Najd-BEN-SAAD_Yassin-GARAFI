/*
 * analog_input.c
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#include "acquisition/input_analog.h"
#include "adc.h"
#include "tim.h"
#include "user_interface/shell.h"

typedef struct {
    const char *name;
    uint32_t adc_channel;
    uint32_t zero_raw;      /* valeur brute a courant nul */
} analog_sensor_t;

/* Le moteur est entre les bras U et V : I_moteur = I_U = -I_V
 * I_V et I_Bus servent a verifier I_U. Le bras W ne porte aucun courant */
static analog_sensor_t analog_sensors[] = {
    { "I_U",   ADC_CHANNEL_2,  0 },     /* PA1 (CN7-30) : U_Imes */
    { "I_V",   ADC_CHANNEL_12, 0 },     /* PB1 (CN10-24) : V_Imes */
    { "I_Bus", ADC_CHANNEL_8,  0 },     /* PC2 (CN7-35) : Bus_Imes */
};

#define ANALOG_SENSOR_COUNT (sizeof(analog_sensors) / sizeof(analog_sensors[0]))

/* conversion brut -> mA, depuis la fonction de transfert du capteur */
static long analog_raw_to_milliamps(uint32_t raw, uint32_t zero_raw)
{
    float delta_raw = (float)raw - (float)zero_raw;
    float current_a = delta_raw * ANALOG_ADC_VREF_V / ANALOG_ADC_MAX / ANALOG_SENSITIVITY_V_PER_A;

    return (long)(current_a * ANALOG_MILLI_PER_UNIT);
}

/* mesure en polling : le CPU attend chaque fin de conversion */
static int analog_read_polling(uint32_t adc_channel, uint32_t *raw)
{
    ADC_ChannelConfTypeDef channel = {0};
    uint32_t sum = 0;

    channel.Channel = adc_channel;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ANALOG_SAMPLING_TIME;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) return HAL_ERROR;

    for (uint32_t i = 0; i < ANALOG_POLLING_SAMPLES; i++) {
        if (HAL_ADC_Start(&hadc1) != HAL_OK) return HAL_ERROR;

        if (HAL_ADC_PollForConversion(&hadc1, ANALOG_POLLING_TIMEOUT_MS) != HAL_OK) {
            HAL_ADC_Stop(&hadc1);
            return HAL_ERROR;
        }
        sum += HAL_ADC_GetValue(&hadc1);
    }
    *raw = sum / ANALOG_POLLING_SAMPLES;

    return HAL_ADC_Stop(&hadc1);
}

/* mesure le zero de tous les capteurs : a appeler a courant nul */
static int analog_measure_zero(void)
{
    for (uint32_t i = 0; i < ANALOG_SENSOR_COUNT; i++) {
        if (analog_read_polling(analog_sensors[i].adc_channel, &analog_sensors[i].zero_raw) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

/* envoie une ligne de texte sur le shell */
static void analog_print(h_shell_t *h_shell, const char *message)
{
    int size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE, "%s\r\n", message);

    h_shell->drv.transmit(h_shell->print_buffer, size);
}

/* current : mesure et affiche les courants */
static int sh_current(h_shell_t *h_shell, int argc, char **argv)
{
    for (uint32_t i = 0; i < ANALOG_SENSOR_COUNT; i++) {
        uint32_t raw;
        int size;

        if (analog_read_polling(analog_sensors[i].adc_channel, &raw) != HAL_OK) {
            analog_print(h_shell, "Erreur: mesure ADC");
            return HAL_ERROR;
        }

        size = snprintf(h_shell->print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "%s=%ld mA (brut=%lu, zero=%lu)\r\n",
                        analog_sensors[i].name,
                        analog_raw_to_milliamps(raw, analog_sensors[i].zero_raw),
                        (unsigned long)raw,
                        (unsigned long)analog_sensors[i].zero_raw);
        h_shell->drv.transmit(h_shell->print_buffer, size);
    }

    return HAL_OK;
}

/* current_zero : remesure le zero des capteurs, PWM coupees */
static int sh_current_zero(h_shell_t *h_shell, int argc, char **argv)
{
    if ((htim1.Instance->CR1 & TIM_CR1_CEN) != 0U) {
        analog_print(h_shell, "Erreur: taper stop et attendre l'arret du moteur");
        return HAL_ERROR;
    }

    if (analog_measure_zero() != HAL_OK) {
        analog_print(h_shell, "Erreur: mesure ADC");
        return HAL_ERROR;
    }

    analog_print(h_shell, "OK: zero mesure");
    return HAL_OK;
}

int input_analog_init(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) return HAL_ERROR;

    /* les PWM sont coupees au demarrage : le courant est nul */
    if (analog_measure_zero() != HAL_OK) return HAL_ERROR;

    if (shell_add(&hshell1, "current", sh_current, "Courants en mA (polling)") != 0)                  return HAL_ERROR;
    if (shell_add(&hshell1, "current_zero", sh_current_zero, "Remesure le zero, moteur arrete") != 0) return HAL_ERROR;

    return HAL_OK;
}
