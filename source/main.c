#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "fsl_pwm.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "hbridge.h"
#include "pixy.h"
#include "fsl_common.h"
#include "Config.h"
#include "servo.h"
#include "esc.h"
#include "fsl_lpi2c.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include <string.h>

static int limiteaza(int valoare, int minim, int maxim)
{
    if (valoare < minim) return minim;
    if (valoare > maxim) return maxim;
    return valoare;
}

static uint8_t punct_jos_x(const pixy_vector_t *vector)
{
    if (vector->y1 >= vector->y0)
        return vector->x1;
    else
        return vector->x0;
}

static uint8_t punct_jos_y(const pixy_vector_t *vector)
{
    if (vector->y1 >= vector->y0)
        return vector->y1;
    else
        return vector->y0;
}

static int lungime_vector(const pixy_vector_t *vector)
{
    int dx = (int)vector->x1 - (int)vector->x0;
    int dy = (int)vector->y1 - (int)vector->y0;
    return dx * dx + dy * dy;
}

static int este_in_imagine(const pixy_vector_t *vector)
{
    if (vector->x0 > 79 || vector->x1 > 79) return 0;
    if (vector->y0 > 52 || vector->y1 > 52) return 0;
    return 1;
}

static int este_destul_de_lung(const pixy_vector_t *vector)
{
    return (lungime_vector(vector) >= 25);
}

static int este_destul_de_jos(const pixy_vector_t *vector)
{
    return (punct_jos_y(vector) >= 20);
}

static int vector_valid(const pixy_vector_t *vector)
{
    if (!este_in_imagine(vector)) return 0;
    if (!este_destul_de_lung(vector)) return 0;
    if (!este_destul_de_jos(vector)) return 0;
    return 1;
}

static int vector_stanga(const pixy_vector_t *vector)
{
    int dx = (int)vector->x1 - (int)vector->x0;
    int dy = (int)vector->y1 - (int)vector->y0;

    if ((dx * dy) < 0)
        return 1;

    return 0;
}

static int vector_dreapta(const pixy_vector_t *vector)
{
    int dx = (int)vector->x1 - (int)vector->x0;
    int dy = (int)vector->y1 - (int)vector->y0;

    if ((dx * dy) > 0)
        return 1;

    return 0;
}

static int selectstanga(const pixy_vector_t *vectori, size_t nr, pixy_vector_t *rezultat)
{
    int gasit = 0;
    uint8_t cel_mai_jos = 0;
    int cea_mai_mare_lungime = 0;

    for (size_t i = 0; i < nr; i++)
    {
        if (!vector_valid(&vectori[i]))
            continue;

        if (!vector_stanga(&vectori[i]))
            continue;

        uint8_t jos = punct_jos_y(&vectori[i]);
        int lungime = lungime_vector(&vectori[i]);

        if (!gasit)
        {
            *rezultat = vectori[i];
            cel_mai_jos = jos;
            cea_mai_mare_lungime = lungime;
            gasit = 1;
        }
        else
        {
            if (jos > cel_mai_jos)
            {
                *rezultat = vectori[i];
                cel_mai_jos = jos;
                cea_mai_mare_lungime = lungime;
            }
            else if (jos == cel_mai_jos && lungime > cea_mai_mare_lungime)
            {
                *rezultat = vectori[i];
                cea_mai_mare_lungime = lungime;
            }
        }
    }

    return gasit;
}

static int selectdreapta(const pixy_vector_t *vectori, size_t nr, pixy_vector_t *rezultat)
{
    int gasit = 0;
    uint8_t cel_mai_jos = 0;
    int cea_mai_mare_lungime = 0;

    for (size_t i = 0; i < nr; i++)
    {
        if (!vector_valid(&vectori[i]))
            continue;

        if (!vector_dreapta(&vectori[i]))
            continue;

        uint8_t jos = punct_jos_y(&vectori[i]);
        int lungime = lungime_vector(&vectori[i]);

        if (!gasit)
        {
            *rezultat = vectori[i];
            cel_mai_jos = jos;
            cea_mai_mare_lungime = lungime;
            gasit = 1;
        }
        else
        {
            if (jos > cel_mai_jos)
            {
                *rezultat = vectori[i];
                cel_mai_jos = jos;
                cea_mai_mare_lungime = lungime;
            }
            else if (jos == cel_mai_jos && lungime > cea_mai_mare_lungime)
            {
                *rezultat = vectori[i];
                cea_mai_mare_lungime = lungime;
            }
        }
    }

    return gasit;
}

static int calculeaza_x_la_y(const pixy_vector_t *vector, int y_virtual, int *x_rezultat)
{
    int x0 = (int)vector->x0;
    int y0 = (int)vector->y0;
    int x1 = (int)vector->x1;
    int y1 = (int)vector->y1;

    int y_min;
    int y_max;

    if (y0 < y1)
    {
        y_min = y0;
        y_max = y1;
    }
    else
    {
        y_min = y1;
        y_max = y0;
    }

    if (y_virtual < y_min || y_virtual > y_max)
        return 0;

    if (y1 == y0)
        return 0;

    *x_rezultat = x0 + (y_virtual - y0) * (x1 - x0) / (y1 - y0);
    return 1;
}

int main(void)
{
    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    pixy_vector_t v[10];
    size_t n = 0;

    pixy_t cam1;
    pixy_init(&cam1,
              LPI2C2,
              0x54,
              &LP_FLEXCOMM2_RX_Handle,
              &LP_FLEXCOMM2_TX_Handle);

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);

    const int centru_imagine = 36;
    const int y_virtual = 30;

    const int kp = 3;
    const int kd = 4;
    const int offset_servo = 0;   // modifica doar daca mecanic nu e centrat
    int eroare_anterioara = 0;

    Steer(offset_servo);

    while (1)
    {
        status_t status_vectori = pixy_get_vectors(&cam1, v, 10, &n);

        if (status_vectori == kStatus_Success)
        {
            pixy_vector_t stanga;
            pixy_vector_t dreapta;

            int are_stanga = selectstanga(v, n, &stanga);
            int are_dreapta = selectdreapta(v, n, &dreapta);

            if (are_stanga && are_dreapta)
            {
                int x_stanga = 0;
                int x_dreapta = 0;

                int ok_stanga = calculeaza_x_la_y(&stanga, y_virtual, &x_stanga);
                int ok_dreapta = calculeaza_x_la_y(&dreapta, y_virtual, &x_dreapta);

                if (ok_stanga && ok_dreapta)
                {
                    int centru_banda = (x_stanga + x_dreapta) / 2;

                    int eroare = centru_imagine - centru_banda;

                    int derivativa = eroare - eroare_anterioara;

                    eroare_anterioara = eroare;

                    int comanda_pd = kp * eroare + kd * derivativa;
                    int comanda_servo = offset_servo + comanda_pd;

                    comanda_servo = limiteaza(comanda_servo, -60, 60);

                    Steer(comanda_servo);

                    HbridgeSpeed(&g_hbridge, 80, -80);

                   //
                }
                else
                {
                   //
                }
            }
            else
            {
                //
            }
        }
        else
        {
           //
        }

        SDK_DelayAtLeastUs(8000, SystemCoreClock);
    }
}
