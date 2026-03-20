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
#include "oled.h"
#include "fsl_lpuart.h"

#define OLED_I2C   LPI2C3
#define OLED_ADDR  0x3C
#define CENTRU_IMAGINE       38
#define Y_NEAR               40    /* aproape de masina - pozitie laterala */
#define Y_FAR                20    /* mai departe       - anticipare curba */

#define KP_NEAR               3    /* castig eroare pozitie               */
#define KP_FAR                1    /* castig eroare heading (anticipare)  */
#define KD                    1    /* castig derivativa erorii totale     */
#define OFFSET_SERVO          10

#define LATIME_BANDA_DEFAULT 30    /* pixeli - de calibrat pe traseu      */
#define LATIME_MIN           10
#define LATIME_MAX           70
#define MAX_SALT_POZITIE     15    /* rejecta detectii cu salt brusc > 15 px */

/* Cadre pentru fallback */
#define CADRE_HOLD            3    /* mentinem ultima comanda buna        */
#define CADRE_REDUCE         10    /* reducem treptat spre centru         */
#define PAS_REDUCERE          5    /* grade / cadru la reducere           */

/* Filtru EMA (aritmetica intreaga): alpha = EMA_NUM / EMA_DEN */
#define EMA_NUM               4
#define EMA_DEN              10

/* Debug serial: pune 0 ca sa opresti printurile */
#define DEBUG_SERIAL          0

int i2c_ping(LPI2C_Type *base, uint8_t addr)
{
    lpi2c_master_transfer_t xfer = {0};
    uint8_t dummy = 0x00;

    xfer.slaveAddress = addr;
    xfer.direction = kLPI2C_Write;
    xfer.subaddress = 0;
    xfer.subaddressSize = 0;
    xfer.data = &dummy;
    xfer.dataSize = 1;
    xfer.flags = kLPI2C_TransferDefaultFlag;

    return (LPI2C_MasterTransferBlocking(base, &xfer) == kStatus_Success) ? 1 : 0;
}

static void i2c_scan(LPI2C_Type *base)
{
    for (uint8_t a = 1; a < 0x7F; a++)
    {
        if (i2c_ping(base, a))
            PRINTF("I2C device @ 0x%02X\r\n", a);
    }
}


static status_t oled_i2c_write(uint8_t addr7, const uint8_t *data, size_t len)
{
    lpi2c_master_transfer_t t = {0};
    t.slaveAddress = addr7;
    t.direction    = kLPI2C_Write;
    t.data         = (uint8_t*)data;
    t.dataSize     = len;
    return LPI2C_MasterTransferBlocking(OLED_I2C, &t);
}

typedef struct __attribute__((packed)) {
    uint8_t  sync;      // 0xA5
    int16_t  yb;        // Y_NEAR
    int16_t  yb_lx;     // dbg_x_st_near
    int16_t  yb_rx;     // dbg_x_dr_near
    int16_t  yu;        // Y_FAR
    int16_t  yu_lx;     // dbg_x_st_far
    int16_t  yu_rx;     // dbg_x_dr_far
    int16_t  servo;     // comanda_servo
    uint8_t  chk;       // checksum
} telemetry_t;

static telemetry_t txData;

static int limiteaza(int valoare, int minim, int maxim)
{
    if (valoare < minim) return minim;
    if (valoare > maxim) return maxim;
    return valoare;
}

static int ema_update(int vechi, int nou)
{
    return (EMA_NUM * nou + (EMA_DEN - EMA_NUM) * vechi) / EMA_DEN;
}

/* ---------------------------------------------------------------
 * Validare vectori
 * --------------------------------------------------------------- */
static uint8_t punct_jos_y(const pixy_vector_t *v)
{
    return (v->y1 >= v->y0) ? v->y1 : v->y0;
}

static int lungime_vector_sq(const pixy_vector_t *v)
{
    int dx = (int)v->x1 - (int)v->x0;
    int dy = (int)v->y1 - (int)v->y0;
    return dx * dx + dy * dy;
}

static int este_in_imagine(const pixy_vector_t *v)
{
    return (v->x0 <= 79 && v->x1 <= 79 && v->y0 <= 52 && v->y1 <= 52);
}

static int vector_valid(const pixy_vector_t *v)
{
    if (!este_in_imagine(v))        return 0;
    if (lungime_vector_sq(v) < 25)  return 0;  /* prea scurt */
    if (punct_jos_y(v) < 20)        return 0;  /* prea sus   */
    return 1;
}

/* Interpolare liniara: calculeaza x la y_virtual pe segmentul vectorului.
 * Returneaza 1 daca y_virtual e in intervalul vectorului, 0 altfel.
 */
static int calculeaza_x_la_y(const pixy_vector_t *v, int y_virtual, int *x_out)
{
    int x0 = (int)v->x0, y0 = (int)v->y0;
    int x1 = (int)v->x1, y1 = (int)v->y1;

    if (y1 == y0) return 0;

    int y_min = (y0 < y1) ? y0 : y1;
    int y_max = (y0 < y1) ? y1 : y0;

    if (y_virtual < y_min || y_virtual > y_max) return 0;

    *x_out = x0 + (y_virtual - y0) * (x1 - x0) / (y1 - y0);
    return 1;
}

/* ---------------------------------------------------------------
 * Scor pentru selectia vectorului cel mai bun
 * --------------------------------------------------------------- */
static int scor_vector(const pixy_vector_t *v, int x_curent,
                       int are_istoric, int x_anterior)
{
    int scor = 4 * (int)punct_jos_y(v) + lungime_vector_sq(v);

    if (are_istoric)
    {
        int dist = x_curent - x_anterior;
        if (dist < 0) dist = -dist;
        scor -= 8 * dist;
    }
    return scor;
}

/* ---------------------------------------------------------------
 * Selectie vector stanga / dreapta la un y_virtual dat
 * vrem_stanga: 1 = cautam stanga (x < centru), 0 = dreapta (x > centru)
 * --------------------------------------------------------------- */
static int selecteaza_candidat(const pixy_vector_t *vectori, size_t nr,
                                int y_virtual, int centru,
                                int vrem_stanga,
                                int are_istoric, int x_anterior,
                                int *x_selectat)
{
    int gasit    = 0;
    int scor_max = -1000000;

    for (size_t i = 0; i < nr; i++)
    {
        int x_cur = 0;
        if (!vector_valid(&vectori[i]))                          continue;
        if (!calculeaza_x_la_y(&vectori[i], y_virtual, &x_cur)) continue;

        int conditie = vrem_stanga ? (x_cur < centru) : (x_cur > centru);
        if (!conditie) continue;

        /* Verificare plausibilitate: rejectam salturi bruste */
        if (are_istoric)
        {
            int salt = x_cur - x_anterior;
            if (salt < 0) salt = -salt;
            if (salt > MAX_SALT_POZITIE) continue;
        }

        int scor = scor_vector(&vectori[i], x_cur, are_istoric, x_anterior);
        if (!gasit || scor > scor_max)
        {
            *x_selectat = x_cur;
            scor_max    = scor;
            gasit       = 1;
        }
    }
    return gasit;
}


int main(void)
{
    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();
    //LPUART_EnableTx(LPUART2,true);

    pixy_vector_t v[10];
    size_t n = 0;

    pixy_t cam1;
    pixy_init(&cam1,
              LPI2C3, 0x54,
              &LP_FLEXCOMM3_RX_Handle,
              &LP_FLEXCOMM3_TX_Handle);

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);

    /* --- Stare memorie --- */
    int ultim_x_stanga       = -1;  /* -1 = nu avem inca istoric valid */
    int ultim_x_dreapta      = -1;
    int latime_banda         = LATIME_BANDA_DEFAULT;

    int eroare_anterioara    = 0;
    int cadre_fara_detectie  = 0;
    int ultima_comanda_servo = OFFSET_SERVO;

    Steer(OFFSET_SERVO);

    while (1)
    {
        status_t status_vectori = pixy_get_vectors(&cam1, v, 10, &n);

        int avem_centru_valid = 0;
        int avem_centru_far   = 0;
        int centru_near       = 0;
        int centru_far        = 0;

        /* variabile debug — initializate cu -1 = "negasit" */
        int dbg_x_st_near = -1;
        int dbg_x_dr_near = -1;
        int dbg_x_st_far  = -1;
        int dbg_x_dr_far  = -1;

        if (status_vectori == kStatus_Success)
        {
            int x_stanga  = 0;
            int x_dreapta = 0;
            int x_st_far  = 0;
            int x_dr_far  = 0;

            /* ----- Punct NEAR ----- */
            int are_stanga = selecteaza_candidat(v, n, Y_NEAR, CENTRU_IMAGINE,
                                                 1,
                                                 (ultim_x_stanga  >= 0),
                                                 ultim_x_stanga,
                                                 &x_stanga);

            int are_dreapta = selecteaza_candidat(v, n, Y_NEAR, CENTRU_IMAGINE,
                                                  0,
                                                  (ultim_x_dreapta >= 0),
                                                  ultim_x_dreapta,
                                                  &x_dreapta);

            /* ----- Punct FAR ----- */
            int are_st_far = selecteaza_candidat(v, n, Y_FAR, CENTRU_IMAGINE,
                                                 1,
                                                 (ultim_x_stanga  >= 0),
                                                 ultim_x_stanga,
                                                 &x_st_far);

            int are_dr_far = selecteaza_candidat(v, n, Y_FAR, CENTRU_IMAGINE,
                                                 0,
                                                 (ultim_x_dreapta >= 0),
                                                 ultim_x_dreapta,
                                                 &x_dr_far);

            /* salvam pentru debug */
            if (are_stanga)  dbg_x_st_near = x_stanga;
            if (are_dreapta) dbg_x_dr_near = x_dreapta;
            if (are_st_far)  dbg_x_st_far  = x_st_far;
            if (are_dr_far)  dbg_x_dr_far  = x_dr_far;

            /* --- centru NEAR --- */
            if (are_stanga && are_dreapta)
            {
                int latime_curenta = x_dreapta - x_stanga;
                if (latime_curenta > LATIME_MIN && latime_curenta < LATIME_MAX)
                {
                    centru_near       = (x_stanga + x_dreapta) / 2;
                    avem_centru_valid = 1;

                    latime_banda    = ema_update(latime_banda,    latime_curenta);
                    ultim_x_stanga  = ema_update(ultim_x_stanga,  x_stanga);
                    ultim_x_dreapta = ema_update(ultim_x_dreapta, x_dreapta);
                }
            }
            else if (are_stanga)
            {
                centru_near       = x_stanga + latime_banda / 2;
                avem_centru_valid = 1;
                ultim_x_stanga    = (ultim_x_stanga < 0) ? x_stanga
                                    : ema_update(ultim_x_stanga, x_stanga);
            }
            else if (are_dreapta)
            {
                centru_near       = x_dreapta - latime_banda / 2;
                avem_centru_valid = 1;
                ultim_x_dreapta   = (ultim_x_dreapta < 0) ? x_dreapta
                                    : ema_update(ultim_x_dreapta, x_dreapta);
            }

            /* --- centru FAR (doar cu ambele margini) --- */
            if (are_st_far && are_dr_far)
            {
                int latime_far = x_dr_far - x_st_far;
                if (latime_far > LATIME_MIN && latime_far < LATIME_MAX)
                {
                    centru_far      = (x_st_far + x_dr_far) / 2;
                    avem_centru_far = 1;
                }
            }
        }

        /* --- control servo --- */
        int err_near      = 0;
        int err_far       = 0;
        int comanda_servo = ultima_comanda_servo;

        if (avem_centru_valid)
        {
            err_near = CENTRU_IMAGINE - centru_near;
            err_far  = avem_centru_far ? (CENTRU_IMAGINE - centru_far) : 0;

            int eroare_totala = KP_NEAR * err_near + KP_FAR * err_far;
            int derivativa    = eroare_totala - eroare_anterioara;
            eroare_anterioara = eroare_totala;

            comanda_servo = OFFSET_SERVO + eroare_totala + KD * derivativa;
            comanda_servo = limiteaza(comanda_servo, -60, 60);

            Steer(comanda_servo);
            ultima_comanda_servo = comanda_servo;
            cadre_fara_detectie  = 0;
        }
        else
        {
            /*
             * Fallback in trei faze:
             *  Faza 1 [0 .. CADRE_HOLD]      – mentinem ultima comanda buna
             *  Faza 2 [CADRE_HOLD .. CADRE_REDUCE] – reducem treptat spre centru
             *  Faza 3 [> CADRE_REDUCE]        – centru ferm + reset istoric
             */
            cadre_fara_detectie++;
            eroare_anterioara = 0;  /* resetam derivativa la reluare */

            if (cadre_fara_detectie <= CADRE_HOLD)
            {
                /* pastram ultima_comanda_servo neschimbata */
            }
            else if (cadre_fara_detectie <= CADRE_REDUCE)
            {
                if (ultima_comanda_servo > OFFSET_SERVO + PAS_REDUCERE)
                    ultima_comanda_servo -= PAS_REDUCERE;
                else if (ultima_comanda_servo < OFFSET_SERVO - PAS_REDUCERE)
                    ultima_comanda_servo += PAS_REDUCERE;
                else
                    ultima_comanda_servo = OFFSET_SERVO;
            }
            else
            {
                ultima_comanda_servo = OFFSET_SERVO;
                ultim_x_stanga       = -1;
                ultim_x_dreapta      = -1;
            }

            Steer(ultima_comanda_servo);
        }



                //oled_i2c_write(OLED_ADDR, (uint8_t*)&txData, sizeof(telemetry_t));
                //i2c_scan(LPI2C3);
                /* --- Telemetrie UART (P4_2 TX către ESP32 RX) --- */
        txData.sync  = 0xA5;
                txData.yb    = (int16_t)Y_NEAR;
                txData.yb_lx = (int16_t)dbg_x_st_near;
                txData.yb_rx = (int16_t)dbg_x_dr_near;
                txData.yu    = (int16_t)Y_FAR;
                txData.yu_lx = (int16_t)dbg_x_st_far;
                txData.yu_rx = (int16_t)dbg_x_dr_far;
                txData.servo = (int16_t)comanda_servo;

                // Calcul Checksum
                uint8_t calcul_chk = 0;
                uint8_t *data_ptr = (uint8_t*)&txData;
                for(int i = 0; i < sizeof(telemetry_t) - 1; i++) {
                    calcul_chk += data_ptr[i];
                }
                txData.chk = calcul_chk;

                /* TRIMITE STRUCTURA COMPLETA (16 bytes) */
                // Foloseste LPUART2 daca pinii sunt P4_2/P4_3
                LPUART_WriteBlocking(LPUART2, (uint8_t*)&txData, sizeof(telemetry_t));

                /* Pauză mică pentru stabilitate */

        //HbridgeSpeed(&g_hbridge, 85, -85);
        SDK_DelayAtLeastUs(8000, SystemCoreClock);
    }
}
