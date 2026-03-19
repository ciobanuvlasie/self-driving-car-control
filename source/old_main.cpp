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
#include "fsl_lpi2c.h"
#include <string.h>
#include"oled.h"
#include"ina.h"


static void u32_to_dec(uint32_t v, char *out)
{
    char tmp[11];
    int n = 0;
    if (v == 0) { out[0] = '0'; out[1] = 0; return; }
    while (v && n < 10) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
    for (int i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
    out[n] = 0;
}

static void i32_to_dec(int32_t v, char *out)
{
    if (v < 0) { *out++ = '-'; v = -v; }
    u32_to_dec((uint32_t)v, out);
}

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

int main(void)
{
    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    PRINTF("I2C scan start\r\n");
    i2c_scan(LPI2C2);
    PRINTF("I2C scan done\r\n");

    pixy_vector_t v[10];
    size_t n;

    pixy_t cam1;
    pixy_init(&cam1,
              LPI2C2,          // sau instanța ta reală
              0x54,            // adresa Pixy (7-bit)
              &LP_FLEXCOMM2_RX_Handle,   // EXACT ce folosești în proiectul tău
              &LP_FLEXCOMM2_TX_Handle);

    oled_init();
    HbridgeInit(&g_hbridge,
                    CTIMER0_PERIPHERAL,
                    CTIMER0_PWM_PERIOD_CH,
                    CTIMER0_PWM_1_CHANNEL,
                    CTIMER0_PWM_2_CHANNEL,
                    GPIO0, 24U,     // pin DIR/IN pentru motor 1 (exemplu)
                    GPIO0, 27U);    // pin DIR/IN pentru motor 2 (exemplu)


    // INA226: LPI2C2, addr 0x40, Rshunt=100mOhm (R100), Current_LSB = 1000uA (1mA/bit)
    ina_t ina;
    status_t st = ina_init_ina226(&ina, LPI2C2, 0x40, 100, 1000);
    (void)st; // poti face debug daca vrei

    while (1)
    {
    	status_t st = pixy_get_vectors(&cam1, v, 10, &n);

    	    PRINTF("st=%d vectors=%u\r\n", (int)st, (unsigned)n);
    	    for (size_t i = 0; i < n; i++)
    	    {
    	        PRINTF("[%u] (%u,%u)->(%u,%u) idx=%u flags=0x%02X\r\n",
    	               (unsigned)i, v[i].x0, v[i].y0, v[i].x1, v[i].y1, v[i].index, v[i].flags);
    	    }

    	    SDK_DelayAtLeastUs(100000, SystemCoreClock); // 100ms

        uint32_t vbus_mV = 0;
        int32_t  i_mA    = 0;

        (void)ina_read_vbus_mV(&ina, &vbus_mV);
        (void)ina_read_current_mA(&ina, &i_mA);

        // VBAT: X.XXX V (fara float)
        uint32_t Vint = vbus_mV / 1000;
        uint32_t frac = vbus_mV % 1000;

        char Vint_txt[12];
        u32_to_dec(Vint, Vint_txt);

        char line1[22];
        strcpy(line1, "VBAT=");
        strcat(line1, Vint_txt);
        strcat(line1, ".");

        char d[4];
        d[0] = (char)('0' + (frac / 100));
        d[1] = (char)('0' + ((frac / 10) % 10));
        d[2] = (char)('0' + (frac % 10));
        d[3] = 0;
        strcat(line1, d);
        strcat(line1, " V");

        // I: XXX MA (litere mari, pt font)
        char itxt[12];
        i32_to_dec(i_mA, itxt);

        char line2[22];
        strcpy(line2, "I=");
        strcat(line2, itxt);
        strcat(line2, " mA");

        oled_clear();
        oled_print(0, 0,  line1);
        oled_print(0, 16, line2);

        if (vbus_mV <= 6400)
            oled_print(0, 32, "LOW BAT!");


        PRINTF(line2);


        oled_update();

        //Steer(0);

        //(int)i2c_ping(LPI2C2, 0x58));
        HbridgeSpeed(&g_hbridge, 50 , -50);
        TestServo();


    }

}


