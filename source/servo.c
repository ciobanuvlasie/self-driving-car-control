#include "fsl_ctimer.h"
#include "peripherals.h"
#include "fsl_debug_console.h"

volatile uint32_t g_msTicks = 0;

void SysTick_Handler(void)
{
    g_msTicks++;
}


void Steer(double angle)
{
    if (angle > 100.0)  angle = 100.0;

    double duty = 5.0 + ((angle + 100.0) / 200.0) * 5.0;

    uint32_t periodTicks = CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_PERIOD_CH];
    uint32_t pulseTicks = (uint32_t)((periodTicks * (100.0 - duty)) / 100.0);

    CTIMER2_PERIPHERAL->MR[2] = pulseTicks;
}

extern volatile uint32_t g_msTicks;

void TestServo(void)
{
    static int SteerStrength = -100;
    static uint32_t lastTime = 0;

    if ((g_msTicks - lastTime) >= 10)   // 10 ms
    {
        lastTime = g_msTicks;

        Steer(SteerStrength);
        SteerStrength+=25;

        if (SteerStrength > 100)
        {
            SteerStrength = -100;
        }
    }
}

