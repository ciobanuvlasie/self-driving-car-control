#include "hbridge.h"
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"

Hbridge g_hbridge;
static uint32_t s_srcClockHz;

void HbridgeInit(Hbridge *h, CTIMER_Type *pwmPeriph, ctimer_match_t periodCh,
		ctimer_match_t pwm1Ch, ctimer_match_t pwm2Ch, GPIO_Type *m1DirPort,
		uint32_t m1DirPin, GPIO_Type *m2DirPort, uint32_t m2DirPin) {
	ctimer_config_t config;

	g_hbridge = *h;
	h->periodChannel = periodCh;
	h->pwm1Channel = pwm1Ch;
	h->pwm2Channel = pwm2Ch;
	h->motor1DirPort = m1DirPort;
	h->motor1DirPin = m1DirPin;
	h->motor2DirPort = m2DirPort;
	h->motor2DirPin = m2DirPin;
	h->pwmPeripheral = pwmPeriph;
}

void HbridgeSpeed(Hbridge *h, int16_t speed1, int16_t speed2) {
	uint8_t duty1;
	if (speed1 >= 0) {
		duty1 = (uint8_t) speed1;
	} else {
		duty1 = (uint8_t) (100 + speed1);
	}
	uint8_t duty2;
	if (speed2 >= 0) {
		duty2 = (uint8_t) speed2;
	} else {
		duty2 = (uint8_t) (100 + speed2);
	}
	if (duty1 > 100U)
		duty1 = 100U;
	if (duty2 > 100U)
		duty2 = 100U;
	if (speed2 < 0) {
		GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 1U);
	} else {
		GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
	}
	if (speed1 < 0) {
		GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 1U);
	} else {
		GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);
	}
	CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel,
			h->pwm1Channel, duty1);
	CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel,
			h->pwm2Channel, duty2);
}

void HbridgeBrake(Hbridge *h) {
	GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
	GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);

	CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel,
			h->pwm1Channel, 0U);
	CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel,
			h->pwm2Channel, 0U);
}
