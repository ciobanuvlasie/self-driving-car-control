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
#include "fsl_lpuart.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* conf */

#define CENTRU_IMAGINE         38

#define Y_NEAR                 48
#define Y_FAR                  20

#define KP_NEAR                3
#define KP_FAR                 2
#define KD                     0

#define OFFSET_SERVO           10

#define LATIME_BANDA_DEFAULT   30
#define LATIME_MIN             10
#define LATIME_MAX             70

#define MAX_SALT_POZITIE       20
#define DEAD_BAND              3

#define CADRE_HOLD             4
#define CADRE_REDUCE           10
#define PAS_REDUCERE           5

#define EMA_NUM                3
#define EMA_DEN                10

#define DEBUG_SERIAL           0

#define SERVO_MIN             -60
#define SERVO_MAX              60

#define LOOP_DELAY_US        10000

typedef struct {
	int ultim_x_stanga;
	int ultim_x_dreapta;
	int latime_banda;

	int eroare_anterioara;
	int deriv_filtrata;

	int cadre_fara_detectie;
	int ultima_comanda_servo;
} lane_state_t;

static lane_state_t g_lane = { .ultim_x_stanga = -1, .ultim_x_dreapta = -1,
		.latime_banda = LATIME_BANDA_DEFAULT, .eroare_anterioara = 0,
		.deriv_filtrata = 0, .cadre_fara_detectie = 0, .ultima_comanda_servo =
				OFFSET_SERVO };

typedef struct {
	int avem_centru_valid;
	int avem_centru_far;

	int centru_near;
	int centru_far;

	int avem_panta_stanga;
	int avem_panta_dreapta;
	int panta_stanga;
	int panta_dreapta;
} track_result_t;

static int g_panta_filtrata = 0;
static int g_viteza_curenta = 65;

static int limiteaza(int valoare, int minim, int maxim) {
	if (valoare < minim)
		return minim;
	if (valoare > maxim)
		return maxim;
	return valoare;
}

static int abs_i(int x) {
	return (x < 0) ? -x : x;
}

static int ema_update(int vechi, int nou) {
	return (EMA_NUM * nou + (EMA_DEN - EMA_NUM) * vechi) / EMA_DEN;
}

static uint8_t punct_jos_y(const pixy_vector_t *v) {
	return (v->y1 >= v->y0) ? v->y1 : v->y0;
}

static int lungime_vector_sq(const pixy_vector_t *v) {
	int dx = (int) v->x1 - (int) v->x0;
	int dy = (int) v->y1 - (int) v->y0;
	return dx * dx + dy * dy;
}

static int este_in_imagine(const pixy_vector_t *v) {
	return (v->x0 <= 79 && v->x1 <= 79 && v->y0 <= 52 && v->y1 <= 52);
}

static int vector_valid(const pixy_vector_t *v) {
	if (!este_in_imagine(v))
		return 0;
	if (lungime_vector_sq(v) < 25)
		return 0;
	if (punct_jos_y(v) < 20)
		return 0;
	return 1;
}

static int calculeaza_x_la_y(const pixy_vector_t *v, int y_virtual, int *x_out) {
	int x0 = (int) v->x0;
	int y0 = (int) v->y0;
	int x1 = (int) v->x1;
	int y1 = (int) v->y1;

	if (y1 == y0)
		return 0;

	int y_min = (y0 < y1) ? y0 : y1;
	int y_max = (y0 < y1) ? y1 : y0;

	if (y_virtual < y_min || y_virtual > y_max)
		return 0;

	*x_out = x0 + (y_virtual - y0) * (x1 - x0) / (y1 - y0);
	return 1;
}

static int calculeaza_panta_scaled(const pixy_vector_t *v, int *panta_out) {
	int dx = (int) v->x1 - (int) v->x0;
	int dy = (int) v->y1 - (int) v->y0;

	if (dy == 0)
		return 0;

	*panta_out = (dx * 100) / dy;
	return 1;
}

static int scor_vector(const pixy_vector_t *v, int x_curent, int are_istoric,
		int x_anterior) {
	int scor = 4 * (int) punct_jos_y(v) + lungime_vector_sq(v);

	if (are_istoric) {
		int dist = abs_i(x_curent - x_anterior);
		scor -= 8 * dist;
	}

	return scor;
}

static int selecteaza_candidat(const pixy_vector_t *vectori, size_t nr,
		int y_virtual, int centru, int vrem_stanga, int are_istoric,
		int x_anterior, int *x_selectat, int *idx_selectat) {
	int gasit = 0;
	int scor_max = -1000000;

	for (size_t i = 0; i < nr; i++) {
		int x_cur = 0;

		if (!vector_valid(&vectori[i]))
			continue;
		if (!calculeaza_x_la_y(&vectori[i], y_virtual, &x_cur))
			continue;

		if (vrem_stanga) {
			if (!(x_cur < centru))
				continue;
		} else {
			if (!(x_cur > centru))
				continue;
		}

		if (are_istoric) {
			int salt = abs_i(x_cur - x_anterior);
			if (salt > MAX_SALT_POZITIE)
				continue;
		}

		int scor = scor_vector(&vectori[i], x_cur, are_istoric, x_anterior);

		if (!gasit || scor > scor_max) {
			scor_max = scor;
			*x_selectat = x_cur;
			*idx_selectat = (int) i;
			gasit = 1;
		}
	}

	return gasit;
}

static void track_init_result(track_result_t *r) {
	r->avem_centru_valid = 0;
	r->avem_centru_far = 0;
	r->centru_near = 0;
	r->centru_far = 0;

	r->avem_panta_stanga = 0;
	r->avem_panta_dreapta = 0;
	r->panta_stanga = 0;
	r->panta_dreapta = 0;
}

static void actualizeaza_track(const pixy_vector_t *v, size_t n,
		lane_state_t *st, track_result_t *out) {
	int x_stanga = 0, x_dreapta = 0;
	int x_st_far = 0, x_dr_far = 0;

	int idx_stanga = -1;
	int idx_dreapta = -1;
	int idx_st_far = -1;
	int idx_dr_far = -1;

	track_init_result(out);

	int are_stanga = selecteaza_candidat(v, n, Y_NEAR, CENTRU_IMAGINE, 1,
			(st->ultim_x_stanga >= 0), st->ultim_x_stanga, &x_stanga,
			&idx_stanga);

	int are_dreapta = selecteaza_candidat(v, n, Y_NEAR, CENTRU_IMAGINE, 0,
			(st->ultim_x_dreapta >= 0), st->ultim_x_dreapta, &x_dreapta,
			&idx_dreapta);

	int are_st_far = selecteaza_candidat(v, n, Y_FAR, CENTRU_IMAGINE, 1,
			(st->ultim_x_stanga >= 0), st->ultim_x_stanga, &x_st_far,
			&idx_st_far);

	int are_dr_far = selecteaza_candidat(v, n, Y_FAR, CENTRU_IMAGINE, 0,
			(st->ultim_x_dreapta >= 0), st->ultim_x_dreapta, &x_dr_far,
			&idx_dr_far);

	if (are_stanga && idx_stanga >= 0) {
		if (calculeaza_panta_scaled(&v[idx_stanga], &out->panta_stanga))
			out->avem_panta_stanga = 1;
	}

	if (are_dreapta && idx_dreapta >= 0) {
		if (calculeaza_panta_scaled(&v[idx_dreapta], &out->panta_dreapta))
			out->avem_panta_dreapta = 1;
	}

	/* centru near */
	if (are_stanga && are_dreapta) {
		int latime_curenta = x_dreapta - x_stanga;

		if (latime_curenta > LATIME_MIN && latime_curenta < LATIME_MAX) {
			out->centru_near = (x_stanga + x_dreapta) / 2;
			out->avem_centru_valid = 1;

			st->latime_banda = ema_update(st->latime_banda, latime_curenta);
			st->latime_banda = limiteaza(st->latime_banda, LATIME_MIN,
					LATIME_MAX);

			st->ultim_x_stanga =
					(st->ultim_x_stanga < 0) ?
							x_stanga : ema_update(st->ultim_x_stanga, x_stanga);

			st->ultim_x_dreapta =
					(st->ultim_x_dreapta < 0) ?
							x_dreapta :
							ema_update(st->ultim_x_dreapta, x_dreapta);
		}
	} else if (are_stanga) {
		out->centru_near = x_stanga + st->latime_banda / 2;
		out->avem_centru_valid = 1;

		st->ultim_x_stanga =
				(st->ultim_x_stanga < 0) ?
						x_stanga : ema_update(st->ultim_x_stanga, x_stanga);
	} else if (are_dreapta) {
		out->centru_near = x_dreapta - st->latime_banda / 2;
		out->avem_centru_valid = 1;

		st->ultim_x_dreapta =
				(st->ultim_x_dreapta < 0) ?
						x_dreapta : ema_update(st->ultim_x_dreapta, x_dreapta);
	}

	/* centru far */
	if (are_st_far && are_dr_far) {
		int latime_far = x_dr_far - x_st_far;

		if (latime_far > LATIME_MIN && latime_far < LATIME_MAX) {
			out->centru_far = (x_st_far + x_dr_far) / 2;
			out->avem_centru_far = 1;
		}
	} else if (are_st_far) {
		out->centru_far = x_st_far + st->latime_banda / 2;
		out->avem_centru_far = 1;
	} else if (are_dr_far) {
		out->centru_far = x_dr_far - st->latime_banda / 2;
		out->avem_centru_far = 1;
	}
}

static int control_servo(lane_state_t *st, const track_result_t *tr,
		int *err_near_out, int *err_far_out) {
	int err_near = 0;
	int err_far = 0;
	int comanda_servo = st->ultima_comanda_servo;

	if (tr->avem_centru_valid) {
		err_near = tr->centru_near - CENTRU_IMAGINE;
		err_far = tr->avem_centru_far ? (tr->centru_far - CENTRU_IMAGINE) : 0;

		if (err_near >= -DEAD_BAND && err_near <= DEAD_BAND)
			err_near = 0;
		if (err_far >= -DEAD_BAND && err_far <= DEAD_BAND)
			err_far = 0;

		int eroare_totala = KP_NEAR * err_near + KP_FAR * err_far;
		int derivativa = eroare_totala - st->eroare_anterioara;

		st->deriv_filtrata = ema_update(st->deriv_filtrata, derivativa);
		st->eroare_anterioara = eroare_totala;

		comanda_servo = OFFSET_SERVO + eroare_totala + KD * st->deriv_filtrata;
		comanda_servo = limiteaza(comanda_servo, SERVO_MIN, SERVO_MAX);

		st->ultima_comanda_servo = comanda_servo;
		st->cadre_fara_detectie = 0;
	}
	else {
		st->cadre_fara_detectie++;
		st->eroare_anterioara = 0;
		st->deriv_filtrata = 0;

		if (st->cadre_fara_detectie <= CADRE_HOLD) {
			/* mentinem ultima comanda */
		} else if (st->cadre_fara_detectie <= CADRE_REDUCE) {
			if (st->ultima_comanda_servo > OFFSET_SERVO + PAS_REDUCERE)
				st->ultima_comanda_servo -= PAS_REDUCERE;
			else if (st->ultima_comanda_servo < OFFSET_SERVO - PAS_REDUCERE)
				st->ultima_comanda_servo += PAS_REDUCERE;
			else
				st->ultima_comanda_servo = OFFSET_SERVO;
		} else {
			st->ultima_comanda_servo = OFFSET_SERVO;
			st->ultim_x_stanga = -1;
			st->ultim_x_dreapta = -1;
		}

		comanda_servo = st->ultima_comanda_servo;
	}

	*err_near_out = err_near;
	*err_far_out = err_far;
	//if (comanda_servo + st->ultima_comanda_servo >= 20) return (comanda_servo + st->ultima_comanda_servo)/2;
	//if (comanda_servo + st->ultima_comanda_servo >= 20) return (comanda_servo + st->ultima_comanda_servo)/2;
	if (comanda_servo * st->ultima_comanda_servo < 0) return (comanda_servo + st->ultima_comanda_servo)/2;
	//else return (comanda_servo);

	else return comanda_servo;
}

static int calcul_viteza(const track_result_t *tr, int servo_cmd, lane_state_t *st) {
//	int indicator = 0;
//	int tinta;
//
//	if (tr->avem_panta_stanga && tr->avem_panta_dreapta) {
//		int a = tr->panta_stanga;
//		int b = tr->panta_dreapta;
//		if(abs_i(st->ultima_comanda_servo - servo_cmd) >= 20)
//		{
//			g_viteza_curenta = 80;
//		}
//		if (a < 0)
//			a = -a;
//		if (b < 0)
//			b = -b;
//		indicator = (a + b) / 2;
//	} else if (tr->avem_panta_stanga) {
//		indicator = abs_i(tr->panta_stanga);
//	} else if (tr->avem_panta_dreapta) {
//		indicator = abs_i(tr->panta_dreapta);
//
//	} else {
//		indicator = 28;
//	}
//
//	g_panta_filtrata = ema_update(g_panta_filtrata, indicator);
//	//printf("PANTA : %d \n", g_panta_filtrata);
//
//	//drum drept 50 aproximativ
//	// curba mare aproximativ 100 si cu cat tine curba creste spre 280 chiar
//	// curba 90

	if (servo_cmd >= OFFSET_SERVO - 8 && servo_cmd <= OFFSET_SERVO + 8){
		g_viteza_curenta = 55;
	}
	else g_viteza_curenta = 75;

	return g_viteza_curenta;
}

int main(void) {
	BOARD_InitHardware();
	BOARD_InitBootPins();
	BOARD_InitBootPeripherals();

	pixy_vector_t v[15];
	size_t n = 0;

	pixy_t cam1;
	pixy_init(&cam1,
	LPI2C3, 0x54, &LP_FLEXCOMM3_RX_Handle, &LP_FLEXCOMM3_TX_Handle);

	HbridgeInit(&g_hbridge,
	CTIMER0_PERIPHERAL,
	CTIMER0_PWM_PERIOD_CH,
	CTIMER0_PWM_1_CHANNEL,
	CTIMER0_PWM_2_CHANNEL,
	GPIO0, 24U,
	GPIO0, 27U);

	Steer(OFFSET_SERVO);
	//pixy_set_lamp(&cam1,0,0);
	//pixy_set_led(&cam1,0,255,0);
	SDK_DelayAtLeastUs(5000000, SystemCoreClock); // 5 secunde sta pe loc
	while (1) {
		track_result_t tr;
		int err_near = 0;
		int err_far = 0;
		int servo_cmd;
		int viteza;
		status_t status_vectori;

		status_vectori = pixy_get_vectors(&cam1, v, 10, &n);

		if (status_vectori == kStatus_Success) {
			actualizeaza_track(v, n, &g_lane, &tr);
		} else {
			track_init_result(&tr);
		}

		servo_cmd = control_servo(&g_lane, &tr, &err_near, &err_far);
		Steer(servo_cmd);

		viteza = calcul_viteza(&tr, servo_cmd,&g_lane);
		HbridgeSpeed(&g_hbridge, viteza, -viteza);
		SDK_DelayAtLeastUs(LOOP_DELAY_US, SystemCoreClock);
	}
}
