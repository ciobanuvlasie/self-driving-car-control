#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "pixy.h"

typedef struct
{
    // estimare lățime bandă (în pixeli Pixy), folosită când lipsește o margine
    int16_t lane_half_width_px;   // ~ (xR-xL)/2
    double  last_angle;           // fallback smoothing
    uint32_t lost_frames;
} lane_state_t;

typedef struct
{
    bool valid;
    int16_t xL;     // coord x “near” pentru marginea stângă
    int16_t xR;     // coord x “near” pentru marginea dreaptă
    int16_t xMid;   // mijlocul culoarului
    int16_t err;    // xMid - xCenter
} lane_meas_t;

// img_w = lățimea în pixeli a coordonatelor Pixy pentru vectors (de obicei 79)
lane_meas_t lane_measure_center(const pixy_vector_t *v, size_t n, int16_t img_w, lane_state_t *st);
