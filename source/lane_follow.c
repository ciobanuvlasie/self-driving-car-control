#include "lane_follow.h"

static inline int16_t i16_abs(int16_t a) { return (a < 0) ? -a : a; }

// alege capătul mai "jos" (y mai mare) -> “near”
static inline void vec_near_point(const pixy_vector_t *v, int16_t *xN, int16_t *yN)
{
    if (v->y0 > v->y1) { *xN = v->x0; *yN = v->y0; }
    else              { *xN = v->x1; *yN = v->y1; }
}

// scor bun = vector mai lung + mai jos + cât mai “curat” (nu foarte oblic)
static int32_t score_vector(const pixy_vector_t *v, int16_t cx)
{
    int16_t dx = (int16_t)v->x1 - (int16_t)v->x0;
    int16_t dy = (int16_t)v->y1 - (int16_t)v->y0;
    uint32_t len2 = (uint32_t)(dx*dx + dy*dy);

    int16_t xN, yN;
    vec_near_point(v, &xN, &yN);

    // penalizează vectori prea oblici (dy mic => aproape orizontal => instabil)
    uint32_t ady = (uint32_t)i16_abs(dy);
    uint32_t adx = (uint32_t)i16_abs(dx);

    // distanță de centru (pentru stabilitate generală)
    uint32_t xdist = (uint32_t)i16_abs((int16_t)xN - cx);

    // coeficienți empirici (funcționează bine ca start)
    int32_t score = (int32_t)len2 * 6
                  + (int32_t)yN   * 30
                  + (int32_t)ady  * 40
                  - (int32_t)adx  * 15
                  - (int32_t)xdist * 10;

    return score;
}

lane_meas_t lane_measure_center(const pixy_vector_t *vecs, size_t n, int16_t img_w, lane_state_t *st)
{
    lane_meas_t m = {0};
    if (!vecs || n == 0) {
        if (st) st->lost_frames++;
        return m;
    }

    const int16_t cx = (img_w - 1) / 2;

    // best stânga / best dreapta
    int32_t bestL = -2147483647;
    int32_t bestR = -2147483647;
    bool haveL = false, haveR = false;
    int16_t xL = 0, xR = 0;

    for (size_t i = 0; i < n; i++)
    {
        const pixy_vector_t *v = &vecs[i];

        // filtru minim: ignoră segmente foarte scurte
        int16_t dx = (int16_t)v->x1 - (int16_t)v->x0;
        int16_t dy = (int16_t)v->y1 - (int16_t)v->y0;
        uint16_t len2 = (uint16_t)(dx*dx + dy*dy);
        if (len2 < 120) continue; // ridică la 200 dacă e mult noise

        int16_t xN, yN;
        vec_near_point(v, &xN, &yN);

        // ignoră lucruri prea sus (departe) dacă te încurcă:
        // (Pixy y tipic 0..51). Dacă la tine e altă scală, scoate filtrul.
        if (yN < 10) continue;

        int32_t sc = score_vector(v, cx);

        if (xN < cx) {
            if (sc > bestL) { bestL = sc; haveL = true; xL = xN; }
        } else if (xN > cx) {
            if (sc > bestR) { bestR = sc; haveR = true; xR = xN; }
        } else {
            // xN == cx -> poate fi chiar mijloc, ignorăm
        }
    }

    // caz ideal: avem ambele margini
    if (haveL && haveR)
    {
        int16_t mid = (int16_t)((xL + xR) / 2);
        m.valid = true;
        m.xL = xL; m.xR = xR; m.xMid = mid;
        m.err = (int16_t)(mid - cx);

        if (st) {
            int16_t halfw = (int16_t)i16_abs((int16_t)(xR - xL) / 2);
            // filtrare ușoară, ca să nu sară
            if (halfw > 2 && halfw < 100) {
                st->lane_half_width_px = (st->lane_half_width_px == 0) ? halfw
                                      : (int16_t)((st->lane_half_width_px * 7 + halfw) / 8);
            }
            st->lost_frames = 0;
        }
        return m;
    }

    // dacă lipsește o margine: estimează folosind lățimea ținută în stare
    if (st && st->lane_half_width_px > 0)
    {
        if (haveL && !haveR) {
            xR = (int16_t)(xL + 2 * st->lane_half_width_px);
            haveR = true;
        } else if (!haveL && haveR) {
            xL = (int16_t)(xR - 2 * st->lane_half_width_px);
            haveL = true;
        }
    }

    if (haveL && haveR)
    {
        int16_t mid = (int16_t)((xL + xR) / 2);
        m.valid = true;
        m.xL = xL; m.xR = xR; m.xMid = mid;
        m.err = (int16_t)(mid - cx);
        if (st) st->lost_frames = 0;
        return m;
    }

    // nimic util
    if (st) st->lost_frames++;
    return m;
}
