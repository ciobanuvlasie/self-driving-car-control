/* pixy.h - Pixy2 I2C (LPI2C + eDMA) mini-driver for MCUXpresso/NXP SDK
 *
 * Features:
 *  - Robust EDMA transfers (no infinite blocking on error)
 *  - SET_LED, SET_LAMP, GET_VERSION
 *  - GET_VECTORS (Pixy2 Line Tracking) with safe packet parsing
 *
 * Notes:
 *  - Pixy2 I2C default 7-bit address is 0x54
 *  - For vectors, Pixy must be in "Line Tracking" program/mode
 */

#ifndef PIXY_H_
#define PIXY_H_

#include <stdint.h>
#include <stddef.h>
#include "fsl_lpi2c_edma.h"
#include "fsl_edma.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t x0, y0;
    uint8_t x1, y1;
    uint8_t index;   // track id
    uint8_t flags;   // Pixy flags
} pixy_vector_t;

typedef struct
{
    uint8_t  major;
    uint8_t  minor;
    uint16_t build;
    char     hardware[12]; // null-terminated (best effort)
    char     firmware[12]; // null-terminated (best effort)
} pixy_version_t;

typedef struct
{
    LPI2C_Type *instance;
    uint8_t     address;     // 7-bit I2C address
    lpi2c_master_edma_handle_t edmaHandle;
} pixy_t;

/* Init: provide EDMA handles from peripherals (rx/tx) */
void pixy_init(pixy_t *cam,
               LPI2C_Type *inst,
               uint8_t addr7,
               edma_handle_t *rxHandle,
               edma_handle_t *txHandle);

/* Basic toys */
status_t pixy_set_led(pixy_t *cam, uint8_t r, uint8_t g, uint8_t b);
status_t pixy_set_lamp(pixy_t *cam, uint8_t on, uint8_t upper);

/* Version read */
status_t pixy_get_version(pixy_t *cam, pixy_version_t *out);

/* Line tracking vectors:
 *  - out: array of pixy_vector_t (max_vectors)
 *  - returns num_vectors found
 */
status_t pixy_get_vectors(pixy_t *cam,
                          pixy_vector_t *out,
                          size_t max_vectors,
                          size_t *num_vectors);

/* Compatibility helper (similar to what you had):
 * vectors packed as [x0,y0,x1,y1] in uint16_t array (4*num)
 */
status_t pixy_get_vectors_u16(pixy_t *cam,
                              uint16_t *vectors,
                              size_t max_vectors,
                              size_t *num_vectors);

#ifdef __cplusplus
}
#endif

#endif /* PIXY_H_ */
