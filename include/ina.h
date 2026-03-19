#ifndef INA_H_
#define INA_H_

#include <stdint.h>
#include "fsl_common.h"
#include "fsl_lpi2c.h"

typedef struct
{
    LPI2C_Type *i2c;
    uint8_t     addr;       // 7-bit (ex: 0x40)
    int32_t     current_lsb_uA; // ex: 1000 => 1mA/LSB
} ina_t;

// init INA226: continuous shunt+bus + calibration for given current_lsb_uA and Rshunt_mOhm
status_t ina_init_ina226(ina_t *dev, LPI2C_Type *i2c, uint8_t addr,
                         uint32_t rshunt_mOhm, int32_t current_lsb_uA);

// Read bus voltage in mV
status_t ina_read_vbus_mV(const ina_t *dev, uint32_t *vbus_mV);

// Read current in mA (signed)
status_t ina_read_current_mA(const ina_t *dev, int32_t *current_mA);

// Optional: read shunt voltage in uV (signed)
status_t ina_read_vshunt_uV(const ina_t *dev, int32_t *vshunt_uV);

// Optional: ping device (0 = ACK, -1 = fail)
int ina_ping(const ina_t *dev);

#endif
