#include "ina.h"

// INA226 regs
#define INA_REG_CONFIG   0x00
#define INA_REG_SHUNT    0x01
#define INA_REG_BUS      0x02
#define INA_REG_POWER    0x03
#define INA_REG_CURRENT  0x04
#define INA_REG_CAL      0x05

static status_t ina_write_reg(const ina_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t b[3] = { reg, (uint8_t)(val >> 8), (uint8_t)val };
    lpi2c_master_transfer_t t = {0};

    t.slaveAddress = dev->addr;
    t.direction    = kLPI2C_Write;
    t.data         = b;
    t.dataSize     = 3;
    t.flags        = kLPI2C_TransferDefaultFlag;

    return LPI2C_MasterTransferBlocking(dev->i2c, &t);
}

static status_t ina_read_reg(const ina_t *dev, uint8_t reg, uint16_t *out)
{
    uint8_t d[2];
    lpi2c_master_transfer_t t = {0};

    t.slaveAddress    = dev->addr;
    t.direction       = kLPI2C_Read;
    t.subaddress      = reg;
    t.subaddressSize  = 1;
    t.data            = d;
    t.dataSize        = 2;
    t.flags           = kLPI2C_TransferDefaultFlag;

    status_t st = LPI2C_MasterTransferBlocking(dev->i2c, &t);
    if (st != kStatus_Success) return st;

    *out = ((uint16_t)d[0] << 8) | d[1];
    return kStatus_Success;
}

static uint16_t ina226_calc_cal(uint32_t rshunt_mOhm, int32_t current_lsb_uA)
{
    // TI formula: CAL = 0.00512 / (Current_LSB * Rshunt)
    // Current_LSB in A, Rshunt in ohm.
    // We'll compute with integer micro-units:
    // Current_LSB(A) = current_lsb_uA * 1e-6
    // Rshunt(ohm) = rshunt_mOhm * 1e-3
    //
    // CAL = 0.00512 / ( (uA*1e-6) * (mOhm*1e-3) )
    //     = 0.00512 / ( uA*mOhm*1e-9 )
    //     = 0.00512 * 1e9 / (uA*mOhm)
    //     = 5_120_000 / (uA*mOhm)
    //
    // Nice integer formula:
    // CAL = 5120000 / (current_lsb_uA * rshunt_mOhm)

    uint64_t denom = (uint64_t)current_lsb_uA * (uint64_t)rshunt_mOhm;
    if (denom == 0) return 1;

    uint32_t cal = (uint32_t)(5120000ULL / denom);
    if (cal < 1) cal = 1;
    if (cal > 65535) cal = 65535;
    return (uint16_t)cal;
}

status_t ina_init_ina226(ina_t *dev, LPI2C_Type *i2c, uint8_t addr,
                         uint32_t rshunt_mOhm, int32_t current_lsb_uA)
{
    dev->i2c = i2c;
    dev->addr = addr;
    dev->current_lsb_uA = current_lsb_uA;

    // Config: continuous shunt + bus
    // 0x4127 = ce foloseai tu și a mers
    status_t st = ina_write_reg(dev, INA_REG_CONFIG, 0x4127);
    if (st != kStatus_Success) return st;

    uint16_t cal = ina226_calc_cal(rshunt_mOhm, current_lsb_uA);
    st = ina_write_reg(dev, INA_REG_CAL, cal);
    return st;
}

status_t ina_read_vbus_mV(const ina_t *dev, uint32_t *vbus_mV)
{
    uint16_t raw = 0;
    status_t st = ina_read_reg(dev, INA_REG_BUS, &raw);
    if (st != kStatus_Success) return st;

    // INA226 bus LSB = 1.25mV/bit
    *vbus_mV = (uint32_t)raw * 1250 / 1000;
    return kStatus_Success;
}

status_t ina_read_current_mA(const ina_t *dev, int32_t *current_mA)
{
    uint16_t raw = 0;
    status_t st = ina_read_reg(dev, INA_REG_CURRENT, &raw);
    if (st != kStatus_Success) return st;

    // CURRENT register is signed in units of Current_LSB
    int32_t cur_uA = (int16_t)raw * dev->current_lsb_uA;
    *current_mA = cur_uA / 1000;
    return kStatus_Success;
}

status_t ina_read_vshunt_uV(const ina_t *dev, int32_t *vshunt_uV)
{
    uint16_t raw = 0;
    status_t st = ina_read_reg(dev, INA_REG_SHUNT, &raw);
    if (st != kStatus_Success) return st;

    // INA226 shunt LSB = 2.5uV/bit
    *vshunt_uV = (int16_t)raw * 25 / 10;
    return kStatus_Success;
}

int ina_ping(const ina_t *dev)
{
    lpi2c_master_transfer_t t = {0};
    t.slaveAddress = dev->addr;
    t.direction    = kLPI2C_Write;
    t.data         = NULL;
    t.dataSize     = 0;
    status_t st = LPI2C_MasterTransferBlocking(dev->i2c, &t);
    return (st == kStatus_Success) ? 0 : -1;
}
