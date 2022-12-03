#include "jd_drivers.h"

#ifndef TLV493D_I2C_ADDR
#define TLV493D_I2C_ADDR 0x5E
#endif

#define TLV493D_WHOAMI 0x0F


#define ACCEL_RANGE(dps, cfg, scale)                                                               \
    { dps * 1024 * 1024, cfg, scale }

static const sensor_range_t accel_ranges[] = { //
    ACCEL_RANGE(2, (0b00 << 2), 6),
    ACCEL_RANGE(4, (0b10 << 2), 7),
    ACCEL_RANGE(8, (0b11 << 2), 8),
    ACCEL_RANGE(16, (0b01 << 2), 9),
    {0, 0, 0}};



static uint8_t inited;
static const sensor_range_t *r_accel;

static void writeReg(uint8_t reg, uint8_t val) {
    i2c_write_reg(TLV493D_I2C_ADDR, reg, val);
}

static void readData(uint8_t reg, uint8_t *dst, int len) {
    i2c_read_reg_buf(TLV493D_I2C_ADDR, reg, dst, len);
}

static int readReg(uint8_t reg) {
    uint8_t r = 0;
    readData(reg, &r, 1);
    return r;
}

static void init_chip(void) {
   // writeReg(TLV493D_CTRL3_C, 0b01000100);
}

static void *tlv493d_get_sample(void) {
    int8_t data[7];
    uint8_t value[4];
    static int32_t sample[3];
    readData(TLV493D_I2C_ADDR, (uint8_t *)data, 7);
    value[0] = (data[0] << 4) | ((data[4] & 0xF0)>>4);
    value[1] = (data[1] << 4) | (data[4] & 0x0F);
    value[2] = (data[2] << 4) | (data[5] & 0x0F);
    value[3] = (data[3] >> 4) | data[6];

    sample[0] = ((int16_t) 0x0 << 16) | ((int16_t)(value[0] << 4) / 16);
    sample[1] = ((int16_t) 0x0 << 16) | ((int16_t)(value[1] << 4) / 16);
    sample[2] = ((int16_t) 0x0 << 16) | ((int16_t)(value[2] << 4) / 16);
    return sample;
}


static void tlv493d_sleep(void) {

    inited = 0;
}

static int32_t tlv493d_accel_get_range(void) {
    return r_accel->range;
}

static int32_t tlv493d_accel_set_range(int32_t range) {
    r_accel = sensor_lookup_range(accel_ranges, range);
    init_chip();
    return r_accel->range;
}


static void tlv493d_init(void) {
    if (inited)
        return;
    inited = 1;
    i2c_init();

    init_chip();
}

const accelerometer_api_t accelerometer_lsm6ds = {
    .init = tlv493d_init,
    .get_reading = tlv493d_get_sample,
    .sleep = tlv493d_sleep,
    .get_range = tlv493d_accel_get_range,
    .set_range = tlv493d_accel_set_range,
    .ranges = accel_ranges,
};
