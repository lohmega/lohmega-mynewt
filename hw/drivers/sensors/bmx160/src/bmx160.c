/*
 * BMI160 BMX160 difference (incomplete)
 * =====================================
 *
 * Note the names in datasheet differs!
 * <reg_addr>:<reg_name>:<description>
 * | BMI160                  | BMX160
 * | 0x4F:MAG_IF_4:wr_data   | 0x4F:MAG_IF_3:wr_data |
 * | 0x4E:MAG_IF_3:wr_addr   | 0x4E:MAG_IF_2:wr_addr |
 * | 0x4D:MAG_IF_2:rd_addr   | 0x4D:MAG_IF_1:rd_addr |
 * | 0x4C:MAG_IF_1:config    | 0x4C:MAG_IF_0:config |
 * | 0x4B:MAG_IF_0:i2c_addr  | 0x4B:NA:reserved |
 */

#include <string.h>
#include <errno.h>
#include <assert.h>

#include "defs/error.h"
#include "os/os.h"
#include "sysinit/sysinit.h"
#include "hal/hal_i2c.h"
#include "i2cn/i2cn.h"

#include "sensor/sensor.h"
#include "sensor/accel.h"
#include "sensor/gyro.h"
#include "sensor/mag.h"
#include "log/log.h"

#include "bmx160/bmx160.h"
#include "bmx160_defs.h"
#include "bmm150.h"

struct bmx160_regval {
    uint8_t reg_addr;
    uint8_t reg_val;
};

struct bmx160_priv {

    struct bmm150_trim_regs trim_regs;
};

#ifndef ARRAY_LEN
#define ARRAY_LEN(array) (sizeof(array) / sizeof(array[0]))
#endif


#define BMX160_ITF_LOCK_TIMEOUT (OS_TIMEOUT_NEVER)
#define BMX160_RW_TIMEOUT       (OS_TICKS_PER_SEC / 10)
#define BMX160_DEBUG_ENABLE     MYNEWT_VAL(BMX160_DEBUG_ENABLE)
#define BMX160_RAW_VALUES       MYNEWT_VAL(BMX160_RAW_VALUES)



#define LOG_MODULE_BMX160    (160)
#define BMX160_LOG_INFO(...)    LOG_INFO(&_log, LOG_MODULE_BMX160, __VA_ARGS__)
#define BMX160_LOG_ERROR(...)   LOG_ERROR(&_log, LOG_MODULE_BMX160, __VA_ARGS__)
#define BMX160_LOG_DEBUG(...)   LOG_DEBUG(&_log, LOG_MODULE_BMX160, __VA_ARGS__)



static struct log _log;


__attribute__((always_inline))
static inline unsigned int unpack_u16(const uint8_t *src, uint16_t *dst)
{
    uint8_t lsb = *src++;
    uint8_t msb = *src++;
    *dst = (uint16_t)((msb << 8) | lsb);
    return sizeof(uint16_t);
}

__attribute__((always_inline))
static inline unsigned int unpack_s16(const uint8_t *src, int16_t *dst)
{
    uint8_t lsb = *src++;
    uint8_t msb = *src++;
    *dst = (int16_t)((msb << 8) | lsb);
    return sizeof(int16_t);
}

static inline struct bmx160_priv * bmx160_get_priv(struct bmx160 *bmx160)
{
    _Static_assert(sizeof(bmx160->_priv) >= sizeof(struct bmx160_priv), "");
    return (struct bmx160_priv *) bmx160->_priv;
}

static int bmx160_unpack_s16xyz(uint8_t *src, float *x, float *y, float *z, float mul)
{
    int16_t s16val = 0;

    src += unpack_s16(src, &s16val);
    *x = s16val * mul;

    src += unpack_s16(src, &s16val);
    *y = s16val * mul;

    src += unpack_s16(src, &s16val);
    *z = s16val * mul;

    return 0;
}

static int bmx160_reg_read_i2c(
        struct sensor_itf   *itf,
        uint8_t             addr,
        uint8_t             *data,
        uint8_t             size)
{
    int err = 0;

    err = sensor_itf_lock(itf, BMX160_ITF_LOCK_TIMEOUT);
    if (err) {
        return err;
    }

    struct hal_i2c_master_data op = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &addr
    };

    err = hal_i2c_master_write(itf->si_num, &op, BMX160_RW_TIMEOUT, 0);
    if (err) {
        BMX160_LOG_ERROR("I2C write failed at address 0x%02X\n", addr);
        goto done;
    }

    op.len     = size;
    op.buffer  = data;

    err = hal_i2c_master_read(itf->si_num, &op, BMX160_RW_TIMEOUT, 1);
    if (err) {
        BMX160_LOG_ERROR("I2C read failed at address 0x%02X length %u err: %d\n",
                   addr, size, err);
        goto done;
    }

done:
    sensor_itf_unlock(itf);

    return err;
}

static int bmx160_reg_read(struct bmx160 *bmx160,
              uint8_t addr,
              uint8_t *data,
              size_t size)
{
    struct sensor_itf *itf = SENSOR_GET_ITF(&bmx160->sensor);

    if (!size) {
        return SYS_EINVAL;
    }

    switch(itf->si_type) {
        case SENSOR_ITF_I2C:
            return bmx160_reg_read_i2c(itf, addr, data, size);
            break;
        case SENSOR_ITF_SPI:
        default:
            return SYS_EINVAL;
    }

    return SYS_EINVAL;
}


static unsigned int bmx160_get_cmd_post_delay_ms(uint8_t cmd_val)
{
    switch(cmd_val) {
        case BMX160_CMD_SOFTRESET:
            return 300; // 300 in manual 1 in bmi160 driver
        case BMX160_CMD_PMU_MODE_ACC_NORMAL:
        case BMX160_CMD_PMU_MODE_ACC_LOW_POWER:
            return 4 + 1;
        case BMX160_CMD_PMU_MODE_GYR_NORMAL:
        case BMX160_CMD_PMU_MODE_GYR_FAST_STARTUP:
            return 80 + 1; // from suspend mode
        case BMX160_CMD_PMU_MODE_MAG_SUSPEND:
        case BMX160_CMD_PMU_MODE_MAG_NORMAL:
        case BMX160_CMD_PMU_MODE_MAG_LOW_POWER:
            return 1 + 1;
        default:
            return 0;
    }
}

static int bmx160_reg_write(struct bmx160 *bmx160,
             uint8_t addr,
             const uint8_t *data,
             size_t size)
{
    int err;
    if (size > (sizeof(bmx160->_txbuf) - 1))
        return SYS_EINVAL;

    uint8_t *tmp = bmx160->_txbuf;
    *tmp++ = addr;
    for (size_t i = 0; i < size; i++) {
        *tmp++ = *data++;
    }

    struct sensor_itf *itf = SENSOR_GET_ITF(&bmx160->sensor);
    err = sensor_itf_lock(itf, BMX160_ITF_LOCK_TIMEOUT);
    if (err) {
        return err;
    }

    switch(itf->si_type) {
        case SENSOR_ITF_I2C: {
            struct hal_i2c_master_data op = {
                .address = itf->si_addr,
                .len     = size + 1,
                .buffer  = bmx160->_txbuf
            };

            err = hal_i2c_master_write(itf->si_num, &op, BMX160_RW_TIMEOUT, 1);
        }
            break;

        case SENSOR_ITF_SPI:
        default:
            err = SYS_EINVAL;
            break;
    }

    sensor_itf_unlock(itf);

    if (err)
        return err;

    //BMX160_LOG_DEBUG("WR_REG:0x%02X=0x%02X\n", addr, bmx160->_txbuf[1]);

    if (addr == BMX160_REG_CMD) {
        assert(size == 1);
        uint8_t cmd_val = bmx160->_txbuf[1];
        unsigned int t_ms = bmx160_get_cmd_post_delay_ms(cmd_val);
        if (t_ms)
            os_time_delay((OS_TICKS_PER_SEC * t_ms) / 1000 + 1);
    }


    return 0;
}

// wait on bmm/mag/aux operation to complete
static int bmx160_bmm_operation_wait(struct bmx160 *bmx160)
{
    int err;
    uint8_t status = 0;

    for (int i = 0; i < 512; i++) {
        err = bmx160_reg_read(bmx160, BMX160_REG_STATUS, &status, 1);
        if (err)
            return err;

        if (status & BMX160_STATUS_MAG_MAN_OP)
            os_time_delay((OS_TICKS_PER_SEC * 1) / 1000);
        else
            return 0;
    }

    return SYS_EBUSY;
}

// read bmm/mag/aux register
static int bmx160_bmm_reg_write(struct bmx160 *bmx160, uint8_t bmm_addr, const void *data, unsigned int size)
{
    int err;
    const uint8_t *bmm_data = data;
    for (unsigned int i = 0; i < size; i++) {

        err = bmx160_reg_write(bmx160, BMX160_REG_MAG_IF_3_WRD, bmm_data, 1);
        if (err)
            return err;

        err = bmx160_reg_write(bmx160, BMX160_REG_MAG_IF_2_WRA, &bmm_addr, 1);
        if (err)
            return err;

        err = bmx160_bmm_operation_wait(bmx160);
        if (err)
            return err;

        bmm_addr++;
        bmm_data++;
    }

    return 0;
}

// read bmm/mag/aux register
static int bmx160_bmm_reg_read(struct bmx160 *bmx160, uint8_t bmm_addr, void *data, unsigned int size)
{
    int err;
    uint8_t *bmm_data = data;
    for (unsigned int i = 0; i < size; i++) {

        err = bmx160_reg_write(bmx160, BMX160_REG_MAG_IF_1_RDA, &bmm_addr, 1);
        if (err)
            return err;

        err = bmx160_bmm_operation_wait(bmx160);
        if (err)
            return err;
        // assumes MAG_IF_0:mag_rd_burst=0b00=1byte (default) but should work regardless
        err = bmx160_reg_read(bmx160, BMX160_REG_MAG_DATA, bmm_data, 1);
        if (err)
            return err;

        bmm_addr++;
        bmm_data++;
    }

    return 0;
}

static int bmm150_reg_read_trim(struct bmx160 *bmx160, struct bmm150_trim_regs *trim)
{
    int err;
    uint16_t temp_msb = 0;

    uint8_t trim_x1y1[2] = { 0 };
    err = bmx160_bmm_reg_read(bmx160, BMM150_DIG_X1, trim_x1y1, 2);
    if (err)
        return err;
    trim->dig_x1 = (int8_t)trim_x1y1[0];
    trim->dig_y1 = (int8_t)trim_x1y1[1];

    uint8_t trim_xyz_data[4] = { 0 };
    err = bmx160_bmm_reg_read(bmx160, BMM150_DIG_Z4_LSB, trim_xyz_data, 4);
    if (err)
        return err;

    trim->dig_x2 = (int8_t)trim_xyz_data[2];
    trim->dig_y2 = (int8_t)trim_xyz_data[3];

    temp_msb = ((uint16_t)trim_xyz_data[1]) << 8;
    trim->dig_z4 = (int16_t)(temp_msb | trim_xyz_data[0]);

    uint8_t trim_xy1xy2[10] = { 0 };
    err = bmx160_bmm_reg_read(bmx160, BMM150_DIG_Z2_LSB, trim_xy1xy2, 10);
    if (err)
        return err;

    temp_msb = ((uint16_t)trim_xy1xy2[3]) << 8;
    trim->dig_z1 = (uint16_t)(temp_msb | trim_xy1xy2[2]);

    temp_msb = ((uint16_t)trim_xy1xy2[1]) << 8;
    trim->dig_z2 = (int16_t)(temp_msb | trim_xy1xy2[0]);

    temp_msb = ((uint16_t)trim_xy1xy2[7]) << 8;
    trim->dig_z3 = (int16_t)(temp_msb | trim_xy1xy2[6]);

    trim->dig_xy1 = trim_xy1xy2[9];
    trim->dig_xy2 = (int8_t)trim_xy1xy2[8];

    temp_msb = ((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8;
    trim->dig_xyz1 = (uint16_t)(temp_msb | trim_xy1xy2[4]);

    return 0;
}

static int bmx160_fetch_mag(struct bmx160 *bmx160, struct sensor_mag_data *smd)
{
    int err;

    struct bmx160_priv *priv = bmx160_get_priv(bmx160);
    uint8_t *buf = bmx160->_rxbuf;

    unsigned int size = sizeof(uint16_t) * 4;
    err = bmx160_reg_read(bmx160, BMX160_REG_MAG_DATA, buf, size);
    if (err)
        return err;

    int16_t raw_x, raw_y, raw_z;
    buf += unpack_s16(buf, &raw_x);
    buf += unpack_s16(buf, &raw_y);
    buf += unpack_s16(buf, &raw_z);
    uint16_t rhall = 0;
    buf += unpack_u16(buf, &rhall);


    smd->smd_x_is_valid = 1;
    smd->smd_y_is_valid = 1;
    smd->smd_z_is_valid = 1;

    if (BMX160_RAW_VALUES) {
        smd->smd_x = raw_x;
        smd->smd_y = raw_y;
        smd->smd_z = raw_z;
    }
    else {
        float f = BMX160_SI_UNIT_FACT_MAG;
        smd->smd_x = f * bmm150_compensate_xf(&priv->trim_regs, rhall, raw_x);
        smd->smd_y = f * bmm150_compensate_yf(&priv->trim_regs, rhall, raw_y);
        smd->smd_z = f * bmm150_compensate_zf(&priv->trim_regs, rhall, raw_z);
    }

    return 0;
}

static int bmx160_sd_set_config(struct sensor *sensor, void *cfg)
{
    struct bmx160 *bmx160 = (struct bmx160 *)SENSOR_GET_DEVICE(sensor);
    struct bmx160_cfg *bmx160_cfg = cfg;

    return bmx160_config(bmx160, bmx160_cfg);
}

static int
bmx160_sd_read(struct sensor *sensor,
                   sensor_type_t sensor_type,
                   sensor_data_func_t cb_func,
                   void *cb_arg,
                   uint32_t timeout)
{
    int err;
    struct bmx160 *bmx160 = (struct bmx160 *)SENSOR_GET_DEVICE(sensor);

    uint8_t status = 0;
    err = bmx160_reg_read(bmx160, BMX160_REG_STATUS, &status, 1);
    if (err)
        return err;

    if (sensor_type & SENSOR_TYPE_ACCELEROMETER) {

        if (!(status & BMX160_STATUS_DRDY_ACC))
            return SYS_EBUSY;

        uint8_t *tmp = bmx160->_rxbuf;
        err = bmx160_reg_read(bmx160, BMX160_REG_ACC_DATA, tmp, 6);
        if (err)
            return err;

        struct sensor_accel_data sad = {
            .sad_x_is_valid = 1,
            .sad_y_is_valid = 1,
            .sad_z_is_valid = 1
        };
        float tounit = BMX160_SI_UNIT_FACT_ACC;
        bmx160_unpack_s16xyz(tmp, &sad.sad_x, &sad.sad_y, &sad.sad_z, tounit);

        err = cb_func(sensor, cb_arg, &sad, SENSOR_TYPE_ACCELEROMETER);
        if (err)
            return err;
    }

    if (sensor_type & SENSOR_TYPE_GYROSCOPE) {

        if (!(status & BMX160_STATUS_DRDY_GYR))
            return SYS_EBUSY;


        uint8_t *tmp = bmx160->_rxbuf;
        err = bmx160_reg_read(bmx160, BMX160_REG_GYR_DATA, tmp, 6);
        if (err)
            return err;
        struct sensor_gyro_data sgd = {
            .sgd_x_is_valid = 1,
            .sgd_y_is_valid = 1,
            .sgd_z_is_valid = 1
        };

        float tounit = BMX160_SI_UNIT_FACT_GYR;
        bmx160_unpack_s16xyz(tmp, &sgd.sgd_x, &sgd.sgd_y, &sgd.sgd_z, tounit);

        err = cb_func(sensor, cb_arg, &sgd, SENSOR_TYPE_GYROSCOPE);
        if (err)
            return err;
    }

    if (sensor_type & SENSOR_TYPE_MAGNETIC_FIELD) {

        if (!(status & BMX160_STATUS_DRDY_MAG))
            return SYS_EBUSY;

        struct sensor_mag_data smd = { 0 };
        err = bmx160_fetch_mag(bmx160, &smd);
        if (err)
            return err;

        err = cb_func(sensor, cb_arg, &smd, SENSOR_TYPE_MAGNETIC_FIELD);
        if (err)
            return err;
    }

    if (sensor_type & SENSOR_TYPE_TEMPERATURE) {
        return SYS_EINVAL; // TODO
    }

    return 0;
}

static int bmx160_sd_get_config(struct sensor * sensor,
                         sensor_type_t sensor_type,
                         struct sensor_cfg * cfg)
{

    if (sensor_type & SENSOR_TYPE_ACCELEROMETER) {
        cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT_TRIPLET;
    }

    if (sensor_type & SENSOR_TYPE_GYROSCOPE) {
        cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT_TRIPLET;
    }

    if (sensor_type & SENSOR_TYPE_MAGNETIC_FIELD) {
        cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT_TRIPLET;
    }

    if (sensor_type & SENSOR_TYPE_TEMPERATURE) {
        cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT;
    }

    return 0;
}


static struct sensor_driver bmx160_sensor_driver = {
    .sd_read               = bmx160_sd_read,
    .sd_get_config         = bmx160_sd_get_config,
    //.sd_set_config         = bmx160_sd_set_config,
    //.sd_set_trigger_thresh = bmx160_sd_set_trigger_thresh,
    //.sd_set_notification   = bmx160_sd_set_notification,
    //.sd_unset_notification = bmx160_sd_unset_notification,
    //.sd_handle_interrupt   = bmx160_sd_handle_interrupt,
};


static int bmx160_config_acc(struct bmx160 *bmx160, const struct bmx160_cfg *cfg)
{
    int err;
    static const struct bmx160_regval regs[] = {
        {BMX160_REG_CMD, BMX160_CMD_PMU_MODE_ACC_NORMAL},
        {BMX160_REG_ACC_CONF, BMX160_ACC_CONF_BWP_NORMAL_AVG4 |
                              BMX160_ACC_CONF_ODR_100HZ},
        {BMX160_REG_ACC_RANGE, BMX160_ACC_RANGE_2G}
    };

    for (int i = 0; i < ARRAY_LEN(regs); i++) {
        err = bmx160_reg_write(bmx160, regs[i].reg_addr, &regs[i].reg_val, 1);
        assert(!err);
    }

    return 0;
}

static int bmx160_config_gyr(struct bmx160 *bmx160, const struct bmx160_cfg *cfg)
{
    int err;
    static const struct bmx160_regval regs[] = {
        {BMX160_REG_CMD, BMX160_CMD_PMU_MODE_GYR_NORMAL},
        {BMX160_REG_GYR_CONF, BMX160_GYR_CONF_BWP_NORMAL |
                              BMX160_GYR_CONF_ODR_100HZ},
        {BMX160_REG_GYR_RANGE, BMX160_GYR_RANGE_2000_DPS}
    };

    for (int i = 0; i < ARRAY_LEN(regs); i++) {
        err = bmx160_reg_write(bmx160, regs[i].reg_addr, &regs[i].reg_val, 1);
        assert(!err);
    }

    return 0;
}

static int bmx160_config_mag(struct bmx160 *bmx160, const struct bmx160_cfg *cfg)
{
    int err;
    uint8_t regval = 0;

#define BMM150_SET_REG(REG, VAL) do { \
       uint8_t __val = (VAL); \
       int __err = bmx160_bmm_reg_write(bmx160, (REG), &__val, 1); \
       assert(!__err); \
    } while(0)

#define BMM150_GET_REG(BMM_ADDR, DST) do { \
       int __err = bmx160_bmm_reg_read(bmx160, (BMM_ADDR), (DST), 1); \
       assert(!__err); \
    } while(0)

#define BMX160_SET_REG(REG, VAL) do { \
       uint8_t __val = (VAL); \
       int __err = bmx160_reg_write(bmx160, (REG), &__val, 1); \
       assert(!__err); \
    } while(0)

#define BMX160_GET_REG(REG, DST) do { \
       int __err = bmx160_reg_read(bmx160, (REG), (DST), 1); \
       assert(!__err); \
    } while(0)

#define BMX160_ASSERT_ERR_REG_ZERO() do { \
        if (BMX160_DEBUG_ENABLE) { \
            uint8_t __err_reg = 0; \
            int __err = bmx160_reg_read(bmx160, BMX160_REG_ERROR, &__err_reg, 1); \
            assert(!__err); \
            if (__err_reg & 0xFF) { \
                BMX160_LOG_ERROR("%d:mag_pmu: err_reg=0x%02X\n", __LINE__, __err_reg); \
            }  \
            assert(!__err_reg); \
        } \
    } while(0)

    struct bmx160_priv *priv = bmx160_get_priv(bmx160);

    /* Undocumented and supposed unused bits in BMX160. see BMI160 manual for
     * details. Some chips have a unresponding magnetometer without this
     * write. Implied fix in Bosch forum thread
     * "BMX160-Magnetometer-data-not-changing". */
    BMX160_SET_REG(BMX160_REG_IF_CONF, 0x20);

    BMX160_SET_REG(BMX160_REG_CMD, BMX160_CMD_PMU_MODE_MAG_NORMAL);
    BMX160_SET_REG(BMX160_REG_MAG_IF_0_CFG, 0x80);

    /* if soft/hard reset needed it should problably happen here!?
     *   BMM150_SET_REG(BMM150_REG_POWER, 0x0); // POR reset
     * or
     *   BMM150_SET_REG(BMM150_REG_POWER, 0x01 | 0x82);  // soft reset
     * os_time_delay((OS_TICKS_PER_SEC * 3) / 1000); // reset wait
     */

    /* 0x01 --> power_control=1 (default).
     * note that this operation seems to set ERR_REG to 0x80 (cmd_drop)
     * but is required and used in manual example(s). */
    BMM150_SET_REG(BMM150_REG_POWER, 0x01);

    if (BMX160_DEBUG_ENABLE) {
        // verify bmm150 and bmx160 both reports mag normal mode (0x01)
        // (as somtimes they disagree and nothing works)
        BMM150_GET_REG(BMM150_REG_POWER, &regval);
        assert((regval & 0x01) == 0x01); // power_control==1=normal_mode

        BMX160_GET_REG(BMX160_REG_PMU_STATUS, &regval);
        assert((regval & 0x03) == 0x01);
    }

    // sanity check to verify the on-board bmm150 responds
    BMM150_GET_REG(BMM150_REG_CHIP_ID, &regval);
    assert(regval == 0x32);

    err = bmm150_reg_read_trim(bmx160, &priv->trim_regs);
    assert(!err);

    BMM150_SET_REG(BMM150_REG_REP_XY, 0x01);
    BMM150_SET_REG(BMM150_REG_REP_Z, 0x0E);
    BMM150_SET_REG(BMM150_REG_OPMODE_ODR, 0x02); // Opmode = forced mode

    // unclear if/why this read needed. used in manual example
    BMM150_GET_REG(BMM150_REG_DATA_X_L, &regval);
    (void) regval;

    BMX160_SET_REG(BMX160_REG_MAG_CONF, BMX160_MAG_CONF_ODR_12_5HZ);
    BMX160_SET_REG(BMX160_REG_MAG_IF_0_CFG, BMX160_MAG_IF_0_CFG_RD_BURST_8);

    return 0;

}

int bmx160_config(struct bmx160 *bmx160, const struct bmx160_cfg *cfg)
{
    (void) bmx160_sd_set_config;
    int err = 0;

    // TODO en_mask from config
    sensor_type_t en_mask = (0
        | SENSOR_TYPE_ACCELEROMETER
        | SENSOR_TYPE_GYROSCOPE
        | SENSOR_TYPE_MAGNETIC_FIELD
    );

    struct sensor *sensor = &bmx160->sensor;

    uint8_t cmd_val = BMX160_CMD_SOFTRESET;
    err = bmx160_reg_write(bmx160, BMX160_REG_CMD, &cmd_val, 1);
    assert(!err);

    // sanity check on magic chip_id
    uint8_t chip_id = 0;
    err = bmx160_reg_read(bmx160, BMX160_REG_CHIP_ID, &chip_id, 1);
    assert(!err);
    assert(chip_id == 0xD8);

    if (en_mask & SENSOR_TYPE_ACCELEROMETER) {
        err = bmx160_config_acc(bmx160, cfg);
        assert(!err);
    }

    if (en_mask & SENSOR_TYPE_GYROSCOPE) {
        err = bmx160_config_gyr(bmx160, cfg);
        assert(!err);
    }

    if (en_mask & SENSOR_TYPE_MAGNETIC_FIELD) {
        err = bmx160_config_mag(bmx160, cfg);
        assert(!err);
    }

    // note: manual says err_reg should not be used for success verification
    if (BMX160_DEBUG_ENABLE) {
        /* in bosch driver bmi160 driver, err_reg bit mask is in practice 0b00000111 (0x7),
         * but manual says 0xFF or 0x5F - depending on what page(s) read.
         * writeing power_control_bit to BMM150_REG_POWER @ 0x4B seems to set ERR_REG to 0x80 (cmd_dropped)
         * */
        uint8_t err_reg = 0;
        err = bmx160_reg_read(bmx160, BMX160_REG_ERROR, &err_reg, 1);
        assert(!err);
        if (err_reg & 0x7F) {
            BMX160_LOG_ERROR("err_reg=0x%02X\n", err_reg);
        }
    }

    err = sensor_set_type_mask(sensor, en_mask);
    assert(!err);

    return 0;
}

int bmx160_init(struct os_dev *dev, void *arg)
{
    int err;
    err = log_register("bmx160", &_log, &log_console_handler, NULL, LOG_SYSLEVEL);
    assert(!err);

    struct bmx160 *bmx160 = (struct bmx160 *)dev;
    struct sensor *sensor = &bmx160->sensor;

    err = sensor_init(sensor, dev);
    assert(!err);
    // TODO all sensor types
    err = sensor_set_driver(sensor,
            (SENSOR_TYPE_GYROSCOPE |
            SENSOR_TYPE_MAGNETIC_FIELD |
            SENSOR_TYPE_ACCELEROMETER),
                           &bmx160_sensor_driver);
    assert(!err);

    err = sensor_set_interface(sensor, arg);
    assert(!err);

    err = sensor_mgr_register(sensor);
    assert(!err);

    return 0;
}
