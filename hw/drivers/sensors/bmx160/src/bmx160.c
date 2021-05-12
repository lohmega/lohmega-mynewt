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
#include "hal/hal_gpio.h"
#include "hal/hal_i2c.h"
#include "i2cn/i2cn.h"

#include "sensor/sensor.h"
#include "sensor/accel.h"
#include "sensor/gyro.h"
#include "sensor/mag.h"
#include "log/log.h"

#include "bmx160/bmx160.h"
#include "bmx160/bmx160_defs.h"
#include "bmm150.h"

struct bmx160_regval {
    uint8_t reg_addr;
    uint8_t reg_val;
};

struct bmx160_priv {

    struct bmm150_trim_regs trim_regs;
};


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

static inline struct bmx160_priv *bmx160_get_priv(struct bmx160 *bmx160)
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

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
static void bmx160_init_node_cb(struct bus_node *bnode, void *arg)
{
    struct sensor_itf *itf = arg;
    bmx160_init((struct os_dev *)bnode, itf);
}

static struct bus_node_callbacks bmx160_bus_node_cbs = {
   .init = bmx160_init_node_cb,
};

int bmx160_create_i2c_sensor_dev(struct bus_i2c_node *node, const char *name,
                              const struct bus_i2c_node_cfg *i2c_cfg,
                              struct sensor_itf *sensor_itf)
{
    sensor_itf->si_dev = &node->bnode.odev;
    bus_node_set_callbacks((struct os_dev *)node, &bmx160_bus_node_cbs);
    return bus_i2c_node_create(name, node, i2c_cfg, sensor_itf);
}
#endif

static int bmx160_reg_read(struct bmx160 *bmx160,
              uint8_t addr,
              uint8_t *data,
              size_t size)
{
    int err = 0;
    struct sensor_itf *itf = SENSOR_GET_ITF(&bmx160->sensor);

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    err = bus_node_simple_write_read_transact(itf->si_dev, &addr, 1, data, size);
#else

    if (!size) {
        return SYS_EINVAL;
    }
    // only support i2c for this interface
    if (itf->si_type != SENSOR_ITF_I2C) {
        return SYS_EINVAL;
    }

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
#endif

    return err;

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
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    err = bus_node_simple_write(itf->si_dev, bmx160->_txbuf, size + 1);
#else

    // only support i2c for this interface
    if (itf->si_type != SENSOR_ITF_I2C) {
        return SYS_EINVAL;
    }

    err = sensor_itf_lock(itf, BMX160_ITF_LOCK_TIMEOUT);
    if (err) {
        return err;
    }

    struct hal_i2c_master_data op = {
        .address = itf->si_addr,
        .len     = size + 1,
        .buffer  = bmx160->_txbuf
    };

    err = hal_i2c_master_write(itf->si_num, &op, BMX160_RW_TIMEOUT, 1);

    sensor_itf_unlock(itf);
#endif
    if (err) {
        return err;
    }
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

static int
bmx160_calc_acc(struct bmx160 *bmx160, uint8_t *buf, struct sensor_accel_data *sad)
{
    float tounit = BMX160_SI_UNIT_FACT_ACC_2G;

    sad->sad_x_is_valid = 1;
    sad->sad_y_is_valid = 1;
    sad->sad_z_is_valid = 1;

    switch (bmx160->cfg.acc_range) {
        case BMX160_ACC_RANGE_2G:
            break;
        case BMX160_ACC_RANGE_4G:
            tounit = BMX160_SI_UNIT_FACT_ACC_2G * 2;
            break;
        case BMX160_ACC_RANGE_8G:
            tounit = BMX160_SI_UNIT_FACT_ACC_2G * 4;
            break;
        case BMX160_ACC_RANGE_16G:
            tounit = BMX160_SI_UNIT_FACT_ACC_2G * 8;
            break;
    }
    bmx160_unpack_s16xyz(buf, &sad->sad_x, &sad->sad_y, &sad->sad_z, tounit);

    return 0;
}

static int
bmx160_calc_gyr(struct bmx160 *bmx160, uint8_t *buf, struct sensor_gyro_data *sgd)
{
    float tounit = BMX160_SI_UNIT_FACT_GYR_2000DPS;

    sgd->sgd_x_is_valid = 1;
    sgd->sgd_y_is_valid = 1;
    sgd->sgd_z_is_valid = 1;

    switch (bmx160->cfg.gyro_range) {
        case BMX160_GYR_RANGE_2000_DPS:
            break;
        case BMX160_GYR_RANGE_1000_DPS:
            tounit = BMX160_SI_UNIT_FACT_GYR_2000DPS / 2;
            break;
        case BMX160_GYR_RANGE_500_DPS:
            tounit = BMX160_SI_UNIT_FACT_GYR_2000DPS / 4;
            break;
        case BMX160_GYR_RANGE_250_DPS:
            tounit = BMX160_SI_UNIT_FACT_GYR_2000DPS / 8;
            break;
        case BMX160_GYR_RANGE_125_DPS:
            tounit = BMX160_SI_UNIT_FACT_GYR_2000DPS / 16;
            break;
    }
    bmx160_unpack_s16xyz(buf, &sgd->sgd_x, &sgd->sgd_y, &sgd->sgd_z, tounit);

    return 0;
}

static int
bmx160_calc_mag(struct bmx160 *bmx160, uint8_t *buf, struct sensor_mag_data *smd)
{
    int16_t raw_x, raw_y, raw_z;
    uint16_t rhall = 0;
    struct bmx160_priv *priv = bmx160_get_priv(bmx160);

    /* Reg manipulations taken from bmm150_read_mag_data */
    /* X (b[0] & 0xF8) >> 3 | (b[1] << 5) */
    raw_x = BMM150_GET_BITS(buf[0], BMM150_DATA_X) | (((int16_t)((int8_t)buf[1])) *32);
    /* Y (b[0] & 0xF8) >> 3 | (b[1] << 5) */
    raw_y = BMM150_GET_BITS(buf[2], BMM150_DATA_Y) | (((int16_t)((int8_t)buf[3])) *32);
    /* Z (b[0] & 0xFE) >> 1 | (b[1] << 7) */
    raw_z = BMM150_GET_BITS(buf[4], BMM150_DATA_Z) | (((int16_t)((int8_t)buf[5])) *128);
    /* RHALL (b[0] & 0xFC) >> 2 | (b[1] << 6) */
    rhall = BMM150_GET_BITS(buf[6], BMM150_DATA_RHALL) | (((int16_t)((int8_t)buf[7])) *64);

    smd->smd_x_is_valid = 1;
    smd->smd_y_is_valid = 1;
    smd->smd_z_is_valid = 1;

    if (BMX160_RAW_VALUES) {
        smd->smd_x = raw_x;
        smd->smd_y = raw_y;
        smd->smd_z = raw_z;
    }
    else {
        /* bmm150_compensate_* returns uT already */
        float f = 1.0f;
        smd->smd_x = f*bmm150_compensate_xf(&priv->trim_regs, rhall, raw_x);
        smd->smd_y = f*bmm150_compensate_yf(&priv->trim_regs, rhall, raw_y);
        smd->smd_z = f*bmm150_compensate_zf(&priv->trim_regs, rhall, raw_z);
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
    const sensor_type_t sensors_acc_gyr = SENSOR_TYPE_ACCELEROMETER|SENSOR_TYPE_GYROSCOPE;
    const sensor_type_t sensors_all_mot = sensors_acc_gyr | SENSOR_TYPE_MAGNETIC_FIELD;
    const sensor_type_t sensors_all = sensors_all_mot | SENSOR_TYPE_TEMPERATURE;
    uint8_t status = 0;
    bool rxbuf_preloaded = false;
    struct bmx160 *bmx160 = (struct bmx160 *)SENSOR_GET_DEVICE(sensor);
    if ((sensor_type & sensors_all) == 0) {
        err = SYS_EINVAL;
        return err;
    }

    /* Read the status register first as reading the sensor data
     * clears the corresponding bits in the status reg */
    err = bmx160_reg_read(bmx160, BMX160_REG_STATUS, &status, 1);
    if (err) {
        return err;
    }

    if ((sensor_type & sensors_all_mot) == sensors_all_mot) {
        /* Read all sensors in a single read */
        err = bmx160_reg_read(bmx160, BMX160_REG_MAG_DATA, bmx160->_rxbuf, 20);
        if (err) {
            return err;
        }
        rxbuf_preloaded = true;
    } else if ((sensor_type & sensors_acc_gyr) == sensors_acc_gyr) {
        /* Read acc and gyro sensors in a single read */
        err = bmx160_reg_read(bmx160, BMX160_REG_GYR_DATA,
                              bmx160->_rxbuf + (BMX160_REG_GYR_DATA - BMX160_REG_MAG_DATA),
                              12);
        if (err) {
            return err;
        }
        rxbuf_preloaded = true;
    }

    if (sensor_type & SENSOR_TYPE_ACCELEROMETER) {
        struct sensor_accel_data sad;

        if (!(status & BMX160_STATUS_DRDY_ACC))
            return SYS_EBUSY;

        uint8_t *tmp = bmx160->_rxbuf;
        if (rxbuf_preloaded) {
            tmp = &bmx160->_rxbuf[BMX160_REG_ACC_DATA - BMX160_REG_MAG_DATA];
        } else {
            err = bmx160_reg_read(bmx160, BMX160_REG_ACC_DATA, tmp, 6);
            if (err) {
                return err;
            }
        }

        bmx160_calc_acc(bmx160, tmp, &sad);

        err = cb_func(sensor, cb_arg, &sad, SENSOR_TYPE_ACCELEROMETER);
        if (err)
            return err;
    }

    if (sensor_type & SENSOR_TYPE_GYROSCOPE) {
        struct sensor_gyro_data sgd;

        if (!(status & BMX160_STATUS_DRDY_GYR))
            return SYS_EBUSY;

        if (err)
            return err;
        uint8_t *tmp = bmx160->_rxbuf;
        if (rxbuf_preloaded) {
            tmp = &bmx160->_rxbuf[BMX160_REG_GYR_DATA - BMX160_REG_MAG_DATA];
        } else {
            err = bmx160_reg_read(bmx160, BMX160_REG_GYR_DATA, tmp, 6);
            if (err) {
                return err;
            }
        }

        bmx160_calc_gyr(bmx160, tmp, &sgd);

        err = cb_func(sensor, cb_arg, &sgd, SENSOR_TYPE_GYROSCOPE);
        if (err)
            return err;
    }

    if (sensor_type & SENSOR_TYPE_MAGNETIC_FIELD) {

        if (!(status & BMX160_STATUS_DRDY_MAG))
            return SYS_EBUSY;

        struct sensor_mag_data smd = { 0 };

        uint8_t *tmp = bmx160->_rxbuf;
        if (!rxbuf_preloaded) {
            err = bmx160_reg_read(bmx160, BMX160_REG_MAG_DATA, tmp, sizeof(uint16_t) * 4);
            if (err) {
                return err;
            }
        }

        err = bmx160_calc_mag(bmx160, tmp, &smd);
        if (err) {
            return err;
        }

        err = cb_func(sensor, cb_arg, &smd, SENSOR_TYPE_MAGNETIC_FIELD);
        if (err)
            return err;
    }

    if (sensor_type & SENSOR_TYPE_TEMPERATURE) {
        struct sensor_temp_data std = {0};
        int16_t val = 0;
        uint8_t *tmp = bmx160->_rxbuf;
        err = bmx160_reg_read(bmx160, BMX160_REG_TEMP_DATA, tmp, sizeof(uint16_t));
        if (err) {
            return err;
        }
        unpack_s16(tmp, &val);
        std.std_temp = 23.0f + val * (1.0f/512.0f);
        err = cb_func(sensor, cb_arg, &std, SENSOR_TYPE_TEMPERATURE);
        if (err)
            return err;
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

static void
call_listener_cbs(struct sensor *sensor, sensor_type_t type, void *data)
{
    struct sensor_listener *listener;

    SLIST_FOREACH(listener, &sensor->s_listener_list, sl_next) {
        if (listener->sl_sensor_type & type) {
            listener->sl_func(sensor, listener->sl_arg, data, type);
        }
    }
}

static int
interpret_fifo(struct bmx160 *bmx160, struct sensor *sensor, uint8_t *buf, int buf_len)
{
    int off = 0;
    uint8_t *tmp;
    const uint8_t overread_val[] = {0x80,0x80,0x80,0x80,0x80,0x80};
    struct sensor_accel_data sad;
    struct sensor_gyro_data sgd;
    struct sensor_mag_data smd;

    while ((off + 19) < buf_len) {
        tmp = buf + off;

        if (!memcmp(tmp, overread_val, sizeof(overread_val))) {
            printf("%s:%d: fifo_or\n", __func__, __LINE__);
        }

        /* Todo check for partial fifo downloads */
        if (!bmx160_calc_acc(bmx160, &tmp[BMX160_REG_ACC_DATA - BMX160_REG_MAG_DATA], &sad)) {
            call_listener_cbs(sensor, SENSOR_TYPE_ACCELEROMETER, &sad);
        }
        if (!bmx160_calc_gyr(bmx160, &tmp[BMX160_REG_GYR_DATA - BMX160_REG_MAG_DATA], &sgd)) {
            call_listener_cbs(sensor, SENSOR_TYPE_GYROSCOPE, &sgd);
        }
        if (!bmx160_calc_mag(bmx160, &tmp[BMX160_REG_MAG_DATA - BMX160_REG_MAG_DATA], &smd)) {
            call_listener_cbs(sensor, SENSOR_TYPE_MAGNETIC_FIELD, &smd);
        }
        off += 20;
        sensor->s_sts.st_cputime += os_cputime_usecs_to_ticks(bmx160->fifo_tbase);
    }
    return 0;
}

/**
 * Manage events from sensor
 *
 * @param sensor The sensor object
 *
 * @return 0 on success, non-zero error on failure.
 */
static int
bmx160_sd_handle_interrupt(struct sensor *sensor)
{
    struct bmx160 *bmx160;
    int err;
    uint16_t fifo_length;

    err = sensor_lock(sensor);
    if (err != 0) {
        return err;
    }
    bmx160 = (struct bmx160 *)SENSOR_GET_DEVICE(sensor);

    /* Interrupt status */
    err = bmx160_reg_read(bmx160, BMX160_REG_INT_STATUS, bmx160->_rxbuf, 2);
    if (!(bmx160->_rxbuf[1] & (BMX160_INT_STATUS_1_FWM|BMX160_INT_STATUS_1_FFULL))) {
        return 0;
    }

    /* Fifo length */
    err = bmx160_reg_read(bmx160, BMX160_REG_FIFO_LENGTH, bmx160->_rxbuf, 2);
    unpack_u16(bmx160->_rxbuf, &fifo_length);

    /* FIFO overflow - throw away and restart fifo */
    if (fifo_length > 999) {
        uint8_t val = BMX160_CMD_FIFO_FLUSH;
        err = bmx160_reg_write(bmx160, BMX160_REG_CMD, &val, 1);
        assert(!err);
        return 0;
    }

    /* Calculate timestamp of first sample in fifo, note that the interrupt was given at
     * the time the watermark was crossed. */
    int64_t ts_offset = (bmx160->cfg.fifo_water_level*4/20)*bmx160->fifo_tbase;
    sensor->s_sts.st_cputime = bmx160->int1_ct - os_cputime_usecs_to_ticks(ts_offset);

    while (fifo_length > 16) {
        int to_read = fifo_length;
        to_read = (to_read < sizeof(bmx160->_rxbuf)) ? to_read : sizeof(bmx160->_rxbuf);
        err = bmx160_reg_read(bmx160, BMX160_REG_FIFO_DATA, bmx160->_rxbuf, to_read);
        if (err) {
            break;
        }

        interpret_fifo(bmx160, sensor, bmx160->_rxbuf, to_read);

        err = bmx160_reg_read(bmx160, BMX160_REG_FIFO_LENGTH, bmx160->_rxbuf, 2);
        unpack_u16(bmx160->_rxbuf, &fifo_length);
    }

    sensor_unlock(sensor);
    return err;
}

static struct sensor_driver bmx160_sensor_driver = {
    .sd_read               = bmx160_sd_read,
    .sd_get_config         = bmx160_sd_get_config,
    //.sd_set_config         = bmx160_sd_set_config,
    //.sd_set_trigger_thresh = bmx160_sd_set_trigger_thresh,
    //.sd_set_notification   = bmx160_sd_set_notification,
    //.sd_unset_notification = bmx160_sd_unset_notification,
    .sd_handle_interrupt   = bmx160_sd_handle_interrupt,
};


static int bmx160_config_acc(struct bmx160 *bmx160, const struct bmx160_cfg *cfg)
{
    int err;
    struct bmx160_regval regs[] = {
        {BMX160_REG_CMD, 0},
        {BMX160_REG_ACC_CONF, 0},
        {BMX160_REG_ACC_RANGE, 0}
    };
    regs[0].reg_val = cfg->acc_mode;
    regs[1].reg_val = cfg->acc_rate;
    regs[2].reg_val = cfg->acc_range;

    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        err = bmx160_reg_write(bmx160, regs[i].reg_addr, &regs[i].reg_val, 1);
        assert(!err);
    }

    return 0;
}

static int bmx160_config_gyr(struct bmx160 *bmx160, const struct bmx160_cfg *cfg)
{
    int err;
    struct bmx160_regval regs[] = {
        {BMX160_REG_CMD, 0},
        {BMX160_REG_GYR_CONF, 0},
        {BMX160_REG_GYR_RANGE, 0}
    };
    regs[0].reg_val = cfg->gyro_mode;
    regs[1].reg_val = cfg->gyro_rate;
    regs[2].reg_val = cfg->gyro_range;

    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
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

    BMX160_SET_REG(BMX160_REG_MAG_CONF, cfg->mag_rate);
    BMX160_SET_REG(BMX160_REG_MAG_IF_0_CFG, BMX160_MAG_IF_0_CFG_RD_BURST_8);

    /* Set actual mode wanted for magnetometer here [Normal, suspend, ...] */
    BMX160_SET_REG(BMX160_REG_CMD, cfg->mag_mode);

    return 0;

}

static void
bmx160_irq1(void *arg)
{
    struct sensor *sensor = arg;
    struct bmx160 *bmx160;
    bmx160 = (struct bmx160 *)SENSOR_GET_DEVICE(sensor);
    bmx160->int1_ct = os_cputime_get32();

    sensor_mgr_put_interrupt_evt(sensor);
}

static int
bmx160_config_fifo(struct bmx160 *bmx160, const struct bmx160_cfg *cfg)
{
    int err;
    struct bmx160_regval regs[] = {
        {BMX160_REG_CMD, BMX160_CMD_INT_RESET},
        {BMX160_REG_INT_OUT_CTRL, (BMX160_INT_OUT_CTRL_INT1_OEN|BMX160_INT_OUT_CTRL_INT1_LVL)},
        {BMX160_REG_INT_MAP_1, (BMX160_INT_MAP_1_INT1_FWM)},
        {BMX160_REG_INT_ENABLE_1, BMX160_INT_ENABLE_1_FWM},
        {BMX160_REG_INT_LATCH, 0x0F},
        {BMX160_REG_FIFO_DOWNS, 0},
        {BMX160_REG_FIFO_CONF_0, 0},
        {BMX160_REG_FIFO_CONF_1, 0},
        {BMX160_REG_CMD, BMX160_CMD_FIFO_FLUSH}
    };
    regs[6].reg_val = cfg->fifo_water_level; /* FIFO_CONF_0 */
    regs[7].reg_val = cfg->fifo_enable;      /* FIFO_CONF_1 */

    /* Calculate tbase from ODR */
    bmx160->fifo_tbase = 10000;
#if 1
    switch (cfg->acc_rate & 0xf) {
        case (BMX160_ACC_CONF_ODR_25HZ):  bmx160->fifo_tbase = 40000;break;
        case (BMX160_ACC_CONF_ODR_50HZ):  bmx160->fifo_tbase = 20000;break;
        case (BMX160_ACC_CONF_ODR_100HZ): bmx160->fifo_tbase = 10000;break;
        case (BMX160_ACC_CONF_ODR_200HZ): bmx160->fifo_tbase = 5000;break;
        default:
            assert(0);
    }
#endif

    assert((cfg->acc_rate&0xf) == (cfg->gyro_rate&0xf));
    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        err = bmx160_reg_write(bmx160, regs[i].reg_addr, &regs[i].reg_val, 1);
        assert(!err);
    }
    return 0;
}


int
bmx160_config(struct bmx160 *bmx160, const struct bmx160_cfg *cfg)
{
    (void) bmx160_sd_set_config;
    int err = 0;

    struct sensor *sensor = &bmx160->sensor;

    uint8_t cmd_val = BMX160_CMD_SOFTRESET;
    err = bmx160_reg_write(bmx160, BMX160_REG_CMD, &cmd_val, 1);
    assert(!err);

    // sanity check on magic chip_id
    uint8_t chip_id = 0;
    err = bmx160_reg_read(bmx160, BMX160_REG_CHIP_ID, &chip_id, 1);
    assert(!err);
    assert(chip_id == 0xD8);

    if (cfg->en_mask & SENSOR_TYPE_ACCELEROMETER) {
        err = bmx160_config_acc(bmx160, cfg);
        assert(!err);
    }

    if (cfg->en_mask & SENSOR_TYPE_GYROSCOPE) {
        err = bmx160_config_gyr(bmx160, cfg);
        assert(!err);
    }

    if (cfg->en_mask & SENSOR_TYPE_MAGNETIC_FIELD) {
        err = bmx160_config_mag(bmx160, cfg);
        assert(!err);
    }

    if (cfg->fifo_enable) {
        err = bmx160_config_fifo(bmx160, cfg);
        assert(!err);
        sensor->s_sts.st_cputime = os_cputime_get32();
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

    err = sensor_set_type_mask(sensor, cfg->en_mask);
    assert(!err);

    memcpy(&bmx160->cfg, cfg, sizeof(bmx160->cfg));

    /* Interrupt */
    hal_gpio_irq_init(cfg->int1_pin, bmx160_irq1, &bmx160->sensor,
                      HAL_GPIO_TRIG_RISING, HAL_GPIO_PULL_NONE);
    hal_gpio_irq_enable(cfg->int1_pin);

    return 0;
}

static int g_open_cnt = 0;
int
bmx160_open(struct os_dev *dev, uint32_t t, void *a)
{
    //struct bmx160 *bmx160 = (struct bmx160 *)dev;
    //printf("%s:%d %p %d\n", __func__, __LINE__, bmx160, g_open_cnt);
    g_open_cnt++;
    return 0;
}

int
bmx160_suspend(struct os_dev *dev, os_time_t t, int a)
{
    uint8_t val, err;
    struct bmx160 *bmx160 = (struct bmx160 *)dev;
    printf("%s:%d %p\n", __func__, __LINE__, bmx160);
    /* Sleep sensors */
    val = BMX160_CMD_PMU_MODE_ACC_SUSPEND;
    err = bmx160_reg_write(bmx160, BMX160_REG_CMD, &val, 1);
    assert(!err);
    val = BMX160_CMD_PMU_MODE_GYR_SUSPEND;
    err = bmx160_reg_write(bmx160, BMX160_REG_CMD, &val, 1);
    assert(!err);
    val = BMX160_CMD_PMU_MODE_MAG_SUSPEND;
    err = bmx160_reg_write(bmx160, BMX160_REG_CMD, &val, 1);
    assert(!err);
    return 0;
}

int
bmx160_resume(struct os_dev *dev)
{
    uint8_t val, err;
    struct bmx160 *bmx160 = (struct bmx160 *)dev;
    //printf("%s:%d %p\n", __func__, __LINE__, bmx160);
    val = bmx160->cfg.acc_mode;
    err = bmx160_reg_write(bmx160, BMX160_REG_CMD, &val, 1);
    assert(!err);
    val = bmx160->cfg.gyro_mode;
    err = bmx160_reg_write(bmx160, BMX160_REG_CMD, &val, 1);
    assert(!err);
    val = bmx160->cfg.mag_mode;
    err = bmx160_reg_write(bmx160, BMX160_REG_CMD, &val, 1);
    assert(!err);
    return 0;
}

int
bmx160_close(struct os_dev *dev)
{
    //struct bmx160 *bmx160 = (struct bmx160 *)dev;
    //printf("%s:%d %p %d\n", __func__, __LINE__, bmx160, g_open_cnt);
    g_open_cnt--;
    return 0;
}

int bmx160_init(struct os_dev *dev, void *arg)
{
    int err;
    err = log_register("bmx160", &_log, &log_console_handler, NULL, LOG_SYSLEVEL);
    assert(!err);

    struct bmx160 *bmx160 = (struct bmx160 *)dev;
    struct sensor *sensor = &bmx160->sensor;

    dev->od_handlers = (struct os_dev_handlers) {
        .od_open = bmx160_open,
        .od_suspend = bmx160_suspend,
        .od_resume = bmx160_resume,
        .od_close = bmx160_close,
    };

    err = sensor_init(sensor, dev);
    assert(!err);
    // TODO all sensor types
    err = sensor_set_driver(sensor,
            (SENSOR_TYPE_GYROSCOPE |
            SENSOR_TYPE_MAGNETIC_FIELD |
            SENSOR_TYPE_ACCELEROMETER |
            SENSOR_TYPE_TEMPERATURE),
            &bmx160_sensor_driver);
    assert(!err);

    err = sensor_set_interface(sensor, arg);
    assert(!err);

    err = sensor_mgr_register(sensor);
    assert(!err);

    return 0;
}
