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
#include "stats/stats.h"

#include "bmx160/bmx160.h"
#include "bmm150.h"

#include "bmx160_defs.h"

struct bmx160_regval {
    uint8_t reg_addr;
    uint8_t reg_val;
};


#ifndef ARRAY_LEN
#define ARRAY_LEN(array) (sizeof(array) / sizeof(array[0]))
#endif

#define BMX160_SENSOR_TYPES_MSK (\
        SENSOR_TYPE_ACCELEROMETER | \
        SENSOR_TYPE_TEMPERATURE | \
        SENSOR_TYPE_GYROSCOPE | \
        SENSOR_TYPE_MAGNETIC_FIELD) 

#define BMX160_ITF_LOCK_TIMEOUT (OS_TIMEOUT_NEVER)
#define BMX160_RW_TIMEOUT (OS_TICKS_PER_SEC / 10)



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

    BMX160_LOG_DEBUG("WR_REG:0x%02X=0x%02X\n", addr, bmx160->_txbuf[1]);

    if (addr == BMX160_REG_CMD) {
        assert(size == 1);
        uint8_t cmd_val = bmx160->_txbuf[1]; 
        unsigned int t_ms = bmx160_get_cmd_post_delay_ms(cmd_val);
        if (t_ms) 
            os_time_delay((OS_TICKS_PER_SEC * t_ms) / 1000 + 1);
    }

	return err;
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

        err = bmx160_reg_write(bmx160, BMX160_REG_MAG_IF_1_RDA, &bmm_addr, 1);
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


static int32_t bmm150_compensate_xy(const struct bmm150_trim_regs *tregs,
				  int16_t xy, uint16_t rhall, bool is_x)
{
	int8_t txy1, txy2;
	int16_t val;
	uint16_t prevalue;
	int32_t temp1, temp2, temp3;

	if (xy == BMM150_XY_OVERFLOW_VAL) {
		return INT32_MIN;
	}

	if (!rhall) {
		rhall = tregs->xyz1;
	}

	if (is_x) {
		txy1 = tregs->x1;
		txy2 = tregs->x2;
	} else {
		txy1 = tregs->y1;
		txy2 = tregs->y2;
	}

	prevalue = (uint16_t)((((int32_t)tregs->xyz1) << 14) / rhall);

	val = (int16_t)((prevalue) - ((uint16_t)0x4000));

	temp1 = (((int32_t)tregs->xy2) * ((((int32_t)val) * ((int32_t)val)) >> 7));

	temp2 = ((int32_t)val) * ((int32_t)(((int16_t)tregs->xy1) << 7));

	temp3 = (((((temp1 + temp2) >> 9) +
		((int32_t)0x100000)) * ((int32_t)(((int16_t)txy2) +
		((int16_t)0xA0)))) >> 12);

	val = ((int16_t)((((int32_t)xy) * temp3) >> 13)) + (((int16_t)txy1) << 3);

	return (int32_t)val;
}

static int32_t bmm150_compensate_z(const struct bmm150_trim_regs *tregs,
				 int16_t z, uint16_t rhall)
{
	int32_t val, temp1, temp2;
	int16_t temp3;

	if (z == BMM150_Z_OVERFLOW_VAL) {
		return INT32_MIN;
	}

	temp1 = (((int32_t)(z - tregs->z4)) << 15);

	temp2 = ((((int32_t)tregs->z3) *
		((int32_t)(((int16_t)rhall) - ((int16_t)tregs->xyz1)))) >> 2);

	temp3 = ((int16_t)(((((int32_t)tregs->z1) *
		((((int16_t)rhall) << 1))) + (1 << 15)) >> 16));

	val = ((temp1 - temp2) / (tregs->z2 + temp3));

	return val;
}

static int bmx160_fetch_mag(struct bmx160 *bmx160, struct sensor_mag_data *smd)
{
	int err;

    uint8_t *buf = bmx160->_rxbuf; // todo check allignment
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
	smd->smd_x = bmm150_compensate_xy(&bmx160->_trim_regs,
							raw_x, rhall, true);
    smd->smd_y_is_valid = 1;
    smd->smd_y = bmm150_compensate_xy(&bmx160->_trim_regs,
							raw_y, rhall, false);
    smd->smd_z_is_valid = 1;
    smd->smd_z = bmm150_compensate_z(&bmx160->_trim_regs,
							raw_z, rhall);

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
#if 0
        uint8_t *tmp = bmx160->_rxbuf;
        err = bmx160_reg_read(bmx160, BMX160_REG_MAG_DATA, tmp, 6);
		if (err)
			return err;

        struct sensor_mag_data smd = {
            .smd_x_is_valid = 1,
            .smd_y_is_valid = 1,
            .smd_z_is_valid = 1
        };

        float tounit = BMX160_SI_UNIT_FACT_MAG;
        bmx160_unpack_s16xyz(tmp, &smd.smd_x, &smd.smd_y, &smd.smd_z, tounit);
#else
        struct sensor_mag_data smd = { 0 };
        err = bmx160_fetch_mag(bmx160, &smd);
        if (err)
            return err;
#endif
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
        {BMX160_REG_ACC_CONF, BMX160_ACC_CONF_BWP_NORMAL_AVG4 | 
                              BMX160_ACC_CONF_ODR_100HZ},
        {BMX160_REG_ACC_RANGE, BMX160_ACC_RANGE_2G},
        {BMX160_REG_CMD, BMX160_CMD_PMU_MODE_ACC_NORMAL}
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
        {BMX160_REG_GYR_CONF, BMX160_GYR_CONF_BWP_NORMAL | 
                              BMX160_GYR_CONF_ODR_100HZ},
        {BMX160_REG_GYR_RANGE, BMX160_GYR_RANGE_2000_DPS},
        {BMX160_REG_CMD, BMX160_CMD_PMU_MODE_GYR_NORMAL}
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
    uint8_t regval;

    regval = BMX160_CMD_PMU_MODE_MAG_NORMAL;
    err = bmx160_reg_write(bmx160, BMX160_REG_CMD, &regval, 1);
    assert(!err);

    err = bmx160_bmm_reg_read(bmx160, BMM150_REG_TRIM_START, &bmx160->_trim_regs, sizeof(bmx160->_trim_regs));
    assert(!err);

    static const struct bmx160_regval bmm_regs[] = {
        // reg 0x4B = 0b0000001 --> power_control=1 (default)
        {BMM150_REG_POWER, 0x01},
        // reg 0x51 = 0x01 --> REPXY=1
        {BMM150_REG_REP_XY, 0x01},
        {BMM150_REG_REP_Z, 0x0E},
        // reg 0x4C = (1 << 1) --> Opmode = forced mode
        {BMM150_REG_OPMODE_ODR, 0x02}
    };
    for (int i = 0; i < ARRAY_LEN(bmm_regs); i++) {
        err = bmx160_bmm_reg_write(bmx160, bmm_regs[i].reg_addr, &bmm_regs[i].reg_val, 1);
        assert(!err);
    }

    // unclear if/why this read needed. used in manual example
    err = bmx160_bmm_reg_read(bmx160, BMM150_REG_X_L, &regval, 1);
    assert(!err);
    (void)regval;

    // enable "Data mode" 
    regval = 0; //BMX160_MAG_IF_0_CFG_RD_BURST_6
    err = bmx160_reg_write(bmx160, BMX160_REG_MAG_IF_0_CFG, &regval, 1);
    assert(!err);

    regval = BMX160_MAG_CONF_ODR_100HZ;
    err = bmx160_reg_write(bmx160, BMX160_REG_MAG_CONF, &regval, 1);
    assert(!err);

    return 0;
}


int bmx160_config(struct bmx160 *bmx160, const struct bmx160_cfg *cfg)
{
    (void) bmx160_sd_set_config;
    int err = 0;

    // TODO en_mask from config
    sensor_type_t en_mask = (SENSOR_TYPE_GYROSCOPE |
            SENSOR_TYPE_MAGNETIC_FIELD |
            SENSOR_TYPE_ACCELEROMETER); 

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

    err = sensor_set_type_mask(sensor, en_mask);
    assert(!err);

    return 0;
}

int bmx160_init(struct os_dev *dev, void *arg)
{
    int err;
    err = log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);
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
