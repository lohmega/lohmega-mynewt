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
#include "log/log.h"
#include "stats/stats.h"

#include "bmx160/bmx160.h"
#include "bmi160.h"
#include "bmi160_defs.h"


// hack to adapt to adapt bosch api without modifications to original source. TODO fix
static struct bmx160 * g_bmx160_dev = NULL;

static inline struct bmi160_dev * bmx160_get_bmi160_dev(struct bmx160 * bmx160)
{
    _Static_assert(sizeof(bmx160->_priv) >= sizeof(struct bmi160_dev), "");

    return (struct bmi160_dev *) bmx160->_priv;
}

#define BMX160_SENSOR_TYPES_MSK (\
        SENSOR_TYPE_ACCELEROMETER | \
        SENSOR_TYPE_TEMPERATURE | \
        SENSOR_TYPE_GYROSCOPE | \
        SENSOR_TYPE_MAGNETIC_FIELD) 

#define BMX160_ITF_LOCK_TIMEOUT (OS_TIMEOUT_NEVER)
#define BMX160_RW_TIMEOUT (OS_TICKS_PER_SEC / 10)

#define LOG_MODULE_BMX160    (160)
#define BMX160_LOG_INFO(...)  LOG_INFO(&_log, LOG_MODULE_BMX160, __VA_ARGS__)
#define BMX160_LOG_ERROR(...)   LOG_ERROR(&_log, LOG_MODULE_BMX160, __VA_ARGS__)

static struct log _log;



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

static int
bmx160_reg_write_i2c(
        struct sensor_itf   *itf,
        uint8_t addr,
        uint8_t *data,
		size_t size)
{
    int err = 0;

    err = sensor_itf_lock(itf, BMX160_ITF_LOCK_TIMEOUT);
	if (err) {
		return err;
	}

    struct hal_i2c_master_data op = {
		.address = itf->si_addr,
		.len     = 1,
		.buffer  = &addr
	};

    err = hal_i2c_master_write(itf->si_num, &op, BMX160_RW_TIMEOUT, 0);
    if (err) {
        BMX160_LOG_ERROR("I2C write failed at address 0x%02X single byte\n",
                   addr);
        goto done;
    }

    op.len     = size;
    op.buffer  = data;

    err = hal_i2c_master_write(itf->si_num, &op, BMX160_RW_TIMEOUT, 1);
    if (err) {
        BMX160_LOG_ERROR("I2C write failed at address 0x%02X single byte\n",
                   addr);
		goto done;
    }

done:
    sensor_itf_unlock(itf);

    return err;
}

static int bmx160_reg_read(struct bmx160 * bmx160,
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


static int bmx160_reg_write(struct bmx160 * bmx160,
             uint8_t addr,
             uint8_t * data,
			size_t size)
{
    struct sensor_itf *itf = SENSOR_GET_ITF(&bmx160->sensor);
    switch(itf->si_type) {
        case SENSOR_ITF_I2C:
            return bmx160_reg_write_i2c(itf, addr, data, size);
        case SENSOR_ITF_SPI:
        default:
			return SYS_EINVAL;
    }

	return SYS_EINVAL;
}

static int bmx160_sd_set_config(struct sensor *sensor, void *cfg)
{
    struct bmx160 *bmx160 = (struct bmx160 *)SENSOR_GET_DEVICE(sensor);
    struct bmx160_cfg *bmx160_cfg = cfg;

    return bmx160_config(bmx160, bmx160_cfg);
}

static void bmx160_bosch_delay_ms(uint32_t n)
{
    os_time_delay((OS_TICKS_PER_SEC * n) / 1000 + 1);
}

static int8_t bmx160_bosch_read(uint8_t _id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return bmx160_reg_read(g_bmx160_dev, reg_addr, data, len);
}

static int8_t bmx160_bosch_write(uint8_t _id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return bmx160_reg_write(g_bmx160_dev, reg_addr, data, len);
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
    struct bmi160_dev *bmi160_dev = bmx160_get_bmi160_dev(bmx160);

    if (sensor_type & SENSOR_TYPE_ACCELEROMETER) {

        struct bmi160_sensor_data accel;
        err = bmi160_get_sensor_data(BMI160_ACCEL_ONLY, &accel, NULL, bmi160_dev);
        if (err)
            return err;

        struct sensor_accel_data sad;
        // TODO confirm scaling
        sad.sad_x = accel.x * STANDARD_ACCEL_GRAVITY;
        sad.sad_y = accel.y * STANDARD_ACCEL_GRAVITY;
        sad.sad_z = accel.z * STANDARD_ACCEL_GRAVITY;
        sad.sad_x_is_valid = 1;
        sad.sad_y_is_valid = 1;
        sad.sad_z_is_valid = 1;

        err = cb_func(sensor, cb_arg, &sad, SENSOR_TYPE_ACCELEROMETER);
        if (err)
            return err;
    }

    if (sensor_type & SENSOR_TYPE_GYROSCOPE) {

        struct bmi160_sensor_data gyro;
        err = bmi160_get_sensor_data(BMI160_GYRO_ONLY, NULL, &gyro, bmi160_dev);
        if (err)
            return err;

        struct sensor_gyro_data sgd;
        // TODO scaling?
        sgd.sgd_x = gyro.x;
        sgd.sgd_y = gyro.y;
        sgd.sgd_z = gyro.z;
        sgd.sgd_x_is_valid = 1;
        sgd.sgd_y_is_valid = 1;
        sgd.sgd_z_is_valid = 1;

        err = cb_func(sensor, cb_arg, &sgd, SENSOR_TYPE_GYROSCOPE);
		if (err)
			return err;
    }

    if (sensor_type & SENSOR_TYPE_MAGNETIC_FIELD) {
        return SYS_EINVAL; // TODO
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


int bmx160_config(struct bmx160 *bmx160, const struct bmx160_cfg *cfg)
{
    (void) bmx160_sd_set_config;
    int err = 0;

    struct sensor *sensor = &bmx160->sensor;
    struct bmi160_dev *bmi160_dev = bmx160_get_bmi160_dev(bmx160);
    bmi160_dev->id = -1; // TODO not used see above
    bmi160_dev->interface = BMI160_I2C_INTF;
    bmi160_dev->read = bmx160_bosch_read;
    bmi160_dev->write = bmx160_bosch_write;
    bmi160_dev->delay_ms = bmx160_bosch_delay_ms;

    _Static_assert(BMI160_OK == 0, "");

    err = bmi160_init(bmi160_dev);
    assert(!err);

    bmi160_dev->accel_cfg.odr = BMI160_ACCEL_ODR_3_12HZ;
    bmi160_dev->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi160_dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    bmi160_dev->accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
    bmi160_dev->gyro_cfg.odr = BMI160_GYRO_ODR_25HZ;
    bmi160_dev->gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
    bmi160_dev->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    bmi160_dev->gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;

    err = bmi160_set_sens_conf(bmi160_dev);
    assert(!err);
    
    // sanity check on magic chip_id
    uint8_t chip_id = 0;
    err = bmx160_reg_read(bmx160, BMI160_CHIP_ID_ADDR, &chip_id, 1);
    assert(!err);
    assert(chip_id == 0xD8);

    sensor_type_t mask = (SENSOR_TYPE_GYROSCOPE |
            SENSOR_TYPE_ACCELEROMETER); // TODO from config
    err = sensor_set_type_mask(sensor, mask);
    assert(!err);

    return 0;
}

int bmx160_init(struct os_dev *dev, void *arg)
{
    int err;
    err = log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);
    assert(!err);
    
    struct bmx160 *bmx160 = (struct bmx160 *)dev;

    g_bmx160_dev = bmx160; // temp hack. TODO
    struct sensor *sensor = &bmx160->sensor;

    err = sensor_init(sensor, dev);
    assert(!err);

    err = sensor_set_driver(sensor,
            (SENSOR_TYPE_GYROSCOPE |
            SENSOR_TYPE_ACCELEROMETER), // TODO all sensor types
                           &bmx160_sensor_driver);
    assert(!err);

    err = sensor_set_interface(sensor, arg);
    assert(!err);

    //sensor->s_next_run = OS_TIMEOUT_NEVER;

    err = sensor_mgr_register(sensor);
    assert(!err);

    return 0;
}
