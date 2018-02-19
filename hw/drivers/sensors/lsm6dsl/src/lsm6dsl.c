/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * resarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <string.h>
#include <errno.h>
#include <assert.h>

#include "defs/error.h"
#include "os/os.h"
#include "sysinit/sysinit.h"
#include "hal/hal_i2c.h"
#include "sensor/sensor.h"
#include "sensor/accel.h"
#include "sensor/gyro.h"
#include "lsm6dsl/lsm6dsl.h"
#include "lsm6dsl_priv.h"
#include "log/log.h"
#include "stats/stats.h"

/* Define the stats section and records */
STATS_SECT_START(lsm6dsl_stat_section)
    STATS_SECT_ENTRY(read_errors)
    STATS_SECT_ENTRY(write_errors)
STATS_SECT_END

/* Define stat names for querying */
STATS_NAME_START(lsm6dsl_stat_section)
    STATS_NAME(lsm6dsl_stat_section, read_errors)
    STATS_NAME(lsm6dsl_stat_section, write_errors)
STATS_NAME_END(lsm6dsl_stat_section)

/* Global variable used to hold stats data */
STATS_SECT_DECL(lsm6dsl_stat_section) g_lsm6dslstats;

#define LOG_MODULE_LSM6DSL    (6000)
#define LSM6DSL_INFO(...)     LOG_INFO(&_log, LOG_MODULE_LSM6DSL, __VA_ARGS__)
#define LSM6DSL_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_LSM6DSL, __VA_ARGS__)
static struct log _log;

/* Exports for the sensor API */
static int lsm6dsl_sensor_read(struct sensor *, sensor_type_t,
        sensor_data_func_t, void *, uint32_t);
static int lsm6dsl_sensor_get_config(struct sensor *, sensor_type_t,
        struct sensor_cfg *);

static const struct sensor_driver g_lsm6dsl_sensor_driver = {
    lsm6dsl_sensor_read,
    lsm6dsl_sensor_get_config
};

/**
 * Writes a single byte to the specified register
 *
 * @param The sensor interface
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lsm6dsl_write8(struct sensor_itf *itf, uint8_t reg, uint32_t value)
{
    int rc;
    uint8_t payload[2] = { reg, value & 0xFF };

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 2,
        .buffer = payload
    };

    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
        LSM6DSL_ERR("Failed to write to 0x%02X:0x%02X with value 0x%02X\n",
                    itf->si_addr, reg, value);
        STATS_INC(g_lsm6dslstats, read_errors);
    }

    return rc;
}

/**
 * Reads a single byte from the specified register
 *
 * @param The sensor interface
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lsm6dsl_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value)
{
    int rc;

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &reg
    };

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        LSM6DSL_ERR("I2C access failed at address 0x%02X\n", itf->si_addr);
        STATS_INC(g_lsm6dslstats, write_errors);
        return rc;
    }

    /* Read one byte back */
    data_struct.buffer = value;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
         LSM6DSL_ERR("Failed to read from 0x%02X:0x%02X\n", itf->si_addr, reg);
         STATS_INC(g_lsm6dslstats, read_errors);
    }
    return rc;
}

/**
 * Reads n bytes from the specified register
 *
 * @param The sensor interface
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 * @param number of bytes to read
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lsm6dsl_read_bytes(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer, uint32_t length)
{
    int rc;

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &reg
    };

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        LSM6DSL_ERR("I2C access failed at address 0x%02X\n", itf->si_addr);
        STATS_INC(g_lsm6dslstats, write_errors);
        return rc;
    }

    /* Read n bytes back */
    data_struct.len = length;
    data_struct.buffer = buffer;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
         LSM6DSL_ERR("Failed to read from 0x%02X:0x%02X\n", itf->si_addr, reg);
         STATS_INC(g_lsm6dslstats, read_errors);
    }
    return rc;
}

int
lsm6dsl_reset(struct sensor_itf *itf)
{
    uint8_t temp;
    lsm6dsl_read8(itf, LSM6DSL_CTRL3_C, &temp);
    temp |= 0x01;
    int rc = lsm6dsl_write8(itf, LSM6DSL_CTRL3_C, temp);
    os_cputime_delay_usecs(10000);  // Wait for all registers to reset 
    return rc;
}

int
lsm6dsl_sleep(struct sensor_itf *itf)
{
    int rc;

    rc = lsm6dsl_write8(itf, LSM6DSL_CTRL1_XL, LSM6DSL_ACCEL_RATE_POWER_DOWN); 
    if (rc) {
        return rc;
    }

    rc = lsm6dsl_write8(itf, LSM6DSL_CTRL2_G, LSM6DSL_GYRO_RATE_POWER_DOWN); 
    return rc;
}

int
lsm6dsl_set_gyro_rate_range(struct sensor_itf *itf, enum lsm6dsl_gyro_rate rate , enum lsm6dsl_gyro_range range)
{
    uint8_t val = (uint8_t)range | (uint8_t)rate; 
    return lsm6dsl_write8(itf, LSM6DSL_CTRL2_G, val);
}

int
lsm6dsl_get_gyro_rate_range(struct sensor_itf *itf, enum lsm6dsl_gyro_rate *rate, enum lsm6dsl_gyro_range *range)
{
    uint8_t reg;
    int rc;

    rc = lsm6dsl_read8(itf, LSM6DSL_CTRL2_G, &reg);
    if (rc) {
        return rc;
    }

    *rate  = (enum lsm6dsl_gyro_rate)(reg  & 0xF0);
    *range = (enum lsm6dsl_gyro_range)(reg & 0x0F);

    return 0;
}

int
lsm6dsl_set_accel_rate_range(struct sensor_itf *itf, enum lsm6dsl_accel_rate rate , enum lsm6dsl_accel_range range)
{
    uint8_t val = (uint8_t)range | (uint8_t)rate; 
    return lsm6dsl_write8(itf, LSM6DSL_CTRL1_XL, val);
}

int
lsm6dsl_get_accel_rate_range(struct sensor_itf *itf, enum lsm6dsl_accel_rate *rate, enum lsm6dsl_accel_range *range)
{
    uint8_t reg;
    int rc;

    rc = lsm6dsl_read8(itf, LSM6DSL_CTRL1_XL, &reg);
    if (rc) {
        return rc;
    }

    *rate  = (enum lsm6dsl_accel_rate)(reg  & 0xF0);
    *range = (enum lsm6dsl_accel_range)(reg & 0x0F);

    return 0;
}

int
lsm6dsl_enable_interrupt(struct sensor_itf *itf, uint8_t enable)
{
    int rc;
    rc = lsm6dsl_write8(itf, LSM6DSL_DRDY_PULSE_CFG, 0x80);
    if (rc) {
        return rc;
    }
    return lsm6dsl_write8(itf, LSM6DSL_INT1_CTRL, (enable)? 0x03 : 0x00);
}


int
lsm6dsl_set_lpf(struct sensor_itf *itf, uint8_t cfg)
{
    return lsm6dsl_write8(itf, LSM6DSL_CTRL8_XL, cfg);
}

int
lsm6dsl_get_lpf(struct sensor_itf *itf, uint8_t *cfg)
{
    return lsm6dsl_read8(itf, LSM6DSL_CTRL8_XL, cfg);
}


/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with this accellerometer
 * @param Argument passed to OS device init, unused
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lsm6dsl_init(struct os_dev *dev, void *arg)
{
    struct lsm6dsl *lsm;
    struct sensor *sensor;
    int rc;

    if (!arg || !dev) {
        return SYS_ENODEV;
    }
    
    lsm = (struct lsm6dsl *) dev;

    lsm->cfg.mask = SENSOR_TYPE_ALL;

    log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    sensor = &lsm->sensor;

    /* Initialise the stats entry */
    rc = stats_init(
        STATS_HDR(g_lsm6dslstats),
        STATS_SIZE_INIT_PARMS(g_lsm6dslstats, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(lsm6dsl_stat_section));
    SYSINIT_PANIC_ASSERT(rc == 0);
    /* Register the entry with the stats registry */
    rc = stats_register(dev->od_name, STATS_HDR(g_lsm6dslstats));
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = sensor_init(sensor, dev);
    if (rc) {
        return rc;
    }

    /* Add the accelerometer/gyroscope driver */
    rc = sensor_set_driver(sensor, SENSOR_TYPE_GYROSCOPE |
         SENSOR_TYPE_ACCELEROMETER,
         (struct sensor_driver *) &g_lsm6dsl_sensor_driver);
    if (rc) {
        return rc;
    }

    rc = sensor_set_interface(sensor, arg);
    if (rc) {
        return rc;
    }

    return sensor_mgr_register(sensor);
}

int
lsm6dsl_config(struct lsm6dsl *lsm, struct lsm6dsl_cfg *cfg)
{
    int rc;
    struct sensor_itf *itf;

    itf = SENSOR_GET_ITF(&(lsm->sensor));

    uint8_t val;
    rc = lsm6dsl_read8(itf, LSM6DSL_WHO_AM_I, &val);
    if (rc) {
        return rc;
    }
    if (val != LSM6DSL_WHO_AM_I_VAL) {
        return SYS_EINVAL;
    }

    rc = lsm6dsl_set_lpf(itf, cfg->lpf_cfg);
    if (rc) {
        return rc;
    }
    lsm->cfg.lpf_cfg = cfg->lpf_cfg;


    rc = lsm6dsl_set_accel_rate_range(itf, cfg->accel_rate, cfg->accel_range);
    if (rc) {
        return rc;
    }
    lsm6dsl_get_accel_rate_range(itf, &(lsm->cfg.accel_rate), &(lsm->cfg.accel_range));

    rc = lsm6dsl_set_gyro_rate_range(itf, cfg->gyro_rate, cfg->gyro_range);
    if (rc) {
        return rc;
    }
    lsm6dsl_get_gyro_rate_range(itf, &(lsm->cfg.gyro_rate), &(lsm->cfg.gyro_range));

    rc = lsm6dsl_read8(itf, LSM6DSL_CTRL3_C, &val);
    if (rc) {
        return rc;
    }

    /* enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1) */
    rc = lsm6dsl_write8(itf, LSM6DSL_CTRL3_C, val| 0x40 | 0x04); 
    if (rc) {
        return rc;
    }
    
    rc = lsm6dsl_enable_interrupt(itf, cfg->int_enable);
    if (rc) {
        return rc;
    }
    lsm->cfg.int_enable = cfg->int_enable;
        
    rc = sensor_set_type_mask(&(lsm->sensor), cfg->mask);
    if (rc) {
        return rc;
    }

    lsm->cfg.mask = cfg->mask;

    return 0;
}

static int
lsm6dsl_sensor_read(struct sensor *sensor, sensor_type_t type,
        sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
    (void)timeout;
    int rc;
    int16_t x, y, z;
    uint8_t payload[14];
    float lsb;
    struct sensor_itf *itf;
    struct lsm6dsl *lsm;
    union {
        struct sensor_accel_data sad;
        struct sensor_gyro_data sgd;
    } databuf;

    memset(payload, 0, sizeof(payload));
    
    /* If the read isn't looking for accel or gyro, don't do anything. */
    if (!(type & SENSOR_TYPE_ACCELEROMETER) &&
       (!(type & SENSOR_TYPE_GYROSCOPE))) {
        return SYS_EINVAL;
    }

    itf = SENSOR_GET_ITF(sensor);
    lsm = (struct lsm6dsl *) SENSOR_GET_DEVICE(sensor);

    if (type & (SENSOR_TYPE_ACCELEROMETER|SENSOR_TYPE_GYROSCOPE)) {
        rc = lsm6dsl_read_bytes(itf, LSM6DSL_OUT_TEMP_L, payload, 14);
        if (rc) {
            return rc;
        }
    }

    
    /* Get a new accelerometer sample */
    if (type & SENSOR_TYPE_ACCELEROMETER) {
        x = (((int16_t)payload[9] << 8) | payload[8]);
        y = (((int16_t)payload[11] << 8) | payload[10]);
        z = (((int16_t)payload[13] << 8) | payload[12]);

        switch (lsm->cfg.accel_range) {
            case LSM6DSL_ACCEL_RANGE_2: /* +/- 2g - 16384 LSB/g */
            /* Falls through */
            default:
                lsb = 16384.0F;
            break;
            case LSM6DSL_ACCEL_RANGE_4: /* +/- 4g - 8192 LSB/g */
                lsb = 8192.0F;
            break;
            case LSM6DSL_ACCEL_RANGE_8: /* +/- 8g - 4096 LSB/g */
                lsb = 4096.0F;
            break;
            case LSM6DSL_ACCEL_RANGE_16: /* +/- 16g - 2048 LSB/g */
                lsb = 2048.0F;
            break;
        }

        databuf.sad.sad_x = (x / lsb) * STANDARD_ACCEL_GRAVITY;
        databuf.sad.sad_x_is_valid = 1;
        databuf.sad.sad_y = (y / lsb) * STANDARD_ACCEL_GRAVITY;
        databuf.sad.sad_y_is_valid = 1;
        databuf.sad.sad_z = (z / lsb) * STANDARD_ACCEL_GRAVITY;
        databuf.sad.sad_z_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.sad,
                SENSOR_TYPE_ACCELEROMETER);
        if (rc) {
            return rc;
        }
    }

    /* Get a new gyroscope sample */
    if (type & SENSOR_TYPE_GYROSCOPE) {
        x = (int16_t)((payload[3] << 8) | payload[2]);
        y = (int16_t)((payload[5] << 8) | payload[4]);
        z = (int16_t)((payload[7] << 8) | payload[6]);

        switch (lsm->cfg.gyro_range) {
            case LSM6DSL_GYRO_RANGE_245: /* +/- 245 Deg/s - 133 LSB/Deg/s */
            /* Falls through */
            default:
                lsb = 131.0F;
            break;
            case LSM6DSL_GYRO_RANGE_500: /* +/- 500 Deg/s - 65.5 LSB/Deg/s */
                lsb = 65.5F;
            break;
            case LSM6DSL_GYRO_RANGE_1000: /* +/- 1000 Deg/s - 32.8 LSB/Deg/s */
                lsb = 32.8F;
            break;
            case LSM6DSL_GYRO_RANGE_2000: /* +/- 2000 Deg/s - 16.4 LSB/Deg/s */
                lsb = 16.4F;
            break;
        }

        databuf.sgd.sgd_x = x / lsb;
        databuf.sgd.sgd_x_is_valid = 1;
        databuf.sgd.sgd_y = y / lsb;
        databuf.sgd.sgd_y_is_valid = 1;
        databuf.sgd.sgd_z = z / lsb;
        databuf.sgd.sgd_z_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.sgd, SENSOR_TYPE_GYROSCOPE);
        if (rc) {
            return rc;
        }
    }

    return 0;
}

static int
lsm6dsl_sensor_get_config(struct sensor *sensor, sensor_type_t type,
        struct sensor_cfg *cfg)
{
    /* If the read isn't looking for accel or gyro, don't do anything. */
    if (!(type & SENSOR_TYPE_ACCELEROMETER) &&
       (!(type & SENSOR_TYPE_GYROSCOPE))) {
        return SYS_EINVAL;
    }

    cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT_TRIPLET;

    return 0;
}
