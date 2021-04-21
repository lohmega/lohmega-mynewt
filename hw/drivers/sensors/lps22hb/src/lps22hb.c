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
#include <stdio.h>

#include "defs/error.h"
#include "os/os.h"
#include "sysinit/sysinit.h"
#include "hal/hal_i2c.h"
#include "sensor/sensor.h"
#include "sensor/pressure.h"
#include "sensor/temperature.h"
#include "lps22hb/lps22hb.h"
#include "lps22hb_priv.h"
#include "log/log.h"
#include <stats/stats.h>

#if MYNEWT_VAL(LPS22HB_STATS_ENABLE)
STATS_SECT_START(lps22hb_stats)
    STATS_SECT_ENTRY(read_errors)
    STATS_SECT_ENTRY(write_errors)
    STATS_SECT_ENTRY(mutex_errors)
STATS_SECT_END

/* Global variable used to hold stats data */
STATS_SECT_DECL(lps22hb_stats) g_lps22hb_stats;

/* Define the stats section and records */
STATS_NAME_START(lps22hb_stats)
    STATS_NAME(lps22hb_stats, read_errors)
    STATS_NAME(lps22hb_stats, write_errors)
    STATS_NAME(lps22hb_stats, mutex_errors)
STATS_NAME_END(lps22hb_stats)

#define LPS22HB_STATS_INC(__X) STATS_INC(g_lps22hb_stats, __X)
#define LPS22HB_STATS_INCN(__X,__N) STATS_INCN(g_lps22hb_stats, __X, __N)
#else
#define LPS22HB_STATS_INC(__X) {}
#define LPS22HB_STATS_INCN(__X,__N) {}
#endif


#define LOG_MODULE_LPS22HB    (82)
#define LPS22HB_INFO(...)     LOG_INFO(&_log, LOG_MODULE_LPS22HB, __VA_ARGS__)
#define LPS22HB_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_LPS22HB, __VA_ARGS__)
static struct log _log;

/* Exports for the sensor API */
static int lps22hb_sensor_read(struct sensor *, sensor_type_t,
        sensor_data_func_t, void *, uint32_t);
static int lps22hb_sensor_get_config(struct sensor *, sensor_type_t,
        struct sensor_cfg *);

static const struct sensor_driver g_lps22hb_sensor_driver = {
    lps22hb_sensor_read,
    lps22hb_sensor_get_config
};

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
static void
lps22hb_init_node_cb(struct bus_node *bnode, void *arg)
{
    struct sensor_itf *itf = arg;
    lps22hb_init((struct os_dev *)bnode, itf);
}

static struct bus_node_callbacks lps22hb_bus_node_cbs = {
   .init = lps22hb_init_node_cb,
};

int
lps22hb_create_i2c_sensor_dev(struct bus_i2c_node *node, const char *name,
                              const struct bus_i2c_node_cfg *i2c_cfg,
                              struct sensor_itf *sensor_itf)
{
    sensor_itf->si_dev = &node->bnode.odev;
    bus_node_set_callbacks((struct os_dev *)node, &lps22hb_bus_node_cbs);
    return bus_i2c_node_create(name, node, i2c_cfg, sensor_itf);
}
#endif
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
lps22hb_write8(struct lps22hb *dev, uint8_t reg, uint32_t value)
{
    int rc;
    struct sensor_itf *itf = SENSOR_GET_ITF(&dev->sensor);
    uint8_t payload[2] = { reg, value & 0xFF };

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    rc = bus_node_simple_write(itf->si_dev, payload, sizeof(payload));
#else
    os_error_t err = 0;

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 2,
        .buffer = payload
    };

    if (dev->i2c_mutex) {
        err = os_mutex_pend(dev->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK)
        {
            LPS22HB_STATS_INC(mutex_errors);
            return err;
        }
    }

    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
        LPS22HB_STATS_INC(write_errors);
    }

    if (dev->i2c_mutex) {
        err = os_mutex_release(dev->i2c_mutex);
        assert(err == OS_OK);
    }
#endif
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
lps22hb_read8(struct lps22hb *dev, uint8_t reg, uint8_t *value)
{
    int rc;
    struct sensor_itf *itf = SENSOR_GET_ITF(&dev->sensor);

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    rc = bus_node_simple_write_read_transact(itf->si_dev, &reg, 1, value, 1);

#else
    os_error_t err = 0;
    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &reg
    };

    if (dev->i2c_mutex)
    {
        err = os_mutex_pend(dev->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK)
        {
            LPS22HB_ERR("Mutex error=%d\n", err);
            LPS22HB_STATS_INC(mutex_errors);
            return err;
        }
    }

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        LPS22HB_STATS_INC(write_errors);
        goto exit;
    }

    /* Read one byte back */
    data_struct.buffer = value;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
         LPS22HB_STATS_INC(read_errors);
    }

exit:
    if (dev->i2c_mutex) {
        err = os_mutex_release(dev->i2c_mutex);
        assert(err == OS_OK);
    }
#endif

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
lps22hb_read_bytes(struct lps22hb *dev, uint8_t reg, uint8_t *buffer, uint32_t length)
{
    int rc;
    struct sensor_itf *itf = SENSOR_GET_ITF(&dev->sensor);

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    rc = bus_node_simple_write_read_transact(itf->si_dev, &reg, 1, buffer, length);
#else
    os_error_t err = 0;
    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &reg
    };

    if (dev->i2c_mutex)
    {
        err = os_mutex_pend(dev->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK)
        {
            LPS22HB_STATS_INC(mutex_errors);
            return err;
        }
    }

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        LPS22HB_STATS_INC(write_errors);
        goto exit;
    }

    /* Read n bytes back */
    data_struct.len = length;
    data_struct.buffer = buffer;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
         LPS22HB_STATS_INC(read_errors);
    }

exit:
    if (dev->i2c_mutex)
    {
        err = os_mutex_release(dev->i2c_mutex);
        assert(err == OS_OK);
    }
#endif
    return rc;
}

static int 
lps22hb_update_byte(struct lps22hb *dev, uint8_t reg_addr, uint8_t mask,
        uint8_t value)
{

    int rc;
    uint8_t reg = 0;
    rc = lps22hb_read8(dev, reg_addr, &reg);
    if (rc) {
        return rc;
    }
    reg &= ~mask;
    reg |= (uint8_t)value & mask;
    return lps22hb_write8(dev, reg_addr, reg);
}


int
lps22hb_reset(struct lps22hb *dev)
{
    int rc;
    uint8_t reg;
    rc = lps22hb_read8(dev, LPS22HB_CTRL_REG2, &reg);
    if (rc) {
        return rc;
    }

    // Reset
    return lps22hb_write8(dev, LPS22HB_CTRL_REG2, reg | LPS22HB_REG2_RESET);
}

int
lps22hb_set_output_rate(struct lps22hb *dev, enum lps22hb_output_rate rate)
{
    int rc;
    uint8_t reg;
    // 0x80 - temperature compensation, continuous mode (bits 0:1 == 00)
    rc = lps22hb_read8(dev, LPS22HB_CTRL_REG1, &reg);
    if (rc) {
        return rc;
    }
#if 1
    if (rate != LPS22HB_OUTPUT_RATE_ONESHOT) {

        reg |=  LPS22HB_CTRL_REG2_FIFO_EN_MASK;
        rc = lps22hb_write8(dev, LPS22HB_CTRL_REG2, 0x50);
        if (rc) {
            return rc;
        }

        // write 0 to flush fifo
        rc = lps22hb_write8(dev, LPS22HB_FIFO_CTRL, 0x00);
        if (rc) {
            return rc;
        }

        // enable fifo stream mode
        rc = lps22hb_write8(dev, LPS22HB_FIFO_CTRL, 0x40);
        if (rc) {
            return rc;
        }
    }
#endif
    reg &= ~0xF;
    reg |= (uint8_t)rate & 0xF;
    return lps22hb_write8(dev, LPS22HB_CTRL_REG1, reg);
}

int
lps22hb_get_output_rate(struct lps22hb *dev, enum lps22hb_output_rate *rate)
{
    int rc;
    uint8_t reg;

    rc = lps22hb_read8(dev, LPS22HB_CTRL_REG1, &reg);
    if (rc) {
        return rc;
    }

    *rate  = (enum lps22hb_output_rate)(reg & 0x70);

    return 0;
}


int
lps22hb_enable_interrupt(struct lps22hb *dev, uint8_t enable)
{
    int rc;
    uint8_t reg;

    rc = lps22hb_read8(dev, LPS22HB_CTRL_REG3, &reg);
    if (rc) {
        return rc;
    }

    reg = (reg & (~LPS22HB_REG3_DRDY)) | ((enable) ? LPS22HB_REG3_DRDY : 0x00);
    return lps22hb_write8(dev, LPS22HB_CTRL_REG3, reg);
}


int
lps22hb_set_lpf(struct lps22hb *dev, enum lps22hb_lpf_config cfg)
{
    return lps22hb_update_byte(dev, LPS22HB_CTRL_REG1, 0x0C, cfg);
}

int
lps22hb_get_lpf(struct lps22hb *dev, uint8_t *cfg)
{
    int rc;
    uint8_t reg;

    rc = lps22hb_read8(dev, LPS22HB_CTRL_REG1, &reg);
    if (rc) {
        return rc;
    }

    *cfg  = (enum lps22hb_output_rate)(reg & 0x0C);

    return 0;
}

int
lps22hb_oneshot(struct lps22hb *dev)
{
    int rc;
    uint8_t reg;
    rc = lps22hb_read8(dev, LPS22HB_CTRL_REG2, &reg);
    if (rc) {
        return rc;
    }
    // Order a single measurement
    return lps22hb_write8(dev, LPS22HB_CTRL_REG2, reg | LPS22HB_REG2_ONESHOT);
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
lps22hb_init(struct os_dev *dev, void *arg)
{
    struct lps22hb *lhb;
    struct sensor *sensor;
    int rc;

    if (!arg || !dev) {
        return SYS_ENODEV;
    }

    lhb = (struct lps22hb *) dev;

    lhb->cfg.mask = SENSOR_TYPE_ALL;

    log_register((char*)dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    sensor = &lhb->sensor;

    rc = sensor_init(sensor, dev);
    if (rc) {
        return rc;
    }

    /* Add the accelerometer/gyroscope driver */
    rc = sensor_set_driver(sensor, (SENSOR_TYPE_PRESSURE | SENSOR_TYPE_TEMPERATURE),
         (struct sensor_driver *) &g_lps22hb_sensor_driver);
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
lps22hb_config(struct lps22hb *lhb, struct lps22hb_cfg *cfg)
{
    int rc;
    uint8_t val;
    rc = lps22hb_reset(lhb);
    if (rc) {
        return rc;
    }

    if (cfg) {
        memcpy(&lhb->cfg, cfg, sizeof(struct lps22hb_cfg));
    }

#if MYNEWT_VAL(LPS22HB_STATS_ENABLE)
    /* Init stats */
    rc = stats_init_and_reg(
        STATS_HDR(g_lps22hb_stats), STATS_SIZE_INIT_PARMS(g_lps22hb_stats,
        STATS_SIZE_32), STATS_NAME_INIT_PARMS(lps22hb_stats), "sen_lps22hb");
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif

    rc = lps22hb_read8(lhb, LPS22HB_WHO_AM_I, &val);
    if (rc) {
        return rc;
    }
    if ((val&LPS22HB_WHO_AM_I_MASK) != LPS22HB_WHO_AM_I_VAL) {
        return SYS_EINVAL;
    }

    rc = lps22hb_set_output_rate(lhb, lhb->cfg.output_rate);
    if (rc) {
        return rc;
    }

    rc = lps22hb_set_lpf(lhb, lhb->cfg.lpf_cfg);
    if (rc) {
        return rc;
    }

    // enable block data read (BDU) (bit 1 == 1)
    rc = lps22hb_update_byte(lhb, LPS22HB_CTRL_REG1, 0x02, 0x02);
    if (rc) {
        return rc;
    }

    rc = lps22hb_enable_interrupt(lhb, lhb->cfg.int_enable);
    if (rc) {
        return rc;
    }

    rc = sensor_set_type_mask(&(lhb->sensor), lhb->cfg.mask);
    if (rc) {
        return rc;
    }

    return 0;
}

int
lps22hb_read_raw(struct lps22hb *dev, uint32_t *pressure)
{
    int rc, timeout;
    uint8_t payload[3], reg;

    /* If sensor is in ONE_SHOT mode, activate it now */
    if (dev->cfg.output_rate == LPS22HB_OUTPUT_RATE_ONESHOT)
    {
        /* Get a new sample */
        rc = lps22hb_oneshot(dev);
        reg = LPS22HB_REG2_ONESHOT;
        timeout = 100;
        while (reg & LPS22HB_REG2_ONESHOT && --timeout)
        {
            os_cputime_delay_usecs(1000);
            lps22hb_read8(dev, LPS22HB_CTRL_REG2, &reg);
        }
        if (timeout==0) return OS_TIMEOUT;
    }

    if (dev->cfg.output_rate == LPS22HB_OUTPUT_RATE_ONESHOT)
    {
        rc  = lps22hb_read8(dev, LPS22HB_PRESS_OUT_XL, payload);
        rc |= lps22hb_read8(dev, LPS22HB_PRESS_OUT_L, payload+1);
        rc |= lps22hb_read8(dev, LPS22HB_PRESS_OUT_H, payload+2);
    }
    else
    {
        // bit 7 must be one to read multiple bytes
        rc = lps22hb_read_bytes(dev, (LPS22HB_PRESS_OUT_XL | 0x80), payload, 3);
    }

    /* Check if reading sensor failed */
    if (rc) {
        return rc;
    }

    if (pressure)
    {
        *pressure = ((uint32_t)payload[2]<<16) | ((uint32_t)payload[1]<<8) | payload[0];
    }

    return 0;
}

static int
lps22hb_sensor_read(struct sensor *sensor, sensor_type_t type,
        sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
    (void)timeout;
    int rc;
    int32_t temperature;
    uint32_t pressure;
    uint8_t payload[3];
    struct lps22hb *lhb;
    union {
        struct sensor_press_data spd;
        struct sensor_temp_data std;
    } databuf;
    uint8_t reg;

    /* If the read isn't looking for accel or gyro, don't do anything. */
    if (!(type & (SENSOR_TYPE_PRESSURE | SENSOR_TYPE_TEMPERATURE))) {
        return SYS_EINVAL;
    }

    lhb = (struct lps22hb *) SENSOR_GET_DEVICE(sensor);

    /* If sensor is in ONE_SHOT mode, activate it now */
    if (lhb->cfg.output_rate == LPS22HB_OUTPUT_RATE_ONESHOT)
    {
        /* Get a new sample */
        lps22hb_oneshot(lhb);
        reg = LPS22HB_REG2_ONESHOT;
        timeout = 100;
        while (reg & LPS22HB_REG2_ONESHOT && --timeout)
        {
            os_cputime_delay_usecs(1000);
            lps22hb_read8(lhb, LPS22HB_CTRL_REG2, &reg);
        }
        if (timeout==0) return OS_TIMEOUT;
    }

    if (type & SENSOR_TYPE_PRESSURE) {
        // bit 7 must be one to read multiple bytes
        rc = lps22hb_read_bytes(lhb, (LPS22HB_PRESS_OUT_XL | 0x80), payload, 3);
        if (rc) {
            return rc;
        }
        pressure = ((uint32_t)payload[2]<<16) | ((uint32_t)payload[1]<<8) | payload[0];

        /* Data in Pa */ 
        databuf.spd.spd_press = pressure *100/4096.0F;
        databuf.spd.spd_press_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.spd,
             SENSOR_TYPE_PRESSURE);
        if (rc) {
            return rc;
        }
    }

    if (type & SENSOR_TYPE_TEMPERATURE) {
        // bit 7 must be one to read multiple bytes
        rc = lps22hb_read_bytes(lhb, (LPS22HB_TEMP_OUT_L | 0x80), payload, 2);
        if (rc) {
            return rc;
        }
        temperature = ((int32_t)payload[1]<<8) | (int32_t)payload[0];

        /* Data in C */
        databuf.std.std_temp = temperature/100.0F;
        databuf.std.std_temp_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.std,
             SENSOR_TYPE_TEMPERATURE);
        if (rc) {
            return rc;
        }
    }

    return 0;
}

static int
lps22hb_sensor_get_config(struct sensor *sensor, sensor_type_t type,
        struct sensor_cfg *cfg)
{
    /* If the read isn't looking for accel or gyro, don't do anything. */
    if (!(type & (SENSOR_TYPE_PRESSURE|SENSOR_TYPE_TEMPERATURE))) {
        return SYS_EINVAL;
    }

    cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT;

    return 0;
}
