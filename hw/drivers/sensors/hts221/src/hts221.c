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
#include "os/os_mutex.h"
#include "sysinit/sysinit.h"
#include "hal/hal_i2c.h"
#include "sensor/sensor.h"
#include "sensor/humidity.h"
#include "sensor/temperature.h"
#include "hts221/hts221.h"
#include "hts221_priv.h"
#include "log/log.h"
#include <stats/stats.h>

STATS_SECT_START(hts221_stats)
    STATS_SECT_ENTRY(read_errors)
    STATS_SECT_ENTRY(write_errors)
    STATS_SECT_ENTRY(mutex_errors)
STATS_SECT_END

/* Global variable used to hold stats data */
STATS_SECT_DECL(hts221_stats) g_hts221_stats;

/* Define the stats section and records */
STATS_NAME_START(hts221_stats)
    STATS_NAME(hts221_stats, read_errors)
    STATS_NAME(hts221_stats, write_errors)
    STATS_NAME(hts221_stats, mutex_errors)
STATS_NAME_END(hts221_stats)


#define LOG_MODULE_HTS221    (221)
#define HTS221_INFO(...)     LOG_INFO(&_log, LOG_MODULE_HTS221, __VA_ARGS__)
#define HTS221_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_HTS221, __VA_ARGS__)
static struct log _log;

/* Exports for the sensor API */
static int hts221_sensor_read(struct sensor *, sensor_type_t,
        sensor_data_func_t, void *, uint32_t);
static int hts221_sensor_get_config(struct sensor *, sensor_type_t,
        struct sensor_cfg *);

static const struct sensor_driver g_hts221_sensor_driver = {
    hts221_sensor_read,
    hts221_sensor_get_config
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
hts221_write8(struct hts221 *dev, uint8_t reg, uint32_t value)
{
    int rc;
    os_error_t err = 0;
    struct sensor_itf *itf = &dev->sensor.s_itf;
    uint8_t payload[2] = { reg, value & 0xFF };

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 2,
        .buffer = payload
    };

    if (dev->i2c_mutex)
    {
        err = os_mutex_pend(dev->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK)
        {
            HTS221_ERR("Mutex error=%d\n", err);
            STATS_INC(g_hts221_stats, mutex_errors);
            return err;
        }
    }

    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
        HTS221_ERR("Failed to write to 0x%02X:0x%02X with value 0x%02X\n",
                       itf->si_addr, reg, value);
        STATS_INC(g_hts221_stats, write_errors);
    }

    if (dev->i2c_mutex)
    {
        err = os_mutex_release(dev->i2c_mutex);
        assert(err == OS_OK);
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
hts221_read8(struct hts221 *dev, uint8_t reg, uint8_t *value)
{
    int rc;
    os_error_t err = 0;
    struct sensor_itf *itf = &dev->sensor.s_itf;

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
            HTS221_ERR("Mutex error=%d\n", err);
            STATS_INC(g_hts221_stats, mutex_errors);
            return err;
        }
    }

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        HTS221_ERR("I2C access failed at address 0x%02X\n", itf->si_addr);
        STATS_INC(g_hts221_stats, write_errors);
        goto exit;
    }

    /* Read one byte back */
    data_struct.buffer = value;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
         HTS221_ERR("Failed to read from 0x%02X:0x%02X\n", itf->si_addr, reg);
         STATS_INC(g_hts221_stats, read_errors);
    }

    exit:
    if (dev->i2c_mutex)
    {
        err = os_mutex_release(dev->i2c_mutex);
        assert(err == OS_OK);
    }

    return rc;
}



int
hts221_enable_interrupt(struct hts221 *dev, uint8_t enable)
{
    int rc;
    uint8_t reg;

    rc = hts221_read8(dev, HTS221_CTRL_REG3, &reg);
    if (rc) {
        return rc;
    }

    reg = (reg & (~HTS221_CTRL_REG3_DRDY_EN_MASK)) | ((enable) ? HTS221_CTRL_REG3_DRDY_EN_MASK : 0x00);
    return hts221_write8(dev, HTS221_CTRL_REG3, reg);
}


int
hts221_start_conv(struct hts221 *dev)
{
    int rc;
    rc = hts221_write8(dev, HTS221_CTRL_REG1, HTS221_CTRL_REG1_PD_MASK);
    if (rc) {
        return rc;
    }
    return hts221_write8(dev, HTS221_CTRL_REG2, HTS221_CTRL_REG2_OS_MASK);
}

int
hts221_read_calibration(struct hts221 *dev, hts221_cal_t *cal)
{
    int rc;
    uint8_t lsb, msb;

    rc = hts221_read8(dev, HTS221_CAL_H0RHX2, &(cal->H0_rH_x2));
    if (rc) {
        return rc;
    }    

    rc = hts221_read8(dev, HTS221_CAL_H1RHX2, &(cal->H1_rH_x2));
    if (rc) {
        return rc;
    }    

    rc = hts221_read8(dev, HTS221_CAL_T1T0MSB, &msb);
    if (rc) {
        return rc;
    }    

    rc = hts221_read8(dev, HTS221_CAL_T0DEGCX8, &lsb);
    if (rc) {
        return rc;
    }    
    cal->T0_decC_x8 = (((uint16_t)msb & 0x3)<<8) | lsb;

    rc = hts221_read8(dev, HTS221_CAL_T1DEGCX8, &lsb);
    if (rc) {
        return rc;
    }    
    cal->T1_decC_x8 = (((uint16_t)msb & 0xC)<<6) | lsb;


    rc = hts221_read8(dev, HTS221_CAL_H0T0OUT_L, &lsb);
    if (rc) {
        return rc;
    }    
    rc = hts221_read8(dev, HTS221_CAL_H0T0OUT_H, &msb);
    if (rc) {
        return rc;
    }    
    cal->H0_T0_out = ((int16_t)msb<<8) | lsb;

    rc = hts221_read8(dev, HTS221_CAL_H1T0OUT_L, &lsb);
    if (rc) {
        return rc;
    }    
    rc = hts221_read8(dev, HTS221_CAL_H1T0OUT_H, &msb);
    if (rc) {
        return rc;
    }    
    cal->H1_T0_out = ((int16_t)msb<<8) | lsb;
    
    rc = hts221_read8(dev, HTS221_CAL_T0OUT_L, &lsb);
    if (rc) {
        return rc;
    }    
    rc = hts221_read8(dev, HTS221_CAL_T0OUT_H, &msb);
    if (rc) {
        return rc;
    }    
    cal->T0_out = ((int16_t)msb<<8) | lsb;

    rc = hts221_read8(dev, HTS221_CAL_T1OUT_L, &lsb);
    if (rc) {
        return rc;
    }    
    rc = hts221_read8(dev, HTS221_CAL_T1OUT_H, &msb);
    if (rc) {
        return rc;
    }    
    cal->T1_out = ((int16_t)msb<<8) | lsb;

    return 0;
}

/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with this device
 * @param Argument passed to OS device init, unused
 *
 * @return 0 on success, non-zero error on failure.
 */
int
hts221_init(struct os_dev *dev, void *arg)
{
    struct hts221 *lhb;
    struct sensor *sensor;
    int rc;

    if (!arg || !dev) {
        return SYS_ENODEV;
    }
    
    lhb = (struct hts221 *) dev;

    lhb->cfg.mask = SENSOR_TYPE_ALL;

    log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    sensor = &lhb->sensor;

    rc = sensor_init(sensor, dev);
    if (rc) {
        return rc;
    }

    /* Add the humidity/temperature driver */
    rc = sensor_set_driver(sensor, (SENSOR_TYPE_RELATIVE_HUMIDITY | SENSOR_TYPE_TEMPERATURE),
         (struct sensor_driver *) &g_hts221_sensor_driver);
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
hts221_config(struct hts221 *lhb, struct hts221_cfg *cfg)
{
    int rc;
    uint8_t val;

    /* Init stats */
    rc = stats_init_and_reg(
        STATS_HDR(g_hts221_stats), STATS_SIZE_INIT_PARMS(g_hts221_stats,
        STATS_SIZE_32), STATS_NAME_INIT_PARMS(hts221_stats), "sen_hts221");
    SYSINIT_PANIC_ASSERT(rc == 0);
        
    rc = hts221_read8(lhb, HTS221_WHO_AM_I, &val);
    if (rc) {
        return rc;
    }
    if (val != HTS221_WHO_AM_I_VAL) {
        return SYS_EINVAL;
    }

    hts221_read_calibration(lhb, &(lhb->calibration));
    
    rc = hts221_enable_interrupt(lhb, cfg->int_enable);
    if (rc) {
        return rc;
    }
    lhb->cfg.int_enable = lhb->cfg.int_enable;
        
    rc = sensor_set_type_mask(&(lhb->sensor), cfg->mask);
    if (rc) {
        return rc;
    }

    lhb->cfg.mask = cfg->mask;

    return 0;
}

// returns relative humidity in procent x10
static int32_t hts221_calc_rh(hts221_cal_t *cal, int32_t h_out)
{
    int32_t kn = cal->H1_rH_x2 - cal->H0_rH_x2;
    int32_t kd = cal->H1_T0_out - cal->H0_T0_out;
    int32_t x = h_out-cal->H0_T0_out;
    return (((kn*x*10)/kd) + cal->H0_rH_x2*10)/2;
}

// returns temperature x 1000
static int32_t hts221_calc_temperature(hts221_cal_t *cal, int32_t t_out)
{
    int32_t kn = cal->T1_decC_x8 - cal->T0_decC_x8;
    int32_t kd = cal->T1_out - cal->T0_out;
    int32_t x = t_out - cal->T0_out;
    return (((kn*x*1000)/kd) + cal->T0_decC_x8*1000)/8;
}

int
hts221_read_raw(struct hts221 *dev, int32_t *temp, uint16_t *rh)
{
    int rc;
    uint8_t msb, lsb;
    int16_t raw;
    
    rc = hts221_read8(dev, HTS221_TEMP_OUT_H, &msb);
    rc = hts221_read8(dev, HTS221_TEMP_OUT_L, &lsb);
    
    if (rc) {
        return rc;
    }
    raw = ((int16_t)msb<<8) | lsb;

    /* Data in C*1000 instead of raw-raw because we need to use
     * calibration data */
    if (temp)
    {
        *temp = hts221_calc_temperature(&(dev->calibration), raw);
    }

    rc = hts221_read8(dev, HTS221_HUMIDITY_OUT_H, &msb);
    rc = hts221_read8(dev, HTS221_HUMIDITY_OUT_L, &lsb);

    if (rc) {
        return rc;
    }
    raw = ((int16_t)msb<<8) | lsb;

    /* Data in %rh*10 instead of raw-raw because we need to use
     * calibration data */
    if(rh)
    {
        *rh = hts221_calc_rh(&(dev->calibration), raw);
    }
        
    return 0;
}

static int
hts221_sensor_read(struct sensor *sensor, sensor_type_t type,
        sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
    (void)timeout;
    int rc;
    uint8_t lsb, msb;
    int16_t raw;
    struct hts221 *lhb;
    union {
        struct sensor_temp_data std;
        struct sensor_humid_data srhd;
    } databuf;

    /* If the read isn't looking for accel or gyro, don't do anything. */
    if (!(type & (SENSOR_TYPE_RELATIVE_HUMIDITY | SENSOR_TYPE_TEMPERATURE))) {
        return SYS_EINVAL;
    }

    lhb = (struct hts221 *) SENSOR_GET_DEVICE(sensor);

    /* Start conversion */
    hts221_start_conv(lhb);
    

    if (type & SENSOR_TYPE_TEMPERATURE) {
        rc = hts221_read8(lhb, HTS221_TEMP_OUT_H, &msb);
        rc = hts221_read8(lhb, HTS221_TEMP_OUT_L, &lsb);
            
        if (rc) {
            return rc;
        }
        raw = ((int16_t)msb<<8) | lsb;

        /* Data in C */ 
        databuf.std.std_temp = hts221_calc_temperature(&(lhb->calibration), raw);
        databuf.std.std_temp /= 1000.0F;
        databuf.std.std_temp_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.std,
             SENSOR_TYPE_TEMPERATURE);
        if (rc) {
            return rc;
        }
    }

    if (type & SENSOR_TYPE_RELATIVE_HUMIDITY) {
        rc = hts221_read8(lhb, HTS221_HUMIDITY_OUT_H, &msb);
        rc = hts221_read8(lhb, HTS221_HUMIDITY_OUT_L, &lsb);

        if (rc) {
            return rc;
        }
        raw = ((int16_t)msb<<8) | lsb;

        /* Data in C */ 
        databuf.srhd.shd_humid = hts221_calc_rh(&(lhb->calibration), raw);
        databuf.srhd.shd_humid /= 10.0F;
        databuf.srhd.shd_humid_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.srhd,
             SENSOR_TYPE_RELATIVE_HUMIDITY);
        if (rc) {
            return rc;
        }
    }
    
    return 0;
}

static int
hts221_sensor_get_config(struct sensor *sensor, sensor_type_t type,
        struct sensor_cfg *cfg)
{
    /* If the read isn't looking for accel or gyro, don't do anything. */
    if (!(type & (SENSOR_TYPE_TEMPERATURE|SENSOR_TYPE_RELATIVE_HUMIDITY))) {
        return SYS_EINVAL;
    }

    cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT;

    return 0;
}
