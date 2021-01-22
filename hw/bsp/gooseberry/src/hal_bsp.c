/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
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

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>
#include <sysinit/sysinit.h>
#include <nrf52.h>
#include "mcu/nrf52_hal.h"
#include "mcu/nrf52_periph.h"
#include "os/os_cputime.h"
#include "syscfg/syscfg.h"
#include "sysflash/sysflash.h"
#include "flash_map/flash_map.h"
#include "hal/hal_bsp.h"
#include "hal/hal_system.h"
#include "hal/hal_flash.h"
#include "hal/hal_spi.h"
#include "hal/hal_watchdog.h"
#include "hal/hal_i2c.h"
#include "hal/hal_gpio.h"
#include "mcu/nrf52_hal.h"

#include "os/os_dev.h"
#include "bsp.h"

#if MYNEWT_VAL(LPS22HB_ONB)
#include <lps22hb/lps22hb.h>
static struct lps22hb lps22hb;

static struct sensor_itf i2c_1_itf_lhb = {
    .si_type = SENSOR_ITF_I2C,
    .si_num  = 1,
    .si_addr = 0b1011100
};
#endif

#if MYNEWT_VAL(BMX160_ONB)
#include <bmx160/bmx160.h>
#include <bmx160/bmx160_defs.h>
static struct bmx160 bmx160 = {0};

static struct sensor_itf i2c_1_itf_bmx = {
    .si_type = SENSOR_ITF_I2C,
    .si_num = 1,
    .si_addr = 0b1101000
};
#endif


/*
 * What memory to include in coredump.
 */
static const struct hal_bsp_mem_dump dump_cfg[] = {
    [0] = {
        .hbmd_start = &_ram_start,
        .hbmd_size = RAM_SIZE
    }
};

const struct hal_flash * hal_bsp_flash_dev(uint8_t id)
{
    /*
     * Internal flash mapped to id 0.
     */
    if (id != 0) {
        return NULL;
    }
    return &nrf52k_flash_dev;
}

const struct hal_bsp_mem_dump * hal_bsp_core_dump(int *area_cnt)
{
    *area_cnt = sizeof(dump_cfg) / sizeof(dump_cfg[0]);
    return dump_cfg;
}

int hal_bsp_power_state(int state)
{
    return (0);
}

/**
 * Returns the configured priority for the given interrupt. If no priority
 * configured, return the priority passed in
 *
 * @param irq_num
 * @param pri
 *
 * @return uint32_t
 */
uint32_t hal_bsp_get_nvic_priority(int irq_num, uint32_t pri)
{
    uint32_t cfg_pri;

    switch (irq_num) {
    /* Radio gets highest priority */
    case RADIO_IRQn:
        cfg_pri = 0;
        break;
    default:
        cfg_pri = pri;
    }
    return cfg_pri;
}


/**
 * LPS22HB Sensor default configuration
 *
 * @return 0 on success, non-zero on failure
 */
int
config_lps22hb_sensor(void)
{
#if MYNEWT_VAL(LPS22HB_ONB)
    int rc;
    struct os_dev *dev;
    struct lps22hb_cfg cfg;

    dev = (struct os_dev *) os_dev_open("lps22hb_0", OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    memset(&cfg, 0, sizeof(cfg));

    cfg.mask = SENSOR_TYPE_PRESSURE|SENSOR_TYPE_TEMPERATURE;
    cfg.output_rate = LPS22HB_OUTPUT_RATE_ONESHOT;
    cfg.lpf_cfg = LPS22HB_LPF_CONFIG_DISABLED;
    cfg.int_enable = 0;

    rc = lps22hb_config((struct lps22hb *)dev, &cfg);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_dev_close(dev);
#endif
    return 0;
}

int
config_bmx160_sensor(void)
{
#if MYNEWT_VAL(BMX160_ONB)
    int rc;
    struct os_dev *dev;
    struct bmx160_cfg cfg = {
        .acc_mode = BMX160_CMD_PMU_MODE_ACC_SUSPEND,
        .acc_rate = BMX160_ACC_CONF_BWP_NORMAL_AVG4 | BMX160_ACC_CONF_ODR_100HZ,
        .acc_range = BMX160_ACC_RANGE_2G,
        .gyro_mode = BMX160_CMD_PMU_MODE_GYR_SUSPEND,
        .gyro_rate = BMX160_GYR_CONF_BWP_NORMAL | BMX160_GYR_CONF_ODR_100HZ,
        .gyro_range = BMX160_GYR_RANGE_2000_DPS,
        .mag_mode = BMX160_CMD_PMU_MODE_MAG_SUSPEND,
        .mag_rate = BMX160_MAG_CONF_ODR_0_78HZ,
    };

    dev = (struct os_dev *) os_dev_open("bmx160_0", OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    memset(&cfg, 0, sizeof(cfg));
    // TODO config things
    rc = bmx160_config((struct bmx160 *)dev, &cfg);
    SYSINIT_PANIC_ASSERT(rc == 0);

    //bmx160_suspend((struct bmx160 *)dev);
    os_dev_suspend(dev, 0, 0);
    os_dev_close(dev);
#endif
    return 0;
}



static void
sensor_dev_create(void)
{
    int rc;
    (void)rc;

    /* TODO: Replace with @apache-mynewt-core/hw/drivers/sensors/lps33hw */
#if MYNEWT_VAL(LPS22HB_ONB)
    rc = os_dev_create((struct os_dev *) &lps22hb, "lps22hb_0",
      OS_DEV_INIT_PRIMARY, 0, lps22hb_init, (void *)&i2c_1_itf_lhb);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(BMX160_ONB)
    rc = os_dev_create((struct os_dev *) &bmx160, "bmx160_0",
      OS_DEV_INIT_PRIMARY, 0, bmx160_init, (void *)&i2c_1_itf_bmx);
    assert(rc == 0);
#endif
}


void hal_bsp_init(void)
{
    int rc;

    (void)rc;

    /* Make sure system clocks have started */
    hal_system_clock_start();

    /* Create all available nRF52840 peripherals */
    nrf52_periph_create();

    sensor_dev_create();
}
