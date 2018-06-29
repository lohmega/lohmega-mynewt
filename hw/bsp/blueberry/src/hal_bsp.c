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

#if MYNEWT_VAL(UART_0)
#include "uart/uart.h"
#include "uart_hal/uart_hal.h"
#endif

#include "os/os_dev.h"
#include "bsp.h"

#if MYNEWT_VAL(MPU6500_ONB)
#include <mpu6500/mpu6500.h>
static struct mpu6500 mpu6500;
#endif

#if MYNEWT_VAL(UART_0)
static struct uart_dev os_bsp_uart0;
static const struct nrf52_uart_cfg os_bsp_uart0_cfg = {
    .suc_pin_tx = MYNEWT_VAL(UART_0_PIN_TX),
    .suc_pin_rx = MYNEWT_VAL(UART_0_PIN_RX),
    .suc_pin_rts = MYNEWT_VAL(UART_0_PIN_RTS),
    .suc_pin_cts = MYNEWT_VAL(UART_0_PIN_CTS),
};
#endif

#if MYNEWT_VAL(I2C_1)
static const struct nrf52_hal_i2c_cfg hal_i2c_cfg = {
    .scl_pin = 12,
    .sda_pin = 11,
    .i2c_frequency = 400    /* 400 kHz */
};

#if MYNEWT_VAL(MPU6500_ONB)
static struct sensor_itf i2c_1_itf_mpu = {
    .si_type = SENSOR_ITF_I2C,
    .si_num  = 1,
    .si_addr = 0x69        /* 0b1101001 */
};
#endif

#if MYNEWT_VAL(LPS22HB_ONB)
#include <lps22hb/lps22hb.h>
static struct lps22hb lps22hb;

static struct sensor_itf i2c_1_itf_lhb = {
    .si_type = SENSOR_ITF_I2C,
    .si_num  = 1,
    .si_addr = 0b1011100
};
#endif

#if MYNEWT_VAL(HTS221_ONB)
#include <hts221/hts221.h>
static struct hts221 hts221;

static struct sensor_itf i2c_1_itf_hts = {
    .si_type = SENSOR_ITF_I2C,
    .si_num  = 1,
    .si_addr = 0b1011111
};
#endif

#if MYNEWT_VAL(SI1133_ONB)
#include <si1133/si1133.h>
static struct si1133 si1133;

static struct sensor_itf i2c_1_itf_si1 = {
    .si_type = SENSOR_ITF_I2C,
    .si_num = 1,
    .si_addr = 0b1010101
};
#endif


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
 * MPU6500 Sensor default configuration
 *
 * @return 0 on success, non-zero on failure
 */
int
config_mpu6500_sensor(void)
{
#if MYNEWT_VAL(MPU6500_ONB)
    int rc;
    struct os_dev *dev;
    struct mpu6500_cfg cfg;

    dev = (struct os_dev *) os_dev_open("mpu6500_0", OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    memset(&cfg, 0, sizeof(cfg));

    cfg.mask = SENSOR_TYPE_ACCELEROMETER | SENSOR_TYPE_GYROSCOPE;
    cfg.accel_range = MPU6500_ACCEL_RANGE_16;
    cfg.gyro_range = MPU6500_GYRO_RANGE_2000;
    cfg.sample_rate_div = MPU6500_GYRO_RATE_200;
    cfg.lpf_cfg = 0;
    cfg.int_enable = 0;
    cfg.int_cfg = 0;
    
    rc = mpu6500_config((struct mpu6500 *)dev, &cfg);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_dev_close(dev);
#endif
    return 0;
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


/**
 * HTS221 Sensor default configuration
 *
 * @return 0 on success, non-zero on failure
 */
int
config_hts221_sensor(void)
{
#if MYNEWT_VAL(HTS221_ONB)
    int rc;
    struct os_dev *dev;
    struct hts221_cfg cfg;

    dev = (struct os_dev *) os_dev_open("hts221_0", OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    memset(&cfg, 0, sizeof(cfg));

    cfg.mask = SENSOR_TYPE_RELATIVE_HUMIDITY|SENSOR_TYPE_TEMPERATURE;
    cfg.int_enable = 0;
    
    rc = hts221_config((struct hts221 *)dev, &cfg);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_dev_close(dev);
#endif
    return 0;
}

int
config_si1133_sensor(void)
{
#if MYNEWT_VAL(SI1133_ONB)
    int rc;
    struct os_dev *dev;
    struct si1133_cfg cfg;

    dev = (struct os_dev *) os_dev_open("si1133_0", OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    memset(&cfg, 0, sizeof(cfg));
    cfg.mask = SENSOR_TYPE_LIGHT;
    cfg.int_enable = 0;

    rc = si1133_config();//(struct si1133 *)dev, &cfg); // TODO add argument 
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_dev_close(dev);
#endif
    return 0;
}




static void
sensor_dev_create(void)
{
    int rc;
    (void)rc;
    
#if MYNEWT_VAL(MPU6500_ONB)
    rc = os_dev_create((struct os_dev *) &mpu6500, "mpu6500_0",
      OS_DEV_INIT_PRIMARY, 0, mpu6500_init, (void *)&i2c_1_itf_mpu);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(LPS22HB_ONB)
    rc = os_dev_create((struct os_dev *) &lps22hb, "lps22hb_0",
      OS_DEV_INIT_PRIMARY, 0, lps22hb_init, (void *)&i2c_1_itf_lhb);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(HTS221_ONB)
    rc = os_dev_create((struct os_dev *) &hts221, "hts221_0",
      OS_DEV_INIT_PRIMARY, 0, hts221_init, (void *)&i2c_1_itf_hts);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SI1133_ONB)
    rc = os_dev_create((struct os_dev *) &si1133, "si1133_0",
      OS_DEV_INIT_PRIMARY, 0, si1133_init, (void *)&i2c_1_itf_si1);
    assert(rc == 0);
#endif
    
}


void hal_bsp_init(void)
{
    int rc;

    (void)rc;

    /* Make sure system clocks have started */
    hal_system_clock_start();

#if MYNEWT_VAL(TIMER_0)
    rc = hal_timer_init(0, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_1)
    rc = hal_timer_init(1, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_2)
    rc = hal_timer_init(2, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_3)
    rc = hal_timer_init(3, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_4)
    rc = hal_timer_init(4, NULL);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_5)
    rc = hal_timer_init(5, NULL);
    assert(rc == 0);
#endif

#if (MYNEWT_VAL(OS_CPUTIME_TIMER_NUM) >= 0)
    rc = os_cputime_init(MYNEWT_VAL(OS_CPUTIME_FREQ));
    assert(rc == 0);
#endif

#if MYNEWT_VAL(I2C_1)
    rc = hal_i2c_init(1, (void *)&hal_i2c_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_0_MASTER)
    rc = hal_spi_init(0, (void *)&os_bsp_spi0m_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_0_SLAVE)
    rc = hal_spi_init(0, (void *)&os_bsp_spi0s_cfg, HAL_SPI_TYPE_SLAVE);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(UART_0)
    rc = os_dev_create((struct os_dev *) &os_bsp_uart0, "uart0",
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&os_bsp_uart0_cfg);
    assert(rc == 0);
#endif

    sensor_dev_create();
}
