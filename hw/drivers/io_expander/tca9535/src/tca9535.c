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


#include <hal/hal_bsp.h>
#include <assert.h>
#include <string.h>
#include "defs/error.h"
#include <os/os.h>
#include "sysinit/sysinit.h"
#include "hal/hal_gpio.h"
#include "hal/hal_i2c.h"
#include "tca9535/tca9535.h"
#include "syscfg/syscfg.h"
#include "log/log.h"
#include "stats/stats.h"

#if MYNEWT_VAL(IO_EXPANDER_1)
#include <io_expander/io_expander.h>
#endif

/* Define the stats section and records */
STATS_SECT_START(tca9535_stats)
    STATS_SECT_ENTRY(read_errors)
    STATS_SECT_ENTRY(write_errors)
    STATS_SECT_ENTRY(mutex_errors)
    STATS_SECT_ENTRY(n_irq)
#if MYNEWT_VAL(TCA9535_INT_STATS)
    STATS_SECT_ENTRY(p0irqs)
    STATS_SECT_ENTRY(p1irqs)
    STATS_SECT_ENTRY(p2irqs)
    STATS_SECT_ENTRY(p3irqs)
    STATS_SECT_ENTRY(p4irqs)
    STATS_SECT_ENTRY(p5irqs)
    STATS_SECT_ENTRY(p6irqs)
    STATS_SECT_ENTRY(p7irqs)
    STATS_SECT_ENTRY(p8irqs)
    STATS_SECT_ENTRY(p9irqs)
    STATS_SECT_ENTRY(pAirqs)
    STATS_SECT_ENTRY(pBirqs)
    STATS_SECT_ENTRY(pCirqs)
    STATS_SECT_ENTRY(pDirqs)
    STATS_SECT_ENTRY(pEirqs)
    STATS_SECT_ENTRY(pFirqs)
#endif
STATS_SECT_END

/* Global variable used to hold stats data */
STATS_SECT_DECL(tca9535_stats) g_tca9535_stats;

/* Define stat names for querying */
STATS_NAME_START(tca9535_stats)
    STATS_NAME(tca9535_stats, read_errors)
    STATS_NAME(tca9535_stats, write_errors)
    STATS_NAME(tca9535_stats, mutex_errors)
    STATS_NAME(tca9535_stats, n_irq)
#if MYNEWT_VAL(TCA9535_INT_STATS)
    STATS_NAME(tca9535_stats, p0irqs)
    STATS_NAME(tca9535_stats, p1irqs)
    STATS_NAME(tca9535_stats, p2irqs)
    STATS_NAME(tca9535_stats, p3irqs)
    STATS_NAME(tca9535_stats, p4irqs)
    STATS_NAME(tca9535_stats, p5irqs)
    STATS_NAME(tca9535_stats, p6irqs)
    STATS_NAME(tca9535_stats, p7irqs)
    STATS_NAME(tca9535_stats, p8irqs)
    STATS_NAME(tca9535_stats, p9irqs)
    STATS_NAME(tca9535_stats, pAirqs)
    STATS_NAME(tca9535_stats, pBirqs)
    STATS_NAME(tca9535_stats, pCirqs)
    STATS_NAME(tca9535_stats, pDirqs)
    STATS_NAME(tca9535_stats, pEirqs)
    STATS_NAME(tca9535_stats, pFirqs)
#endif
STATS_NAME_END(tca9535_stats)

static struct os_eventq interrupt_eventq;
static struct os_event interrupt_ev;
static struct os_task interrupt_task_str;
OS_TASK_STACK_DEFINE(interrupt_task_stack, MYNEWT_VAL(TCA9535_DEV_TASK_STACK_SZ));

#define LOG_MODULE_TCA9535    (9535)
#define TCA9535_INFO(...)     LOG_INFO(&_log, LOG_MODULE_TCA9535, __VA_ARGS__)
#define TCA9535_DEBUG(...)    LOG_DEBUG(&_log, LOG_MODULE_TCA9535, __VA_ARGS__)
#define TCA9535_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_TCA9535, __VA_ARGS__)
static struct log _log;

#define TCA95xx_input  0x00
#define TCA95xx_output 0x02
#define TCA95xx_polarity 0x04
#define TCA95xx_configuration 0x06

//! \brief Bitmask for no pins
//!
#define TCA95xx_NONE    (0x0000)

//! \brief Bitmask for all pins
//!
#define TCA95xx_ALL        (0xFFFF)

//! \brief Compile-time control of whether a read-back verification
//!        is performed for all writes.  Comment out to disable
//!        readback.
//!
#define TCA95xx_VERIFY_WRITES

static int tca9535_write_out(struct io_expander_dev *dev, int pin, int val);

                                      
                                      
/**
 * Writes a single word to the specified register
 *
 * @param The config
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero error on failure.
 */
int
tca95xx_write16(struct tca9535_io_expander_dev_cfg *cfg, uint8_t reg, uint16_t value)
{
    int rc;
    os_error_t err = 0;
#ifdef TCA95xx_VERIFY_WRITES
    uint16_t readback;
#endif

	uint8_t out[3] = {reg, value&0xff, (value>>8)};
    struct hal_i2c_master_data data_struct = {
        .address = cfg->i2c_addr,
        .len = 3,
        .buffer = out
    };

    err = os_mutex_pend(cfg->i2c_mutex, OS_WAIT_FOREVER);
    if (err != OS_OK)
    {
        TCA9535_ERR("Mtx err:%d\n", err);
        STATS_INC(g_tca9535_stats, mutex_errors);
        return err;
    }

    /* Register write */
    rc = hal_i2c_master_write(cfg->i2c_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        TCA9535_ERR("w16 fail\n");
        STATS_INC(g_tca9535_stats, write_errors);
        goto exit;
    }
    
#ifdef TCA95xx_VERIFY_WRITES
    data_struct.buffer = &reg;
    data_struct.len = 1;

    /* Register write */
    rc = hal_i2c_master_write(cfg->i2c_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);

    /* Read one word back */
    data_struct.buffer = (uint8_t*)&readback;
    data_struct.len = 2;
    rc = hal_i2c_master_read(cfg->i2c_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        TCA9535_ERR("w16 ver fail\n");
        STATS_INC(g_tca9535_stats, write_errors);
        goto exit;
    }

    if (readback != value)
    {
        TCA9535_ERR("Rback err: %x != %x\r\n", readback, value);
        STATS_INC(g_tca9535_stats, write_errors);
    }
#endif
exit:
    err = os_mutex_release(cfg->i2c_mutex);
    assert(err == OS_OK);
    return rc;
}

int
tca95xx_read16(struct tca9535_io_expander_dev_cfg *cfg, uint8_t reg, uint16_t *value)
{
    int rc = 0;
    os_error_t err;

    struct hal_i2c_master_data data_struct = {
        .address = cfg->i2c_addr,
        .len = 1,
        .buffer = &reg
    };

    err = os_mutex_pend(cfg->i2c_mutex, OS_WAIT_FOREVER);
    if (err != OS_OK)
    {
        TCA9535_ERR("Mtx err:%d\n", err);
        STATS_INC(g_tca9535_stats, mutex_errors);
        return err;
    }

    /* Register write */
    rc = hal_i2c_master_write(cfg->i2c_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        TCA9535_ERR("r16 fail rc=%d\n", rc);
        STATS_INC(g_tca9535_stats, write_errors);
        goto exit;
    }

    /* Read one word back */
    data_struct.buffer = (uint8_t*)value;
    data_struct.len = 2;
    rc = hal_i2c_master_read(cfg->i2c_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        TCA9535_ERR("r16 fail rc=%d\n", rc);
        STATS_INC(g_tca9535_stats, read_errors);
        goto exit;
    }
exit:
    err = os_mutex_release(cfg->i2c_mutex);
    assert(err == OS_OK);
    return rc;
}

static int
tca9535_init_in16(struct io_expander_dev *dev, uint16_t pin_mask)
{
    int rc;
    uint16_t pin_config;
    struct tca9535_io_expander_dev_cfg *cfg;
    
    cfg  = (struct tca9535_io_expander_dev_cfg *)dev->ioexp_dev.od_init_arg;

    rc = tca95xx_read16(cfg, TCA95xx_configuration, &pin_config);
    if (rc) {
        TCA9535_ERR("Failed to set in16 pins on 0x%02X\n", cfg->i2c_addr);
        return rc;
    }

    pin_config |= pin_mask;
    cfg->direction = pin_config;
    
    rc = tca95xx_write16(cfg, TCA95xx_configuration, pin_config);
    if (rc) {
        TCA9535_ERR("Failed to set in16 pins 0x%02X\n", cfg->i2c_addr);
        return rc;
    }
    
    return 0;
}

int
tca9535_init_in(struct io_expander_dev *dev, int pin, hal_gpio_pull_t pull)
{
    assert(pull == HAL_GPIO_PULL_NONE);
    return tca9535_init_in16(dev, (1 << pin));
}


static int
tca9535_init_out16(struct io_expander_dev *dev, uint16_t pinmask)
{
    int rc;
    uint16_t pin_config;
    struct tca9535_io_expander_dev_cfg *cfg;

    cfg = (struct tca9535_io_expander_dev_cfg *)dev->ioexp_dev.od_init_arg;

    rc = tca95xx_read16(cfg, TCA95xx_configuration, &pin_config);
    if (rc) {
        TCA9535_ERR("Err. out16 pins 0x%02X\n");
        return rc;
    }

    pin_config &= ~(pinmask);
    cfg->direction = pin_config;
    
    rc = tca95xx_write16(cfg, TCA95xx_configuration, pin_config);
    if (rc) {
        TCA9535_ERR("Err. out16 pins\n");
        return rc;
    }    
    
    return 0;
}

static int
tca9535_pin_dir(struct io_expander_dev *dev, int pin, int *dir)
{
    uint16_t pinmask;
    pinmask = (1 << pin);

    struct tca9535_io_expander_dev_cfg *cfg;
    cfg = (struct tca9535_io_expander_dev_cfg *)dev->ioexp_dev.od_init_arg;
    
    if (dir)
    {
        *dir = (pinmask & cfg->direction);
    }
    return 0;
}

static int
tca9535_init_out(struct io_expander_dev *dev, int pin, int val)
{
    int rc;
    struct tca9535_io_expander_dev_cfg *cfg;
    cfg = (struct tca9535_io_expander_dev_cfg *)dev->ioexp_dev.od_init_arg;

    rc = tca9535_init_out16(dev, (1 << pin));
    if (rc) {
        TCA9535_ERR("Failed to set out pins 0x%02X\n", cfg->i2c_addr);
        return rc;
    }    
    
    return tca9535_write_out(dev, pin, val);
}

static int
tca9535_write_out(struct io_expander_dev *dev, int pin, int val)
{
    int rc;
    uint16_t pin_output;
    struct tca9535_io_expander_dev_cfg *cfg;

    cfg = (struct tca9535_io_expander_dev_cfg *)dev->ioexp_dev.od_init_arg;

    rc = tca95xx_read16(cfg, TCA95xx_output, &pin_output);
    if (rc) {
        TCA9535_ERR("Failed to write out pins 0x%02X\n", cfg->i2c_addr);
        return rc;
    }

    if (val)
        pin_output |= (1 << pin);
    else
        pin_output &= ~(1 << pin);
    
    rc = tca95xx_write16(cfg, TCA95xx_output, pin_output);
    if (rc) {
        TCA9535_ERR("Failed to write out pins 0x%02X\n", cfg->i2c_addr);
        return rc;
    }
    
    return 0;
}

static int
tca9535_read_in16(struct io_expander_dev *dev, uint16_t *pin_input)
{
    int rc;
    struct tca9535_io_expander_dev_cfg *cfg;

    cfg = (struct tca9535_io_expander_dev_cfg *)dev->ioexp_dev.od_init_arg;

    rc = tca95xx_read16(cfg, TCA95xx_input, pin_input);
    if (rc) {
        TCA9535_ERR("Failed to read input pins 0x%02X\n", cfg->i2c_addr);
        return rc;
    }

    return 0;
}


static int
tca9535_read(struct io_expander_dev *dev, int pin)
{
    int rc;
    uint16_t pin_input;
    struct tca9535_io_expander_dev_cfg *cfg;

    cfg = (struct tca9535_io_expander_dev_cfg *)dev->ioexp_dev.od_init_arg;

    rc = tca95xx_read16(cfg, TCA95xx_input, &pin_input);
    if (rc) {
        TCA9535_ERR("Failed to read pins 0x%02X\n", cfg->i2c_addr);
        return rc;
    }

    if (pin_input & (1<<pin)) return 1;
    return 0;
}

static int
tca9535_irq_init(struct io_expander_dev *dev, int pin,
                 hal_gpio_irq_handler_t handler, void *arg,
                 hal_gpio_irq_trig_t trig, hal_gpio_pull_t pull)
{
    assert(pull == HAL_GPIO_PULL_NONE);
    assert(pin < IO_EXPANDER_MAX_IRQ);

    struct io_expander_irq * irq = &(dev->ioexp_irqs[pin]);
    irq->enabled = 1;
    irq->trig = trig;
    irq->func = handler;
    irq->arg = arg;
    
    TCA9535_DEBUG("Intr init pin %d\n", pin);
    return 0;
}

static int
tca9535_irq_release(struct io_expander_dev *dev, int pin)
{
    assert(pin < IO_EXPANDER_MAX_IRQ);

    struct io_expander_irq * irq = &(dev->ioexp_irqs[pin]);
    irq->enabled = 0;
    irq->trig = HAL_GPIO_TRIG_NONE;
    irq->func = 0;
    irq->arg = 0;
    return 0;
}

static int
tca9535_irq_enable(struct io_expander_dev *dev, int pin)
{
    assert(pin < IO_EXPANDER_MAX_IRQ);

    struct io_expander_irq * irq = &(dev->ioexp_irqs[pin]);
    irq->enabled = 1;
    return 0;
}

static int
tca9535_irq_disable(struct io_expander_dev *dev, int pin)
{
    assert(pin < IO_EXPANDER_MAX_IRQ);

    struct io_expander_irq * irq = &(dev->ioexp_irqs[pin]);
    irq->enabled = 0;
    return 0;
}


static void
tca9535_irq(void *arg)
{
    STATS_INC(g_tca9535_stats, n_irq);

    if(os_eventq_inited(&interrupt_eventq)) {
        os_eventq_put(&(interrupt_eventq), &(interrupt_ev));
    }
}

static void tca9535_interrupt_ev_cb(struct os_event *ev);

static void
tca9535_interrupt_task(void *arg)
{
    while (1) {
        os_eventq_run(&interrupt_eventq);
    }
}

/**
 *  Event callback that is called everytime an input-pin changes
 *  If an input pin has a callback associated with the current 
 *  change (rising / falling) call this is called.
 **/
static void
tca9535_interrupt_ev_cb(struct os_event *ev)
{
    int rc, i;
    uint16_t pin_input, changed_pins;
    struct tca9535_io_expander_dev_cfg *cfg;
    struct io_expander_dev *dev = (struct io_expander_dev *) ev->ev_arg;
    
    cfg  = (struct tca9535_io_expander_dev_cfg *)dev->ioexp_dev.od_init_arg;

    /* Read current state of pins */
    rc = tca9535_read_in16(dev, &pin_input);
    if (rc) {
        TCA9535_ERR("Failed to get pin values 0x%02X\n", cfg->i2c_addr);
        return;
    }

    changed_pins = (cfg->last_interrupt_input) ^ pin_input;
    //TCA9535_INFO("%x\n", changed_pins);
    cfg->last_interrupt_input = pin_input;

    /* Check for any activated interrupts */
    rc = 1;
    for (i=0;i<IO_EXPANDER_MAX_IRQ;i++) {
        if (dev->ioexp_irqs[i].enabled == 0) rc=0;
        
        if ((changed_pins & (1<<i)) == 0) continue;
#if MYNEWT_VAL(TCA9535_INT_STATS)
        switch(i) {
        case (0x0): STATS_INC(g_tca9535_stats, p0irqs);break;
        case (0x1): STATS_INC(g_tca9535_stats, p1irqs);break;
        case (0x2): STATS_INC(g_tca9535_stats, p2irqs);break;
        case (0x3): STATS_INC(g_tca9535_stats, p3irqs);break;
        case (0x4): STATS_INC(g_tca9535_stats, p4irqs);break;
        case (0x5): STATS_INC(g_tca9535_stats, p5irqs);break;
        case (0x6): STATS_INC(g_tca9535_stats, p6irqs);break;
        case (0x7): STATS_INC(g_tca9535_stats, p7irqs);break;
        case (0x8): STATS_INC(g_tca9535_stats, p8irqs);break;
        case (0x9): STATS_INC(g_tca9535_stats, p9irqs);break;
        case (0xA): STATS_INC(g_tca9535_stats, pAirqs);break;
        case (0xB): STATS_INC(g_tca9535_stats, pBirqs);break;
        case (0xC): STATS_INC(g_tca9535_stats, pCirqs);break;
        case (0xD): STATS_INC(g_tca9535_stats, pDirqs);break;
        case (0xE): STATS_INC(g_tca9535_stats, pEirqs);break;
        case (0xF): STATS_INC(g_tca9535_stats, pFirqs);break;
        }
#endif
    }
    
    if (rc == 1)
    {
        /* No interrupts to service */
        return;
    }

    if (!changed_pins) return;

    for (i=0;i<IO_EXPANDER_MAX_IRQ;i++) {
        if ((changed_pins & (1<<i)) == 0) continue;
        if (dev->ioexp_irqs[i].enabled == 0) continue;
        int pin_now  = ((pin_input & (1<<i)) != 0);
        if ((dev->ioexp_irqs[i].trig == HAL_GPIO_TRIG_RISING)  && pin_now == 0) continue;
        if ((dev->ioexp_irqs[i].trig == HAL_GPIO_TRIG_FALLING) && pin_now == 1) continue;

        /* Call interrupt cb */
        assert(dev->ioexp_irqs[i].func);
        dev->ioexp_irqs[i].func(dev->ioexp_irqs[i].arg);
    }
    
}


/**
 * Callback to initialize an io_expander_dev structure from the os device
 * initialization callback.  This sets up a tca9525_io_expander_device(), so
 * that subsequent lookups to this device allow us to manipulate it.
 *
 * @param1 os device ptr
 * @param2 tca9535 io expander device cfg ptr
 * @return OS_OK on success
 */
int
tca9535_io_expander_dev_init(struct os_dev *odev, void *arg)
{
    int i;
    struct tca9535_io_expander_dev_cfg *cfg;
    struct io_expander_dev *dev;
    struct io_expander_driver_funcs *iof;

    cfg = (struct tca9535_io_expander_dev_cfg *) arg;

    assert(cfg != NULL);

    dev = (struct io_expander_dev *)odev;
    dev->number_of_pins = 16;
    
    log_register(odev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    iof = &dev->iof_funcs;

    iof->iof_init_in = tca9535_init_in;
    iof->iof_init_out = tca9535_init_out;
    iof->iof_pin_dir = tca9535_pin_dir;
    iof->iof_write = tca9535_write_out;
    iof->iof_read = tca9535_read;

    iof->iof_irq_init = tca9535_irq_init;
    iof->iof_irq_release = tca9535_irq_release;
    iof->iof_irq_enable = tca9535_irq_enable;
    iof->iof_irq_disable = tca9535_irq_disable;
    
    /* Clear interrupt structure */
    for (i=0;i<IO_EXPANDER_MAX_IRQ;i++)
    {
        dev->ioexp_irqs[i].trig = HAL_GPIO_TRIG_NONE;
        dev->ioexp_irqs[i].enabled = 0;
        dev->ioexp_irqs[i].func = 0;
        dev->ioexp_irqs[i].arg = 0;
    }

    return (OS_OK);
}

int
tca9535_io_expander_task_init(struct io_expander_dev *dev)
{
    int rc;
    struct tca9535_io_expander_dev_cfg *cfg;

    /* Init stats */
    rc = stats_init_and_reg(
        STATS_HDR(g_tca9535_stats), STATS_SIZE_INIT_PARMS(g_tca9535_stats,
        STATS_SIZE_32), STATS_NAME_INIT_PARMS(tca9535_stats), "tca9535");
    SYSINIT_PANIC_ASSERT(rc == 0);

    cfg  = (struct tca9535_io_expander_dev_cfg *)dev->ioexp_dev.od_init_arg;
    assert(cfg != NULL);

    /* Use a dedicate event queue for timer and interrupt events */
    os_eventq_init(&interrupt_eventq);
    assert(os_eventq_inited(&interrupt_eventq));
    
    /* 
     * Create the task to process timer and interrupt events from the
     * cfg->interrupt_eventq event queue.
     */
    interrupt_ev.ev_queued = 0;
    interrupt_ev.ev_cb = tca9535_interrupt_ev_cb;
    interrupt_ev.ev_arg = (void *)dev;
    
    os_task_init(&interrupt_task_str, "tca_irq", 
                 tca9535_interrupt_task,
                 (void *) dev, 
                 MYNEWT_VAL(TCA9535_DEV_TASK_PRIO), OS_WAIT_FOREVER, 
                 interrupt_task_stack,
                 MYNEWT_VAL(TCA9535_DEV_TASK_STACK_SZ));

    hal_gpio_irq_init(cfg->irq_pin, tca9535_irq, (void*) dev,
                      HAL_GPIO_TRIG_FALLING, HAL_GPIO_PULL_UP);
    hal_gpio_irq_enable(cfg->irq_pin);

    /* Read input in order to clear any outstanding interrupt */
    uint16_t pin_input;
    rc = tca9535_read_in16(dev, &pin_input);
    if (rc)
    {
        TCA9535_ERR("Fail to read init inp state\n");
    }
    cfg->last_interrupt_input = pin_input;

    return (OS_OK);
}
