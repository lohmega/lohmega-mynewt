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
#include "mcp23008/mcp23008.h"
#include "syscfg/syscfg.h"
#include "log/log.h"

#if MYNEWT_VAL(IO_EXPANDER_1)
#include <io_expander/io_expander.h>
#endif

#if MYNEWT_VAL(MCP23008_STATS)
#include "stats/stats.h"

/* Define the stats section and records */
STATS_SECT_START(mcp23008_stats)
    STATS_SECT_ENTRY(read_errors)
    STATS_SECT_ENTRY(write_errors)
    STATS_SECT_ENTRY(mutex_errors)
    STATS_SECT_ENTRY(n_irq)
#if MYNEWT_VAL(MCP23008_INT_STATS)
    STATS_SECT_ENTRY(p0irqs)
    STATS_SECT_ENTRY(p1irqs)
    STATS_SECT_ENTRY(p2irqs)
    STATS_SECT_ENTRY(p3irqs)
    STATS_SECT_ENTRY(p4irqs)
    STATS_SECT_ENTRY(p5irqs)
    STATS_SECT_ENTRY(p6irqs)
    STATS_SECT_ENTRY(p7irqs)
#endif
STATS_SECT_END

/* Global variable used to hold stats data */
STATS_SECT_DECL(mcp23008_stats) g_mcp23008_stats;

/* Define stat names for querying */
STATS_NAME_START(mcp23008_stats)
    STATS_NAME(mcp23008_stats, read_errors)
    STATS_NAME(mcp23008_stats, write_errors)
    STATS_NAME(mcp23008_stats, mutex_errors)
    STATS_NAME(mcp23008_stats, n_irq)
#if MYNEWT_VAL(MCP23008_INT_STATS)
    STATS_NAME(mcp23008_stats, p0irqs)
    STATS_NAME(mcp23008_stats, p1irqs)
    STATS_NAME(mcp23008_stats, p2irqs)
    STATS_NAME(mcp23008_stats, p3irqs)
    STATS_NAME(mcp23008_stats, p4irqs)
    STATS_NAME(mcp23008_stats, p5irqs)
    STATS_NAME(mcp23008_stats, p6irqs)
    STATS_NAME(mcp23008_stats, p7irqs)
#endif
STATS_NAME_END(mcp23008_stats)

#define MCP23008_STATS_INC(__X) STATS_INC(g_mcp23008_stats, __X);
#else /* MYNEWT_VAL(MCP23008_STATS) */

#define MCP23008_STATS_INC(__X) {};

#endif

#define LOG_MODULE_MCP23008    (238)
#define MCP23008_INFO(...)     LOG_INFO(&_log, LOG_MODULE_MCP23008, __VA_ARGS__)
#define MCP23008_DEBUG(...)    LOG_DEBUG(&_log, LOG_MODULE_MCP23008, __VA_ARGS__)
#define MCP23008_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_MCP23008, __VA_ARGS__)
static struct log _log;

#define MCP23008_IODIR_REG   (0x00)
#define MCP23008_IPOL_REG    (0x01)
#define MCP23008_GPINTEN_REG (0x02)
#define MCP23008_DEFVAL_REG  (0x03)
#define MCP23008_INTCON_REG  (0x04)
#define MCP23008_IOCON_REG   (0x05)
#define MCP23008_GPPU_REG    (0x06)
#define MCP23008_INTF_REG    (0x07)
#define MCP23008_INTCAP_REG  (0x08)
#define MCP23008_GPIO_REG    (0x09)
#define MCP23008_OLAT_REG    (0x0A)

//! \brief Compile-time control of whether a read-back verification
//!        is performed for all writes.  Comment out to disable
//!        readback.
//!
#define Mcp23008_VERIFY_WRITES

static int mcp23008_write_out(struct io_expander_dev *dev, int pin, int val);


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
mcp23008_write_reg(struct io_expander_dev *dev, uint8_t reg, uint8_t value)
{
    int rc = 0;
    os_error_t err = 0;
#ifdef Mcp23008_VERIFY_WRITES
    uint8_t readback;
#endif
    uint8_t out[] = {reg, value};

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    err = bus_node_simple_write((struct os_dev*)dev, out, 2);
    if (err) {
        rc = err;
    }
#ifdef Mcp23008_VERIFY_WRITES
    err = bus_node_simple_write_read_transact((struct os_dev*)dev, &reg, 1, &readback, 1);
    if (err) {
        rc = err;
    }
    if (readback != value) {
        MCP23008_ERR("Rback err: %x != %x\r\n", readback, value);
        MCP23008_STATS_INC(write_errors);
    }
#endif

#else  /* MYNEWT_VAL(BUS_DRIVER_PRESENT) */
    struct mcp23008_io_expander_dev_cfg *cfg;
    cfg = (struct mcp23008_io_expander_dev_cfg *)((struct os_dev*)dev)->od_init_arg;

    struct hal_i2c_master_data data_struct = {
        .address = cfg->i2c_addr,
        .len = 2,
        .buffer = out
    };

    err = os_mutex_pend(cfg->i2c_mutex, OS_WAIT_FOREVER);
    if (err != OS_OK)
    {
        MCP23008_ERR("Mtx err:%d\n", err);
        MCP23008_STATS_INC(mutex_errors);
        return err;
    }

    /* Register write */
    rc = hal_i2c_master_write(cfg->i2c_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        MCP23008_ERR("w16 fail\n");
        MCP23008_STATS_INC(write_errors);
        goto exit;
    }

#ifdef Mcp23008_VERIFY_WRITES
    data_struct.buffer = &reg;
    data_struct.len = 1;

    /* Register write */
    rc = hal_i2c_master_write(cfg->i2c_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);

    /* Read one word back */
    data_struct.buffer = (uint8_t*)&readback;
    data_struct.len = 1;
    rc = hal_i2c_master_read(cfg->i2c_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        MCP23008_ERR("wreg ver fail\n");
        MCP23008_STATS_INC(write_errors);
        goto exit;
    }

    if (readback != value) {
        MCP23008_ERR("Rback err: %x != %x\r\n", readback, value);
        MCP23008_STATS_INC(write_errors);
    }
#endif
exit:
    err = os_mutex_release(cfg->i2c_mutex);
    assert(err == OS_OK);
#endif  /* MYNEWT_VAL(BUS_DRIVER_PRESENT) */

    return rc;
}

int
mcp23008_read_reg(struct io_expander_dev *dev, uint8_t reg, uint8_t *value)
{
    int rc = 0;
    os_error_t err;

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    err = bus_node_simple_write_read_transact((struct os_dev*)dev, &reg, 1, value, 1);
    if (err) {
        rc = err;
    }
#else
    struct mcp23008_io_expander_dev_cfg *cfg;
    cfg = (struct mcp23008_io_expander_dev_cfg *)((struct os_dev*)dev)->od_init_arg;

    struct hal_i2c_master_data data_struct = {
        .address = cfg->i2c_addr,
        .len = 1,
        .buffer = &reg
    };

    err = os_mutex_pend(cfg->i2c_mutex, OS_WAIT_FOREVER);
    if (err != OS_OK) {
        MCP23008_ERR("Mtx err:%d\n", err);
        MCP23008_STATS_INC(mutex_errors);
        return err;
    }

    /* Register write */
    rc = hal_i2c_master_write(cfg->i2c_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        MCP23008_ERR("rreg fail rc=%d\n", rc);
        MCP23008_STATS_INC(write_errors);
        goto exit;
    }

    /* Read one word back */
    data_struct.buffer = (uint8_t*)value;
    data_struct.len = 1;
    rc = hal_i2c_master_read(cfg->i2c_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        MCP23008_ERR("r16 fail rc=%d\n", rc);
        MCP23008_STATS_INC(read_errors);
        goto exit;
    }
exit:
    err = os_mutex_release(cfg->i2c_mutex);
    assert(err == OS_OK);
#endif  /* MYNEWT_VAL(BUS_DRIVER_PRESENT) */
    return rc;
}

static int
mcp23008_init_in8(struct io_expander_dev *dev, uint8_t pin_mask, uint8_t pullup_mask)
{
    int rc;
    uint8_t pin_config;
    uint8_t pu_config;
    struct mcp23008_io_expander_dev_cfg *cfg;
    cfg = (struct mcp23008_io_expander_dev_cfg *)((struct os_dev*)dev)->od_init_arg;

    rc = mcp23008_read_reg(dev, MCP23008_IODIR_REG, &pin_config);
    if (rc) {
        MCP23008_ERR("Err reading IODIR on on ioexp[%d]\n", cfg->inst_id);
        return rc;
    }

    pin_config |= pin_mask;
    cfg->direction = pin_config;

    rc = mcp23008_write_reg(dev, MCP23008_IODIR_REG, pin_config);
    if (rc) {
        MCP23008_ERR("Failed to set in8 pins ioexp[%d]\n", cfg->inst_id);
        return rc;
    }

    rc = mcp23008_read_reg(dev, MCP23008_GPPU_REG, &pu_config);
    if (rc) {
        MCP23008_ERR("Err reading GPPU on ioexp[%d]\n", cfg->inst_id);
        return rc;
    }

    pu_config |= pullup_mask;
    cfg->pull_up = pu_config;

    return 0;
}

int
mcp23008_init_in(struct io_expander_dev *dev, int pin, hal_gpio_pull_t pull)
{
    assert(pull == HAL_GPIO_PULL_NONE || pull == HAL_GPIO_PULL_UP);
    return mcp23008_init_in8(dev, (1 << pin), (1 << pull));
}


static int
mcp23008_init_out8(struct io_expander_dev *dev, uint8_t pinmask)
{
    int rc;
    uint8_t pin_config;

    struct mcp23008_io_expander_dev_cfg *cfg;
    cfg = (struct mcp23008_io_expander_dev_cfg *)((struct os_dev*)dev)->od_init_arg;

    rc = mcp23008_read_reg(dev, MCP23008_IODIR_REG, &pin_config);
    if (rc) {
        MCP23008_ERR("Err. out8 pins 0x%02X\n");
        return rc;
    }

    pin_config &= ~(pinmask);
    cfg->direction = pin_config;

    rc = mcp23008_write_reg(dev, MCP23008_IODIR_REG, pin_config);
    if (rc) {
        MCP23008_ERR("Err. out8 pins\n");
        return rc;
    }

    return 0;
}

static int
mcp23008_pin_dir(struct io_expander_dev *dev, int pin, int *dir)
{
    uint8_t pinmask;
    pinmask = (1 << pin);

    struct mcp23008_io_expander_dev_cfg *cfg;
    cfg = (struct mcp23008_io_expander_dev_cfg *)((struct os_dev*)dev)->od_init_arg;

    if (dir) {
        *dir = (pinmask & cfg->direction);
    }
    return 0;
}

static int
mcp23008_write_out(struct io_expander_dev *dev, int pin, int val)
{
    int rc;
    uint8_t pin_output;

    rc = mcp23008_read_reg(dev, MCP23008_OLAT_REG, &pin_output);
    if (rc) {
        MCP23008_ERR("Failed to write out pins\n");
        return rc;
    }

    if (val) {
        pin_output |= (1 << pin);
    } else {
        pin_output &= ~(1 << pin);
    }

    rc = mcp23008_write_reg(dev, MCP23008_OLAT_REG, pin_output);
    if (rc) {
        MCP23008_ERR("Failed to write out pins\n");
        return rc;
    }

    return 0;
}

static int
mcp23008_init_out(struct io_expander_dev *dev, int pin, int val)
{
    int rc;
    struct mcp23008_io_expander_dev_cfg *cfg;
    cfg = (struct mcp23008_io_expander_dev_cfg *)((struct os_dev*)dev)->od_init_arg;

    rc = mcp23008_init_out8(dev, (1 << pin));
    if (rc) {
        MCP23008_ERR("Failed to set out pins ioexp[%d]\n", cfg->inst_id);
        return rc;
    }

    return mcp23008_write_out(dev, pin, val);
}

static int
mcp23008_read_in8(struct io_expander_dev *dev, uint8_t *pin_input)
{
    int rc;

    rc = mcp23008_read_reg(dev, MCP23008_GPIO_REG, pin_input);
    if (rc) {
        MCP23008_ERR("Failed to read input pins\n");
        return rc;
    }

    return 0;
}


static int
mcp23008_read(struct io_expander_dev *dev, int pin)
{
    int rc;
    uint8_t pin_input;

    rc = mcp23008_read_in8(dev, &pin_input);
    if (rc) {
        MCP23008_ERR("Failed to read input pins\n");
    }

    return (pin_input & (1<<pin)) ? 1 : 0;
}

static int
mcp23008_irq_init(struct io_expander_dev *dev, int pin,
                 hal_gpio_irq_handler_t handler, void *arg,
                 hal_gpio_irq_trig_t trig, hal_gpio_pull_t pull)
{
    int rc;
    uint8_t reg;
    assert(pull == HAL_GPIO_PULL_NONE);
    assert(pin < IO_EXPANDER_MAX_IRQ);

    struct io_expander_irq * irq = &(dev->ioexp_irqs[pin]);
    irq->enabled = 1;
    irq->trig = trig;
    irq->func = handler;
    irq->arg = arg;

    rc = mcp23008_read_reg(dev, MCP23008_GPINTEN_REG, &reg);
    assert(rc == 0);
    reg |= (1 << pin);
    rc = mcp23008_write_reg(dev, MCP23008_GPINTEN_REG, reg);
    assert(rc == 0);

    MCP23008_DEBUG("Intr init pin %d\n", pin);
    return 0;
}

static int
mcp23008_irq_release(struct io_expander_dev *dev, int pin)
{
    int rc;
    uint8_t reg;
    assert(pin < IO_EXPANDER_MAX_IRQ);

    struct io_expander_irq * irq = &(dev->ioexp_irqs[pin]);
    irq->enabled = 0;
    irq->trig = HAL_GPIO_TRIG_NONE;
    irq->func = 0;
    irq->arg = 0;

    rc = mcp23008_read_reg(dev, MCP23008_GPINTEN_REG, &reg);
    assert(rc == 0);
    reg &= ~((uint8_t)1 << pin);
    rc = mcp23008_write_reg(dev, MCP23008_GPINTEN_REG, reg);
    assert(rc == 0);

    return 0;
}

static int
mcp23008_irq_enable(struct io_expander_dev *dev, int pin)
{
    assert(pin < IO_EXPANDER_MAX_IRQ);

    struct io_expander_irq * irq = &(dev->ioexp_irqs[pin]);
    irq->enabled = 1;
    return 0;
}

static int
mcp23008_irq_disable(struct io_expander_dev *dev, int pin)
{
    assert(pin < IO_EXPANDER_MAX_IRQ);

    struct io_expander_irq * irq = &(dev->ioexp_irqs[pin]);
    irq->enabled = 0;
    return 0;
}


static void
mcp23008_irq(void *arg)
{
    struct mcp23008_io_expander_dev_cfg *cfg;
    struct io_expander_dev *dev = (struct io_expander_dev *)arg;
    assert(arg);
    cfg = (struct mcp23008_io_expander_dev_cfg *)((struct os_dev*)dev)->od_init_arg;
    MCP23008_STATS_INC(n_irq);

    if (cfg->interrupt_eventq) {
        os_eventq_put(cfg->interrupt_eventq, &cfg->interrupt_event);
    }
}

/**
 *  Event callback that is called everytime an input-pin changes
 *  If an input pin has a callback associated with the current
 *  change (rising / falling) call this is called.
 **/
static void
mcp23008_interrupt_ev_cb(struct os_event *ev)
{
    int rc, i;
    uint8_t pin_input, changed_pins;
    struct mcp23008_io_expander_dev_cfg *cfg;
    struct io_expander_dev *dev = (struct io_expander_dev *) ev->ev_arg;

    cfg = (struct mcp23008_io_expander_dev_cfg *)((struct os_dev*)dev)->od_init_arg;
    /* Read current state of pins */
    rc = mcp23008_read_in8(dev, &pin_input);
    if (rc) {
        MCP23008_ERR("Failed to get pin values ioexp[%d]\n", cfg->inst_id);
        return;
    }

    changed_pins = (cfg->last_interrupt_input) ^ pin_input;
    //MCP23008_INFO("%x\n", changed_pins);
    cfg->last_interrupt_input = pin_input;

    /* Check for any activated interrupts */
    rc = 1;
    for (i=0;i<IO_EXPANDER_MAX_IRQ;i++) {
        if (dev->ioexp_irqs[i].enabled == 0) rc=0;

        if ((changed_pins & (1<<i)) == 0) continue;
#if MYNEWT_VAL(MCP23008_INT_STATS)
        switch(i) {
        case (0x0): MCP23008_STATS_INC(p0irqs);break;
        case (0x1): MCP23008_STATS_INC(p1irqs);break;
        case (0x2): MCP23008_STATS_INC(p2irqs);break;
        case (0x3): MCP23008_STATS_INC(p3irqs);break;
        case (0x4): MCP23008_STATS_INC(p4irqs);break;
        case (0x5): MCP23008_STATS_INC(p5irqs);break;
        case (0x6): MCP23008_STATS_INC(p6irqs);break;
        case (0x7): MCP23008_STATS_INC(p7irqs);break;
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
 * @param2 mcp23008 io expander device cfg ptr
 * @return OS_OK on success
 */
int
mcp23008_io_expander_dev_init(struct os_dev *odev, void *arg)
{
    struct io_expander_dev *dev;
    struct io_expander_driver_funcs *iof;

    dev = (struct io_expander_dev *)odev;
    dev->number_of_pins = 8;

    log_register((char*)odev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    iof = &dev->iof_funcs;

    iof->iof_init_in = mcp23008_init_in;
    iof->iof_init_out = mcp23008_init_out;
    iof->iof_pin_dir = mcp23008_pin_dir;
    iof->iof_write = mcp23008_write_out;
    iof->iof_read = mcp23008_read;

    iof->iof_irq_init = mcp23008_irq_init;
    iof->iof_irq_release = mcp23008_irq_release;
    iof->iof_irq_enable = mcp23008_irq_enable;
    iof->iof_irq_disable = mcp23008_irq_disable;

    /* Clear interrupt structure */
    memset(dev->ioexp_irqs, 0, sizeof(dev->ioexp_irqs));

    return (OS_OK);
}

int
mcp23008_io_expander_cfg(struct io_expander_dev *dev)
{
    int rc;
    struct mcp23008_io_expander_dev_cfg *cfg;
    cfg = (struct mcp23008_io_expander_dev_cfg *)((struct os_dev*)dev)->od_init_arg;
    assert(cfg != NULL);

#if MYNEWT_VAL(MCP23008_STATS)
    static char *stats_name[] = {
        "io_exp0",
        "io_exp1",
    };

    /* Init stats */
    assert(cfg->inst_id < 2);
    rc = stats_init_and_reg(
        STATS_HDR(g_mcp23008_stats), STATS_SIZE_INIT_PARMS(g_mcp23008_stats,
        STATS_SIZE_32), STATS_NAME_INIT_PARMS(mcp23008_stats), stats_name[cfg->inst_id]);
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif

    cfg->interrupt_event.ev_cb = mcp23008_interrupt_ev_cb;
    cfg->interrupt_event.ev_arg = (void*) dev;
    hal_gpio_irq_init(cfg->irq_pin, mcp23008_irq, (void*) dev,
                      HAL_GPIO_TRIG_FALLING, HAL_GPIO_PULL_UP);
    hal_gpio_irq_enable(cfg->irq_pin);
    if (!cfg->interrupt_eventq) {
        cfg->interrupt_eventq = os_eventq_dflt_get();
    }

    /* HW Reset */
    hal_gpio_init_out(cfg->rst_pin, 0);
    os_cputime_delay_usecs(1000);
    hal_gpio_init_in(cfg->rst_pin, HAL_GPIO_PULL_UP);
    os_cputime_delay_usecs(1000);

    /* Read input in order to clear any outstanding interrupt */
    uint8_t pin_input;
    rc = mcp23008_read_in8(dev, &pin_input);
    if (rc) {
        MCP23008_ERR("Fail to read init inp state\n");
    }
    cfg->last_interrupt_input = pin_input;


    /* Test writing to and reading back from INTCON */
    uint8_t reg;
    rc = mcp23008_read_reg(dev, MCP23008_INTCON_REG, &reg);
    SYSINIT_PANIC_ASSERT(rc == 0);
    rc = mcp23008_write_reg(dev, MCP23008_INTCON_REG, 0xA5);
    SYSINIT_PANIC_ASSERT(rc == 0);
    rc = mcp23008_read_reg(dev, MCP23008_INTCON_REG, &reg);
    SYSINIT_PANIC_ASSERT(rc == 0);
    SYSINIT_PANIC_ASSERT(reg == 0xA5);
    rc = mcp23008_write_reg(dev, MCP23008_INTCON_REG, 0x00);
    SYSINIT_PANIC_ASSERT(rc == 0);

    /* Pins are default input, so only outputs needs to be set */
    rc = mcp23008_init_out8(dev, ~(cfg->direction));
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = mcp23008_init_in8(dev, cfg->direction, cfg->pull_up);
    SYSINIT_PANIC_ASSERT(rc == 0);

    return (OS_OK);
}

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
static void
mcp23008_init_node_cb(struct bus_node *bnode, void *arg)
{
    int err = mcp23008_io_expander_dev_init((struct os_dev *)bnode, arg);
    assert(!err);
}

static struct bus_node_callbacks mcp23008_bus_node_cbs = {
        .init = mcp23008_init_node_cb,
    };

int
mcp23008_io_expander_create_i2c_dev(struct bus_i2c_node *node, const char *name,
                                    struct mcp23008_io_expander_dev_cfg *cfg)
{
    bus_node_set_callbacks((struct os_dev *)node, &mcp23008_bus_node_cbs);
    // this will call struct x_dev_init()
    return bus_i2c_node_create(name, node, &cfg->i2c_node_cfg, cfg);
}
#endif
