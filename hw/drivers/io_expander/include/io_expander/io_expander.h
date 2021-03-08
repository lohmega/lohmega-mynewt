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

#ifndef __IO_EXPANDER_H__
#define __IO_EXPANDER_H__

#include <os/os_dev.h>
#include <hal/hal_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

struct io_expander_dev;

/**
 * Configure an IO_EXPANDER in pin.
 *
 * @param The  IO_EXPANDER device to configure
 * @param The  pin number to configure
 * @param pull pull type, from hal_gpio
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*io_expander_init_in_func_t)(struct io_expander_dev *dev, int pin, hal_gpio_pull_t pull);

/**
 * Configure an IO_EXPANDER out pin.
 *
 * @param dev The IO_EXPANDER device to configure
 * @param pin Pin number to set as output
 * @param val Value to set pin
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*io_expander_init_out_func_t)(struct io_expander_dev *dev, int pin, int val);

/**
 * Read the direction of an IO_EXPANDER pin.
 *
 * @param dev The IO_EXPANDER device to configure
 * @param pin Pin number to set as output
 * @param *dir Current direction of pin, 0=output, 1=input
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*io_expander_pin_dir_func_t)(struct io_expander_dev *dev, int pin, int *dir);

/**
 * Write an IO_EXPANDER out pin.
 *
 * @param dev The IO_EXPANDER device
 * @param pin Pin number
 * @param val Value to set pin
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*io_expander_write_func_t)(struct io_expander_dev *dev, int pin, int val);

/**
 * Read an IO_EXPANDER pin.
 *
 * @param dev The IO_EXPANDER device
 * @param pin Pin number
 *
 * @return int 0: low, 1: high
 */
typedef int (*io_expander_read_func_t)(struct io_expander_dev *dev, int pin);

/**
 * Attach an Interrupt to an IO_EXPANDER pin.
 *
 * @param dev The IO_EXPANDER device
 * @param pin Pin number
 * @param handler interrupt callback function
 * @param trig type of interrupt
 * @param pull obs: only HAL_GPIO_PULL_NONE accepted
 *
 * @return int 0: low, 1: high
 */
typedef int (*io_expander_irq_init_func_t)(struct io_expander_dev *dev, int pin,
                                           hal_gpio_irq_handler_t handler, void *arg,
                                           hal_gpio_irq_trig_t trig, hal_gpio_pull_t pull);

/**
 * Release interrupt on IO_EXPANDER pin.
 *
 * @param dev The IO_EXPANDER device
 * @param pin Pin number
 *
 * @return int 0: low, 1: high
 */
typedef int (*io_expander_irq_release_func_t)(struct io_expander_dev *dev, int pin);

/**
 * Enable interrupt on IO_EXPANDER pin.
 *
 * @param dev The IO_EXPANDER device
 * @param pin Pin number
 *
 * @return int 0: low, 1: high
 */
typedef int (*io_expander_irq_enable_func_t)(struct io_expander_dev *dev, int pin);

/**
 * Disable interrupt on IO_EXPANDER pin.
 *
 * @param dev The IO_EXPANDER device
 * @param pin Pin number
 *
 * @return int 0: low, 1: high
 */
typedef int (*io_expander_irq_disable_func_t)(struct io_expander_dev *dev, int pin);


struct io_expander_driver_funcs {
    io_expander_init_in_func_t  iof_init_in;
    io_expander_init_out_func_t iof_init_out;
    io_expander_pin_dir_func_t iof_pin_dir;
    io_expander_write_func_t iof_write;
    io_expander_read_func_t iof_read;

    io_expander_irq_init_func_t iof_irq_init;
    io_expander_irq_release_func_t iof_irq_release;
    io_expander_irq_enable_func_t iof_irq_enable;
    io_expander_irq_disable_func_t iof_irq_disable;
};

/* Storage for IRQ callbacks. */
#define IO_EXPANDER_MAX_IRQ 16

struct io_expander_irq {
    hal_gpio_irq_trig_t trig;
    uint8_t enabled;
    hal_gpio_irq_handler_t func;
    void *arg;
};

struct io_expander_dev {
    struct os_dev ioexp_dev;    /* Has to be here for cast in create_dev to work*/
    struct os_mutex ioexp_lock;
    struct io_expander_driver_funcs iof_funcs;
    int number_of_pins;
    struct io_expander_irq ioexp_irqs[IO_EXPANDER_MAX_IRQ];
};

/**
 * Configure an IO_EXPANDER in pin.
 *
 * @param The  IO_EXPANDER device to configure
 * @param The  pin number to configure
 * @param pull pull type
 *
 * @return 0 on success, non-zero error code on failure.
 */
static inline int
io_expander_init_in(struct io_expander_dev *dev, int pin, hal_gpio_pull_t pull)
{
    return (dev->iof_funcs.iof_init_in(dev, pin, pull));
}

/**
 * Configure an IO_EXPANDER out pin.
 *
 * @param dev The IO_EXPANDER device to configure
 * @param pin Pin number to set as output
 * @param val Value to set pin
 *
 * @return 0 on success, non-zero error code on failure.
 */
static inline int
io_expander_init_out(struct io_expander_dev *dev, int pin, int val)
{
    return (dev->iof_funcs.iof_init_out(dev, pin, val));
}

/**
 * Read the direction of an IO_EXPANDER pin.
 *
 * @param dev The IO_EXPANDER device to configure
 * @param pin Pin number to set as output
 * @param *dir Current direction of pin, 0=output, 1=input
 *
 * @return 0 on success, non-zero error code on failure.
 */
static inline int
io_expander_pin_dir(struct io_expander_dev *dev, int pin, int *dir)
{
    return (dev->iof_funcs.iof_pin_dir(dev, pin, dir));
}

/**
 * Write an IO_EXPANDER out pin.
 *
 * @param dev The IO_EXPANDER device
 * @param pin Pin number
 * @param val Value to set pin
 *
 * @return 0 on success, non-zero error code on failure.
 */
static inline int
io_expander_write(struct io_expander_dev *dev, int pin, int val)
{
    return (dev->iof_funcs.iof_write(dev, pin, val));
}

/**
 * Read IO_EXPANDER pin.
 *
 * @param dev The IO_EXPANDER device
 * @param pin Pin number
 *
 * @return int 0: low, 1: high
 */
static inline int
io_expander_read(struct io_expander_dev *dev, int pin)
{
    return (dev->iof_funcs.iof_read(dev, pin));
}


/**
 * Attach an Interrupt to an IO_EXPANDER pin.
 *
 * @param dev The IO_EXPANDER device
 * @param pin Pin number
 * @param handler interrupt callback function
 * @param trig type of interrupt
 * @param pull obs: only HAL_GPIO_PULL_NONE accepted
 *
 * @return int 0: low, 1: high
 */
static inline int
io_expander_irq_init(struct io_expander_dev *dev, int pin, hal_gpio_irq_handler_t handler,
                     void *arg, hal_gpio_irq_trig_t trig, hal_gpio_pull_t pull)
{
    return (dev->iof_funcs.iof_irq_init(dev, pin, handler, arg, trig, pull));
}

/**
 * Release interrupt on IO_EXPANDER pin.
 *
 * @param dev The IO_EXPANDER device
 * @param pin Pin number
 *
 * @return int 0: low, 1: high
 */
static inline void
io_expander_irq_release(struct io_expander_dev *dev, int pin)
{
    (dev->iof_funcs.iof_irq_release(dev, pin));
}


static inline void
io_expander_irq_enable(struct io_expander_dev *dev, int pin)
{
    (dev->iof_funcs.iof_irq_enable(dev, pin));
}

static inline void
io_expander_irq_disable(struct io_expander_dev *dev, int pin)
{
    (dev->iof_funcs.iof_irq_disable(dev, pin));
}


#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
