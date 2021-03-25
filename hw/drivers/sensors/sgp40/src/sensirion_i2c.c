/** This is the HAL implementation/port for the sensiron driver code */

#include "os/os.h"
#include <stdio.h>

#include "sensor/sensor.h"

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
#include "bus/drivers/i2c_common.h"
#else
#include "hal/hal_i2c.h"
#include "i2cn/i2cn.h"
#endif

#include "sensirion_arch_config.h"
#include "sensirion_i2c.h"

#define SENSIRION_RW_TIMEOUT (OS_TICKS_PER_SEC / 10)
#define SENSIRION_RW_RETRIES (2)
#define SENSIRION_LOCK_TIMEOUT (1000)

static uint8_t g_sgp40_i2c_bus = 0;

// hack to make bus driver work with sensirion's i2c abstraction
struct sensor_itf *g_sgp40_itf = NULL;


static inline uint8_t get_itf_i2c_addr(const struct sensor_itf *itf)
{
    const struct os_dev *dev = itf->si_dev;

    //const struct bus_node *bnode = NULL;
    const struct bus_i2c_node *node = (const struct bus_i2c_node *)dev;
    return node->addr;
}

void sensirion_i2c_init(void *sensor_itf)
{
    assert(sensor_itf);
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    g_sgp40_itf = (struct sensor_itf *) sensor_itf;
#else
    g_sgp40_i2c_bus = itf->si_num;
    // unfortenly sgp40 address is hardcoded. verify it.
    uint8_t hardcoded_addr = sgp40_get_configured_address();
    assert(hardcoded_addr == itf->si_addr);
#endif
    (void) g_sgp40_i2c_bus;
}

void sensirion_i2c_release(void)
{
}

int16_t sensirion_i2c_select_bus(uint8_t bus_idx)
{
    return 0;
}

int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count) 
{
    int err = 0;

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    struct sensor_itf *itf = g_sgp40_itf;
    assert(itf);
    assert(address == get_itf_i2c_addr(itf));
    err = bus_node_simple_read(itf->si_dev, data, count);
#else
    struct hal_i2c_master_data op = {
        .address = address,
        .len = count,
        .buffer = data
    };

    err = hal_i2c_master_read(g_sgp40_i2c_bus, &op, SENSIRION_RW_TIMEOUT, 1);
#endif
    return err ? -1 : 0;
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data,
                           uint16_t count) 
{
    int err = 0;

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    struct sensor_itf *itf = g_sgp40_itf;
    assert(itf);
    assert(address == get_itf_i2c_addr(itf));
    err = bus_node_simple_write(itf->si_dev, data, count);

#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
    struct hal_i2c_master_data op = {
        .address = address,
        .len = count,
        .buffer = data
    };
#pragma GCC diagnostic pop

    err = hal_i2c_master_write(g_sgp40_i2c_bus, &op, SENSIRION_RW_TIMEOUT, 1);
#endif

    return err ? -1 : 0;
}

/**
 * should delay at least the given time, but may also sleep longer.
 */
void sensirion_sleep_usec(uint32_t usec) 
{
    os_time_delay((OS_TICKS_PER_SEC * usec) / 1e6 + 1); // TODO overflow check
}

