/** This is the HAL implementation/port for the sensiron driver code */

#include "os/os.h"
#include <stdio.h>

#include "hal/hal_i2c.h"
#include "i2cn/i2cn.h"
#include "sensirion_arch_config.h"
#include "sensirion_i2c.h"


#define SENSIRION_RW_TIMEOUT (OS_TICKS_PER_SEC / 10)
static uint8_t g_sgp40_i2c_bus = 0;

void sensirion_i2c_init(void) 
{
}

void sensirion_i2c_release(void) 
{
}

int16_t sensirion_i2c_select_bus(uint8_t bus_idx)
{
    g_sgp40_i2c_bus = bus_idx;

    return 0;
}

int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count) 
{
    int err = 0;

    struct hal_i2c_master_data op = {
        .address = address,
        .len = count,
        .buffer = data
    };

    err = hal_i2c_master_read(g_sgp40_i2c_bus, &op, SENSIRION_RW_TIMEOUT, 1);

    return err ? -1 : 0;
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data,
                           uint16_t count) 
{
    int err = 0;

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

    return err ? -1 : 0;
}

/**
 * should delay at least the given time, but may also sleep longer.
 */
void sensirion_sleep_usec(uint32_t usec) 
{
    os_time_delay((OS_TICKS_PER_SEC * usec) / 1e6 + 1); // TODO overflow check
}

