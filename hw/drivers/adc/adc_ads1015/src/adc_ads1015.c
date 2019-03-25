#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "defs/error.h"
#include "hal/hal_i2c.h"
#include "os/os.h"
#include "sysinit/sysinit.h"

#include "adc_ads1015/adc_ads1015.h"
#include "adc_ads1015_priv.h"

#include "log/log.h"
#include "stats/stats.h"

#include <shell/shell.h>
#include <console/console.h>

#include <adc/adc.h>

#define LOG_MODULE_ADS1015 (115)
#define ADS1015_INFO(...) LOG_INFO(&_log, LOG_MODULE_ADS1015, __VA_ARGS__)
#define ADS1015_DEBUG(...) LOG_DEBUG(&_log, LOG_MODULE_ADS1015, __VA_ARGS__)
#define ADS1015_ERR(...) LOG_ERROR(&_log, LOG_MODULE_ADS1015, __VA_ARGS__)
static struct log _log;

static uint8_t ads1015_adc_chans[ADS1015_CHANNEL_COUNT * sizeof(struct adc_chan_config)];

static int
read_reg(struct ads1015_adc_dev_cfg *cfg, uint8_t addr, uint16_t *val)
{
    os_error_t err = 0;
    uint8_t inbuf[2];
    struct hal_i2c_master_data data_struct = {
        .address = cfg->i2c_addr, .len = 1, .buffer=&addr
    };

    if (os_started())
    {
        err = os_mutex_pend(cfg->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK)
        {
            return err;
        }
    }
    
    /* Register write */
    int rc = hal_i2c_master_write(cfg->i2c_num, &data_struct, OS_TICKS_PER_SEC / 10, 0);
    if (rc != 0) {
        ADS1015_ERR("Wr err\n");
        goto exit;
    }

    /* Read one byte back */
    data_struct.buffer = inbuf;
    data_struct.len = 2;
    rc = hal_i2c_master_read(cfg->i2c_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    if (rc != 0) {
        ADS1015_ERR("Rd err\n");
    }
exit:    
    if (os_started())
    {
        err = os_mutex_release(cfg->i2c_mutex);
        assert(err == OS_OK);
    }
    *val = ((uint16_t)inbuf[0])*256 + (uint16_t)inbuf[1];
    
    return rc;
}


static int
write_reg(struct ads1015_adc_dev_cfg *cfg, uint8_t addr, uint16_t value)
{
    int rc;
    os_error_t err = 0;
    uint8_t *val_ptr = (uint8_t *)&value;
    uint8_t payload[3] = { addr, val_ptr[1], val_ptr[0]};

    struct hal_i2c_master_data data_struct = {
        .address = cfg->i2c_addr, .len = 3, .buffer = payload
    };

    if (os_started()) {
        err = os_mutex_pend(cfg->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK) {
            return err;
        }
    }
    rc = hal_i2c_master_write(cfg->i2c_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    if (rc != 0) {
        ADS1015_ERR("Wr err\n", addr);
    }

    if (os_started()) {
        err = os_mutex_release(cfg->i2c_mutex);
        assert(err == OS_OK);
    }
    return rc;
}

static int
configure_channel(struct adc_dev *dev, uint8_t channel, void *arg)
{
    int rc;
    uint16_t config;
    struct ads1015_adc_dev_cfg *cfg;

    cfg = (struct ads1015_adc_dev_cfg *)dev->ad_dev.od_init_arg;
    rc = read_reg(cfg, ADS1015_REG_POINTER_CONFIG, &config);
    if (rc != 0) {
        goto err;
    }

    switch (channel)
    {
    case (0):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
        break;
    case (1):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
        break;
    case (2):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
        break;
    case (3):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
        break;
    default:
        return OS_EINVAL;
    }
    
    rc = write_reg(cfg, ADS1015_REG_POINTER_CONFIG, config);
    if (rc != 0) {
        goto err;
    }

    uint16_t refmv=0;
    switch (cfg->gain)
    {
    case (ADS1015_REG_CONFIG_PGA_6_144V):
        refmv = 3;
        break;
    case (ADS1015_REG_CONFIG_PGA_4_096V):
        refmv = 2;
        break;
    case (ADS1015_REG_CONFIG_PGA_2_048V):
        refmv = 1;
        break;
    default:
        ADS1015_DEBUG("Gain gives refmv<1\n");
        return OS_EINVAL;
        break;
    }
    
    dev->ad_chans[channel].c_res = 12;
    dev->ad_chans[channel].c_refmv = refmv;
    dev->ad_chans[channel].c_configured = 1;
err:    
    return rc;
}

static int
read_channel(struct adc_dev *dev, uint8_t channel, int *val)
{
    int rc;
    uint16_t config, results;    
    struct ads1015_adc_dev_cfg *cfg;

    cfg = (struct ads1015_adc_dev_cfg *)dev->ad_dev.od_init_arg;

    config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
        ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
        ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
        ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
        ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
        ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

    // Set PGA/voltage range
    config &= ~(ADS1015_REG_CONFIG_PGA_MASK);
    config |= cfg->gain;

    rc = write_reg(cfg, ADS1015_REG_POINTER_CONFIG, config);
    if (rc != 0) {
        ADS1015_DEBUG("Err set bcfg\n");
        goto err;
    }

    /* Select channel */
    rc = configure_channel(dev, channel, 0);
    if (rc != 0) {
        ADS1015_DEBUG("Err cfg ch\n");
        goto err;
    }
    
    rc = read_reg(cfg, ADS1015_REG_POINTER_CONFIG, &config);
    if (rc != 0) {
        ADS1015_DEBUG("Err read-back cfg\n");
        goto err;
    }
    
    // Set 'start single-conversion' bit
    config |= ADS1015_REG_CONFIG_OS_SINGLE;
    
    // Write config register to the ADC
    rc = write_reg(cfg, ADS1015_REG_POINTER_CONFIG, config);
    if (rc != 0) {
        ADS1015_DEBUG("Err st conv\n");
    }

    /* Wait for the conversion to complete */
    os_cputime_delay_usecs(cfg->conversion_delay*1000);
    
    // Read the conversion results
    rc = read_reg(cfg, ADS1015_REG_POINTER_CONVERT, &results);
    if (rc != 0) {
        ADS1015_DEBUG("Err rd res\n");
        goto err;
    }

    // Shift 12-bit results right 4 bits for the ADS1015
    results = (results >> cfg->bit_shift);
    if (val) {
        *val = results;
    }
    return 0;
err:
    return rc;
}

/**
 * Callback to initialize an adc_dev structure from the os device
 * initialization callback.  This sets up a ads1015_adc_device(), so
 * that subsequent lookups to this device allow us to manipulate it.
 */
int
ads1015_adc_dev_init(struct os_dev *odev, void *arg)
{
    struct adc_dev *dev;
    struct adc_driver_funcs *af;
    struct ads1015_adc_dev_cfg *cfg = (struct ads1015_adc_dev_cfg *) arg;
    assert(cfg != NULL);

    dev = (struct adc_dev *) odev;

    os_mutex_init(&dev->ad_lock);

    dev->ad_chans = (void *) ads1015_adc_chans;
    dev->ad_chan_count = ADS1015_CHANNEL_COUNT;

    log_register(odev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    //OS_DEV_SETHANDLERS(odev, nrf52_adc_open, nrf52_adc_close);
    af = &dev->ad_funcs;

    af->af_configure_channel = configure_channel;
    af->af_sample = 0;
    af->af_read_channel = read_channel;
    af->af_set_buffer = 0;
    af->af_release_buffer = 0;
    af->af_read_buffer = 0;
    af->af_size_buffer = 0;

    return (0);
}


int
ads1015_adc_hwtest(struct adc_dev *dev)
{
    int rc = 0;
    struct ads1015_adc_dev_cfg *cfg;
    uint16_t config, readback;
    
    cfg = (struct ads1015_adc_dev_cfg *)dev->ad_dev.od_init_arg;
    // Start with default values
    config = ADS1015_REG_CONFIG_CQUE_2CONV  | // Disable the comparator (default val)
        ADS1015_REG_CONFIG_CLAT_LATCH  | // Non-latching (default val)
        ADS1015_REG_CONFIG_CPOL_ACTVHI | // Alert/Rdy active low   (default val)
        ADS1015_REG_CONFIG_CMODE_WINDOW  | // Traditional comparator (default val)
        ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
        ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)
    rc = write_reg(cfg, ADS1015_REG_POINTER_CONFIG, config);
    assert(rc==0);
    rc = read_reg(cfg, ADS1015_REG_POINTER_CONFIG, &readback);
    assert(rc==0);

    if (config != (readback & 0x7FFF)) /* bit 15 is a status bit */
    {
        ADS1015_DEBUG("Err cfg %x rdback %x\r\n", config, readback);
        return 1;
    }
    return 0;
}

