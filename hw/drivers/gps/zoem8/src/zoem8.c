#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "defs/error.h"
#include "hal/hal_i2c.h"
#include "os/os.h"
#include "base64/hex.h"
#include "sysinit/sysinit.h"
#include "log/log.h"
#include "stats/stats.h"

#include <shell/shell.h>
#include <console/console.h>
#include <datetime/datetime.h>

#include <gps/gps.h>
#include <gps/nmea.h>
#include <zoem8/zoem8.h>

/* Define the stats section and records */
STATS_SECT_START(zoem8_stats)
    STATS_SECT_ENTRY(read_errors)
    STATS_SECT_ENTRY(write_errors)
    STATS_SECT_ENTRY(mutex_errors)
    STATS_SECT_ENTRY(read_bytes)
STATS_SECT_END

/* Global variable used to hold stats data */
STATS_SECT_DECL(zoem8_stats) g_zoem8_stats;

/* Define stat names for querying */
STATS_NAME_START(zoem8_stats)
    STATS_NAME(zoem8_stats, read_errors)
    STATS_NAME(zoem8_stats, write_errors)
    STATS_NAME(zoem8_stats, mutex_errors)
    STATS_NAME(zoem8_stats, read_bytes)
STATS_NAME_END(zoem8_stats)

#define LOG_MODULE_ZOEM8 (108)
#define ZOEM8_DEBUG(...) LOG_DEBUG(&_log, LOG_MODULE_ZOEM8, __VA_ARGS__)
#define ZOEM8_INFO(...) LOG_INFO(&_log, LOG_MODULE_ZOEM8, __VA_ARGS__)
#define ZOEM8_ERR(...) LOG_ERROR(&_log, LOG_MODULE_ZOEM8, __VA_ARGS__)
static struct log _log;

#define ZOEM8_NR_BYTES_HIGH 0xFD
#define ZOEM8_NR_BYTES_LOW 0xFE


static int
read_reg(struct zoem8_gps_dev_cfg *cfg, uint8_t addr, uint8_t *val)
{
    os_error_t err = 0;
    struct hal_i2c_master_data data_struct = {
        .address = cfg->i2c_addr, .len = 1, .buffer=&addr
    };

    if (os_started())
    {
        err = os_mutex_pend(cfg->i2c_mutex, OS_WAIT_FOREVER);
        if (err != OS_OK)
        {
            STATS_INC(g_zoem8_stats, mutex_errors);
            return err;
        }
    }
    
    /* Register write */
    int rc = hal_i2c_master_write(cfg->i2c_num, &data_struct, OS_TICKS_PER_SEC / 10, 0);
    if (rc != 0) {
        STATS_INC(g_zoem8_stats, write_errors);
        goto exit;
    }

    /* Read one byte back */
    data_struct.buffer = val;
    data_struct.len = 1;
    rc = hal_i2c_master_read(cfg->i2c_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    if (rc != 0) {
        STATS_INC(g_zoem8_stats, read_errors);
    }
exit:    
    if (os_started())
    {
        err = os_mutex_release(cfg->i2c_mutex);
        assert(err == OS_OK);
    }
    return rc;
}

static int
read_reg_block(struct zoem8_gps_dev_cfg *cfg, uint8_t reg, uint8_t* block, int length)
{
    os_error_t err = 0;
    struct hal_i2c_master_data data_struct = {
        .address = cfg->i2c_addr, .len = 1, .buffer = &reg
    };

    err = os_mutex_pend(cfg->i2c_mutex, OS_WAIT_FOREVER);
    if (err != OS_OK)
    {
        ZOEM8_ERR("Mtx err=%d\n", err);
        STATS_INC(g_zoem8_stats, mutex_errors);
        return err;
    }

    /* Register write */
    int rc = hal_i2c_master_write(cfg->i2c_num, &data_struct, OS_TICKS_PER_SEC / 10, 0);
    if (rc != 0) {
        ZOEM8_ERR("Failed to set reg adress for block rd\n");
        STATS_INC(g_zoem8_stats, write_errors);
        goto exit;
    }
    data_struct.buffer = block;
    data_struct.len = length;
    rc = hal_i2c_master_read(cfg->i2c_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);

    if (rc != 0) {
        ZOEM8_ERR("Failed block rd, len 0x%02X\n", length);
        STATS_INC(g_zoem8_stats, read_errors);
    }
exit:
    err = os_mutex_release(cfg->i2c_mutex);
    assert(err == OS_OK);
    return rc;
}


uint16_t
zoem8_read_nr_bytes(struct zoem8_gps_dev_cfg *cfg)
{
    uint8_t nr_bytes_high, nr_bytes_low;
    uint16_t nr_bytes;
    int rc = read_reg(cfg, ZOEM8_NR_BYTES_HIGH, &nr_bytes_high);
    if (rc != 0) {
        return 0;
    }
    if (nr_bytes_high == 0xFF)
    {
        return 0;
    }
    rc = read_reg(cfg, ZOEM8_NR_BYTES_LOW, &nr_bytes_low);
    
    nr_bytes = ((uint16_t)nr_bytes_high*256) + nr_bytes_low;
    if (nr_bytes == 0xFFFF)
    {
        ZOEM8_INFO("gps num_bytes=0xffff\n");
        return 0;
    }
    return nr_bytes;
}

static const char *
parse_sentences(struct zoem8_gps_dev_cfg *cfg, char *input, int length,
                struct nmea *ns, int *nmea_updates) {
    char *cp;
    char *end;
    char *last_complete_end;
    
    cp = input;
    end = input + length;
    last_complete_end = input;
    while (cp < end && *cp) {
        if (*cp == '$') {
            if (nmea_updates) {
                (*nmea_updates)++;
            }
            last_complete_end = nmea_parse(cp, length-(cp-input), 1, ns);
            cp = last_complete_end + 1;
        } else {
            cp++;
        }
        if (*cp == 0 || *cp == 0xFF) {
            break;
        }
    }
    return last_complete_end;
}

#define ZOEM8_BUF_SIZE 128
int zoem8_update(struct gps_dev *dev, int *nmea_updates)
{
    int rc;
    int offset, read_nr_bytes, nr_bytes;
    const char *endp;
    uint8_t *buffer;
    struct zoem8_gps_dev_cfg *cfg;
    struct nmea *ns = &(dev->ns);

    buffer = (uint8_t*)malloc(ZOEM8_BUF_SIZE);
    if (buffer==0) {
        ZOEM8_ERR("Could not allocate buffer memory\n");
        return 1;
    }
    memset(buffer, 0xFF, sizeof(ZOEM8_BUF_SIZE));
    if (nmea_updates) {
        *nmea_updates = 0;
    }
    
    cfg = (struct zoem8_gps_dev_cfg *)dev->gps_dev.od_init_arg;
    
    nr_bytes = zoem8_read_nr_bytes(cfg);

    offset = 0;
    rc = read_nr_bytes = (nr_bytes > (ZOEM8_BUF_SIZE - 1)) ? (ZOEM8_BUF_SIZE-1): nr_bytes;
    if (rc!=0) {
        goto exit;
    }
    while (read_nr_bytes > 0) {
        STATS_INCN(g_zoem8_stats, read_bytes, read_nr_bytes);
        //ZOEM8_DEBUG("GPS Read %d\n", read_nr_bytes);
        assert(offset + read_nr_bytes < ZOEM8_BUF_SIZE);
        rc = read_reg_block(cfg, 0xFF, buffer+offset, read_nr_bytes);
        if (rc!=0) {
            goto exit;
        }

        buffer[read_nr_bytes+offset] = '\0';
        //ZOEM8_DEBUG("%s", buffer);
        endp = parse_sentences(cfg, (char*)buffer, read_nr_bytes, ns,
                               nmea_updates);

        if (endp - (const char*)buffer < read_nr_bytes)
        {
            offset = read_nr_bytes - (endp - (const char*)buffer);
            memcpy(buffer, endp, offset);
        }
        else
        {
            offset = 0;
        }
        
        nr_bytes = zoem8_read_nr_bytes(cfg);
        read_nr_bytes = nr_bytes > (ZOEM8_BUF_SIZE - 1 - offset) ? (ZOEM8_BUF_SIZE-1-offset): nr_bytes;
    }
exit:
    free(buffer);
    return rc;
}

static uint8_t *row_buffer = 0;
static int row_buffer_offs;
int zoem8_update_one(struct gps_dev *dev, int *nmea_updates)
{
    int read_nr_bytes, nr_bytes, interpreted_bytes;
    const char *endp;
    struct zoem8_gps_dev_cfg *cfg;
    struct nmea *ns = &(dev->ns);

    if (row_buffer == 0) {
        row_buffer = (uint8_t*)malloc(ZOEM8_BUF_SIZE);
        row_buffer_offs = 0;
        memset(row_buffer, 0xFF, sizeof(ZOEM8_BUF_SIZE));
    }
    if (row_buffer==0) {
        ZOEM8_ERR("Could not allocate buffer memory\n");
        return 1;
    }
    if (nmea_updates) {
        *nmea_updates = 0;
    }
    
    cfg = (struct zoem8_gps_dev_cfg *)dev->gps_dev.od_init_arg;
    
    nr_bytes = zoem8_read_nr_bytes(cfg);

    read_nr_bytes = (nr_bytes > (ZOEM8_BUF_SIZE - 1 - row_buffer_offs)) ?
        (ZOEM8_BUF_SIZE - 1 - row_buffer_offs) : nr_bytes;
    if (read_nr_bytes > 0) {
        STATS_INCN(g_zoem8_stats, read_bytes, read_nr_bytes);
        // ZOEM8_DEBUG("GPS Read %d offs %d\n", read_nr_bytes, row_buffer_offs);

        read_reg_block(cfg, 0xFF, row_buffer + row_buffer_offs,
                       read_nr_bytes);
        row_buffer[read_nr_bytes+row_buffer_offs] = '\0';

        // ZOEM8_DEBUG("gps:'%s'\n", row_buffer);
        endp = parse_sentences(cfg, (char*)row_buffer, read_nr_bytes, ns,
                               nmea_updates);
        interpreted_bytes = endp - (const char*)row_buffer;
        // ZOEM8_DEBUG("interpreted: %d\n", interpreted_bytes);
        if (interpreted_bytes && interpreted_bytes < read_nr_bytes)
        {
            row_buffer_offs = read_nr_bytes - interpreted_bytes;
            memcpy(row_buffer, endp, row_buffer_offs);
            row_buffer[row_buffer_offs] = '\0';
            // ZOEM8_DEBUG("post memcpy: '%s'\n", row_buffer);
        } else if (interpreted_bytes==0) {
            row_buffer_offs += read_nr_bytes;
        } else {
            row_buffer_offs = 0;
        }
        // ZOEM8_DEBUG("gps: offs: %d\n", row_buffer_offs);
    }
    if (row_buffer_offs==0 || row_buffer_offs==(ZOEM8_BUF_SIZE - 1)) {
        free(row_buffer);
        row_buffer = 0;
    }
    return 0;
}


int
zoem8_set_systime(struct gps_dev *dev)
{
    char tstring[32];
    struct os_timeval sys_offset;
    struct os_timeval gps_data_offset;
    struct os_timeval utctime;
    struct os_timeval gps_utctime;
    struct os_timeval gps_corrected_utctime;
    struct os_timezone timezone;
    struct os_timezone gps_timezone;
    
    if (dev->ns.time.updated_at_usec == 0)
    {
        ZOEM8_ERR("no valid gps time\n");
        goto err;
    }
    
    gps_data_offset.tv_sec = 0;
    gps_data_offset.tv_usec = os_get_uptime_usec() -
        dev->ns.time.updated_at_usec;
    while (gps_data_offset.tv_usec >= 1000000) {
        gps_data_offset.tv_sec++;
        gps_data_offset.tv_usec -= 1000000;
    }

    /* Check if the time was updated in the last second*/
    if (gps_data_offset.tv_sec)
    {
        ZOEM8_ERR("gps time too old\n");
        goto err;
    }
        
    snprintf(tstring, 32, "%04d-%02d-%02dT%02d:%02d:%02d",
             dev->ns.time.year, dev->ns.time.mon, dev->ns.time.mday,
             dev->ns.time.hour, dev->ns.time.min, dev->ns.time.sec);
    
    datetime_parse(tstring, &gps_utctime, &gps_timezone);

    ZOEM8_DEBUG("NMEA DateTime: %04d-%02d-%02dT", dev->ns.time.year,
        dev->ns.time.mon, dev->ns.time.mday);
    ZOEM8_DEBUG("%02d:%02d:%02d\n", dev->ns.time.hour,
                dev->ns.time.min, dev->ns.time.sec);
    
    /* Compensate for the age of the gps measurement */
    os_timeradd(&gps_utctime, &gps_data_offset, &gps_corrected_utctime);
    
    /* Check current system time */
    os_gettimeofday(&utctime, &timezone);
    
    /*  */
    os_timersub(&gps_corrected_utctime, &utctime, &sys_offset);
    ZOEM8_DEBUG("sys offset s:%lld us:%ld\n",
                sys_offset.tv_sec, sys_offset.tv_usec);
    
    os_settimeofday(&gps_corrected_utctime, &gps_timezone);
    
    return OS_OK;
err:
    return OS_ERROR;
}

int
zoem8_gps_init(struct os_dev *odev, void *arg)
{
    int rc=0;
    struct gps_driver_funcs *gf;
    struct gps_dev *dev;
    struct zoem8_gps_dev_cfg *cfg = (struct zoem8_gps_dev_cfg *) arg;
    assert(cfg != NULL);
    
    dev = (struct gps_dev *)odev;
    memset(&dev->ns, 0, sizeof(struct nmea));

    gf = &dev->gf_funcs;
    gf->gf_get_time = 0;
    gf->gf_set_systime = zoem8_set_systime;
    gf->gf_update = zoem8_update_one;

    log_register(odev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    return rc;
}


int
zoem8_gps_config(struct gps_dev *dev, void *arg)
{
    /* Init stats */
    int rc;
    rc = stats_init_and_reg(
        STATS_HDR(g_zoem8_stats), STATS_SIZE_INIT_PARMS(g_zoem8_stats,
        STATS_SIZE_32), STATS_NAME_INIT_PARMS(zoem8_stats), "zoem8");
    SYSINIT_PANIC_ASSERT(rc == 0);
    return 0;
}

