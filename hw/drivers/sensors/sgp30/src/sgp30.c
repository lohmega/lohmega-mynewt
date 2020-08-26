#include <string.h>
#include <errno.h>
#include <assert.h>

#include "defs/error.h"
#include "os/os.h"
#include "sysinit/sysinit.h"
#include "hal/hal_i2c.h"
#include "i2cn/i2cn.h"

#include "sensor/sensor.h"

#include "sensor/voc.h"

#include "log/log.h"
#include "stats/stats.h"

#include "sgp30/sgp30.h"
// wrapping the sensiron dirver
#include "sensirion_sgp30.h"



#define LOG_MODULE_SGP30    (30)
#define SGP30_LOG_INFO(...)    LOG_INFO(&_log, LOG_MODULE_SGP30, __VA_ARGS__)
#define SGP30_LOG_ERROR(...)   LOG_ERROR(&_log, LOG_MODULE_SGP30, __VA_ARGS__)
#define SGP30_LOG_DEBUG(...)   LOG_DEBUG(&_log, LOG_MODULE_SGP30, __VA_ARGS__)

static struct log _log;

#if 0
static inline struct sgp30_priv * sgp30_get_priv(struct sgp30 *sgp30)
{
    _Static_assert(sizeof(sgp30->_priv) >= sizeof(struct sgp30_priv), "");
    return (struct sgp30_priv *) sgp30->_priv;
}
#endif

static int sgp30_sd_set_config(struct sensor *sensor, void *cfg)
{
    struct sgp30 *sgp30 = (struct sgp30 *)SENSOR_GET_DEVICE(sensor);
    struct sgp30_cfg *sgp30_cfg = cfg;

    return sgp30_config(sgp30, sgp30_cfg);
}

static int
sgp30_sd_read(struct sensor *sensor,
                   sensor_type_t sensor_type,
                   sensor_data_func_t cb_func,
                   void *cb_arg,
                   uint32_t timeout)
{
    int err = 0;
    struct sgp30 *sgp30 = (struct sgp30 *)SENSOR_GET_DEVICE(sensor);
    (void) sgp30;

    if (!(sensor_type & SENSOR_TYPE_VOC)) {
        return -1;
    }

    struct sensor_voc_data svd = { 0 };

    uint16_t tvoc_ppb = 0;
    uint16_t co2_eq_ppm = 0;
    err = sgp30_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
    if (err)
        return err;
    
    /* Total Volatile Organic Compounds in ppb */
    svd.svd_tvoc_is_valid = 1;
    svd.svd_tvoc = tvoc_ppb;

    /* carbon dioxide equivalent in ppm */
    svd.svd_co2eq_is_valid = 1;
    svd.svd_co2eq = co2_eq_ppm;
    
    // TODO? only retrive iaq_baseline every hour?
    uint32_t iaq_baseline = 0;
    err = sgp30_get_iaq_baseline(&iaq_baseline);
    if (err)
        return err;

    svd.svd_iaqbl_is_valid = 1;
    svd.svd_iaqbl = iaq_baseline;

    err = cb_func(sensor, cb_arg, &svd, SENSOR_TYPE_ACCELEROMETER);
    if (err)
        return err;

    return 0;
}

static int sgp30_sd_get_config(struct sensor * sensor,
                         sensor_type_t sensor_type,
                         struct sensor_cfg * cfg)
{

    if (sensor_type & SENSOR_TYPE_VOC) {
        cfg->sc_valtype = SENSOR_VALUE_TYPE_OPAQUE;
    }

    return 0;
}


static struct sensor_driver sgp30_sensor_driver = {
    .sd_read               = sgp30_sd_read,
    .sd_get_config         = sgp30_sd_get_config,
    //.sd_set_config         = sgp30_sd_set_config,
    //.sd_set_trigger_thresh = sgp30_sd_set_trigger_thresh,
    //.sd_set_notification   = sgp30_sd_set_notification,
    //.sd_unset_notification = sgp30_sd_unset_notification,
    //.sd_handle_interrupt   = sgp30_sd_handle_interrupt,
};


int sgp30_config(struct sgp30 *sgp30, const struct sgp30_cfg *cfg)
{
    (void) sgp30_sd_set_config;
    int err = 0;

    sensor_type_t en_mask = (SENSOR_TYPE_VOC);

    struct sensor *sensor = &sgp30->sensor;

    if (MYNEWT_VAL(SGP30_DEBUG_ENABLE)) {

        const char *driver_ver = sgp30_get_driver_version();
        if (driver_ver) {
            SGP30_LOG_DEBUG("Sensirion driver version %s\n", driver_ver);
        }

        uint16_t feature_set_version = 0;
        uint8_t product_type = 0;
        err = sgp30_get_feature_set_version(&feature_set_version, &product_type);
        if (!err) {
            SGP30_LOG_DEBUG("Feature set version: %u\n", feature_set_version);
            SGP30_LOG_DEBUG("Product type: %u\n", product_type);
        }

        uint64_t serial_id = 0;
        err = sgp30_get_serial_id(&serial_id);
        if (!err) {
            SGP30_LOG_DEBUG("SerialID: %" PRIu64 "\n", serial_id);
        } 

        uint16_t ethanol_raw_signal = 0;
        uint16_t h2_raw_signal = 0;
        err = sgp30_measure_raw_blocking_read(&ethanol_raw_signal, &h2_raw_signal);
        if (!err) {
            SGP30_LOG_DEBUG("Ethanol raw signal: %u\n", ethanol_raw_signal);
            SGP30_LOG_DEBUG("H2 raw signal: %u\n", h2_raw_signal);
        }
    }

    
    err = sgp30_probe();
    if (err == SGP30_ERR_UNSUPPORTED_FEATURE_SET) {
        SGP30_LOG_ERROR("needs at feature set version >= 1.0 (0x20)\n");
        // TODO return err!?
    }
    else if (err) {
        return err;
    }
   
    err = sgp30_iaq_init();
    assert(!err);

    err = sensor_set_type_mask(sensor, en_mask);
    assert(!err);

    return 0;
}

int sgp30_init(struct os_dev *dev, void *arg)
{
    int err = 0;
    struct sgp30 *sgp30 = (struct sgp30 *)dev;
    struct sensor *sensor = &sgp30->sensor;
    struct sensor_itf *itf = SENSOR_GET_ITF(sensor);

    err = log_register("sgp30", &_log, &log_console_handler, NULL, LOG_SYSLEVEL);
    assert(!err);
    
    // probalby does nothing. 
    sensirion_i2c_init();

    // 
    err = sensirion_i2c_select_bus(itf->si_num);
    assert(!err);

    // unfortenly sgp30 address is hardcoded. verify it.
    uint8_t hardcoded_addr = sgp30_get_configured_address();
    assert(hardcoded_addr == itf->si_addr);

    err = sensor_init(sensor, dev);
    assert(!err);
    err = sensor_set_driver(sensor, 
                        SENSOR_TYPE_VOC,
                        &sgp30_sensor_driver);
    assert(!err);

    err = sensor_set_interface(sensor, arg);
    assert(!err);

    err = sensor_mgr_register(sensor);
    assert(!err);


    return 0;
}
