#include <assert.h>
#include <errno.h>
#include <string.h>

#include "defs/error.h"
#include "hal/hal_i2c.h"
#include "i2cn/i2cn.h"
#include "os/os.h"
#include "sysinit/sysinit.h"

#include "sensor/sensor.h"

#include "sensor/voc.h"

#include "log/log.h"
#include "stats/stats.h"

#include "sgp40/sgp40.h"
// wrapping the sensiron dirver
#include "sensirion_sgp40.h"

#if 1
#define LOG_MODULE_SGP40 (30)
#define SGP40_LOG_INFO(...) LOG_INFO(&_log, LOG_MODULE_SGP40, __VA_ARGS__)
#define SGP40_LOG_ERROR(...) LOG_ERROR(&_log, LOG_MODULE_SGP40, __VA_ARGS__)
#define SGP40_LOG_DEBUG(...) LOG_DEBUG(&_log, LOG_MODULE_SGP40, __VA_ARGS__)
#else
#include "console/console.h"
#define SGP40_LOG_ERROR(fmt, ...) console_printf("sgp40:E:" fmt, ##__VA_ARGS__)
#define SGP40_LOG_DEBUG(fmt, ...) console_printf("sgp40:D:" fmt, ##__VA_ARGS__)
#endif

static struct log _log;

#if 0
static inline struct sgp40_priv * sgp40_get_priv(struct sgp40 *sgp40)
{
    _Static_assert(sizeof(sgp40->_priv) >= sizeof(struct sgp40_priv), "");
    return (struct sgp40_priv *) sgp40->_priv;
}
#endif

static int sgp40_sd_set_config(struct sensor *sensor, void *cfg)
{
    struct sgp40 *sgp40         = (struct sgp40 *)SENSOR_GET_DEVICE(sensor);
    struct sgp40_cfg *sgp40_cfg = cfg;

    return sgp40_config(sgp40, sgp40_cfg);
}

static int sgp40_sd_read(struct sensor *sensor, sensor_type_t sensor_type,
                         sensor_data_func_t cb_func, void *cb_arg,
                         uint32_t timeout)
{
    int err             = 0;
    struct sgp40 *sgp40 = (struct sgp40 *)SENSOR_GET_DEVICE(sensor);
    (void)sgp40;

    if (!(sensor_type & SENSOR_TYPE_VOC)) {
        return -1;
    }

    struct sensor_voc_data svd = { 0 };

    uint16_t tvoc_ppb   = 0;
    uint16_t co2_eq_ppm = 0;
    err = sgp40_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
    if (err)
        return err;

    /* Total Volatile Organic Compounds in ppb */
    svd.svd_tvoc_is_valid = 1;
    svd.svd_tvoc          = tvoc_ppb;

    /* carbon dioxide equivalent in ppm */
    svd.svd_co2eq_is_valid = 1;
    svd.svd_co2eq          = co2_eq_ppm;
    // TODO? only retrive iaq_baseline every hour?
    /** TODO!? set_iaq_baseline (if stored measurement is < 1 week old) or only
     * set is_valid if retrived from chip and >= 1 h of "uptime"/measurement.
     */
    uint32_t iaq_baseline = 0;
    err                   = sgp40_get_iaq_baseline(&iaq_baseline);
    if (err) {
        svd.svd_iaqbl_is_valid = 0;
        svd.svd_iaqbl          = 0;
    } else {
        svd.svd_iaqbl_is_valid = 1;
        svd.svd_iaqbl          = iaq_baseline;
    }

    err = cb_func(sensor, cb_arg, &svd, SENSOR_TYPE_VOC);
    if (err)
        return err;

    return 0;
}

static int sgp40_sd_get_config(struct sensor *sensor, sensor_type_t sensor_type,
                               struct sensor_cfg *cfg)
{

    if (sensor_type & SENSOR_TYPE_VOC) {
        cfg->sc_valtype = SENSOR_VALUE_TYPE_OPAQUE;
    }

    return 0;
}

static struct sensor_driver sgp40_sensor_driver = {
    .sd_read       = sgp40_sd_read,
    .sd_get_config = sgp40_sd_get_config,
    .sd_set_config = sgp40_sd_set_config,
    //.sd_set_trigger_thresh = sgp40_sd_set_trigger_thresh,
    //.sd_set_notification   = sgp40_sd_set_notification,
    //.sd_unset_notification = sgp40_sd_unset_notification,
    //.sd_handle_interrupt   = sgp40_sd_handle_interrupt,
};

int sgp40_config(struct sgp40 *sgp40, const struct sgp40_cfg *cfg)
{
    int err               = 0;
    struct sensor *sensor = &sgp40->sensor;

    /* wait for chip to boot after power reset.*/
    static const int attempts_per_sec = 8;
    for (int i = 0; i <= (2 * attempts_per_sec); i++) {
        err = sgp40_probe();
        if (!err)
            break;
        else
            os_time_delay(OS_TICKS_PER_SEC / attempts_per_sec);
    }

    if (err) {
        SGP40_LOG_ERROR("sgp40_probe rc=%d\n", err);
        return err;
    }

    if (MYNEWT_VAL(SGP40_DEBUG_ENABLE)) {

        const char *driver_ver = sgp40_get_driver_version();
        if (driver_ver) {
            SGP40_LOG_DEBUG("Sensirion driver version %s\n", driver_ver);
        }

        uint16_t feature_set_version = 0;
        uint8_t product_type         = 0;
        err = sgp40_get_feature_set_version(&feature_set_version,
                                            &product_type);
        if (!err) {
            SGP40_LOG_DEBUG("Feature set version: %u\n", feature_set_version);
            SGP40_LOG_DEBUG("Product type: %u\n", product_type);
        }

        uint64_t serial_id = 0;
        err                = sgp40_get_serial_id(&serial_id);
        if (!err) {
            SGP40_LOG_DEBUG("SerialID: %" PRIu64 "\n", serial_id);
        }

        uint16_t ethanol_raw_signal = 0;
        uint16_t h2_raw_signal      = 0;
        err = sgp40_measure_raw_blocking_read(&ethanol_raw_signal,
                                              &h2_raw_signal);
        if (!err) {
            SGP40_LOG_DEBUG("Ethanol raw signal: %u\n", ethanol_raw_signal);
            SGP40_LOG_DEBUG("H2 raw signal: %u\n", h2_raw_signal);
        }
    }

    err = sgp40_iaq_init();
    assert(!err);

    err = sensor_set_type_mask(sensor, SENSOR_TYPE_VOC);
    assert(!err);

    return 0;
}

int sgp40_init(struct os_dev *dev, void *arg)
{
    int err               = 0;
    struct sgp40 *sgp40   = (struct sgp40 *)dev;
    struct sensor *sensor = &sgp40->sensor;

    err = log_register("sgp40", &_log, &log_console_handler, NULL,
                       LOG_SYSLEVEL);
    assert(!err);

    err = sensor_init(sensor, dev);
    assert(!err);
    err = sensor_set_driver(sensor, SENSOR_TYPE_VOC, &sgp40_sensor_driver);
    assert(!err);

    err = sensor_set_interface(sensor, arg);
    assert(!err);

    err = sensor_mgr_register(sensor);
    assert(!err);

    // note: itf is initalized above
    struct sensor_itf *itf = SENSOR_GET_ITF(sensor);

    // probalby does nothing.
    sensirion_i2c_init();

    //
    err = sensirion_i2c_select_bus(itf->si_num);
    assert(!err);

    // unfortenly sgp40 address is hardcoded. verify it.

    uint8_t hardcoded_addr = sgp40_get_configured_address();
    assert(hardcoded_addr == itf->si_addr);

    return 0;
}
