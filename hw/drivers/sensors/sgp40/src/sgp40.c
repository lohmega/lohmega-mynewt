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
#include "sensirion_voc_algorithm.h"

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
static VocAlgorithmParams sgp40_voc_alg_params;

static int sgp40_sd_set_config(struct sensor *sensor, void *cfg)
{
    struct sgp40 *sgp40 = (struct sgp40 *)SENSOR_GET_DEVICE(sensor);
    struct sgp40_cfg *sgp40_cfg = cfg;

    return sgp40_config(sgp40, sgp40_cfg);
}

#if 0
static int sgp40_measure_voc_index_with_rh_t(int32_t* voc_index,
                                              int32_t* relative_humidity,
                                              int32_t* temperature) 
{
    int32_t int_temperature, int_humidity;
    int16_t ret;
    uint16_t sraw;

    ret = shtc1_measure_blocking_read(&int_temperature, &int_humidity);
    if (ret)
        return SENSIRION_GET_RHT_SIGNAL_FAILED;

    if (temperature) {
        *temperature = int_temperature;
    }
    if (relative_humidity) {
        *relative_humidity = int_humidity;
    }

    ret = sgp40_measure_raw_with_rht_blocking_read(int_humidity,
                                                   int_temperature, &sraw);
    if (ret) {
        return SENSIRION_GET_SGP_SIGNAL_FAILED;
    }

    VocAlgorithm_process(&sgp40_voc_alg_params, sraw, voc_index);
    return 0;
}
#endif
static int sgp40_sd_read(struct sensor *sensor, sensor_type_t sensor_type,
                         sensor_data_func_t cb_func, void *cb_arg,
                         uint32_t timeout)
{
    int err             = 0;
    struct sgp40 *sgp40 = (struct sgp40 *)SENSOR_GET_DEVICE(sensor);
    (void)sgp40;

    struct sensor_itf *itf = SENSOR_GET_ITF(sensor);
    (void) itf;
    if (!(sensor_type & SENSOR_TYPE_VOC)) {
        return -1;
    }

    struct sensor_voc_data svd = { 0 };
    // TODO get rh and temp from other sensor
    float rh_pc = 50;
    float temp_C = 25;

    // Relative humidity in percent multiplied by 1000. 'milli percent'
    int32_t rh_mpc = rh_pc * 1000;
    // Temperature in degree Celsius multiplied by 1000. i.e. milli celisius
    int32_t temp_mC = temp_C * 1000;
    uint16_t sraw = 0;
    err = sgp40_measure_raw_with_rht_blocking_read(rh_mpc, temp_mC, &sraw);
    if (err)
        return err;
    int32_t voc_index = 0;
    VocAlgorithm_process(&sgp40_voc_alg_params, sraw, &voc_index);

    /* Total Volatile Organic Compounds in ppb */
    svd.svd_tvoc_is_valid = 1;
    svd.svd_tvoc = voc_index; // TODO confirm unit!

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

        //uint8_t serial_id[SGP40_SERIAL_ID_NUM_BYTES];
        uint64_t serial_id = 0;
        err  = sgp40_get_serial_id((uint8_t *)&serial_id);
        if (!err) {
            SGP40_LOG_DEBUG("SerialID: %" PRIu64 "\n", serial_id);
        }

    }

    err = sensor_set_type_mask(sensor, SENSOR_TYPE_VOC);
    assert(!err);

    return 0;
}


static void init_node_cb(struct bus_node *bnode, void *arg)
{
    struct sensor_itf *itf = arg;
    sgp40_init((struct os_dev *)bnode, itf);
}

static struct bus_node_callbacks sgp40_bus_node_cbs = {
   .init = init_node_cb,
};

int sgp40_create_i2c_sensor_dev(struct bus_i2c_node *node, const char *name,
                              const struct bus_i2c_node_cfg *i2c_cfg,
                              struct sensor_itf *sensor_itf)
{

    int rc;

    sensor_itf->si_dev = &node->bnode.odev;
    bus_node_set_callbacks((struct os_dev *)node, &sgp40_bus_node_cbs);

    rc = bus_i2c_node_create(name, node, i2c_cfg, sensor_itf);

    return rc;
#if 0
    struct lps33hw *dev = (struct lps33hw *)node;
    struct bus_node_callbacks cbs = {
        .init = init_node_cb,
    };
    int rc;

    dev->node_is_spi = false;

    sensor_itf->si_dev = &node->bnode.odev;
    bus_node_set_callbacks((struct os_dev *)node, &cbs);

    rc = bus_i2c_node_create(name, node, i2c_cfg, sensor_itf);
#endif

    return rc;
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
    sensirion_i2c_init(itf);

    VocAlgorithm_init(&sgp40_voc_alg_params);
    return 0;
}
