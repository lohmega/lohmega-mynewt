
#ifndef __SENSOR_SGP40_H__
#define __SENSOR_SGP40_H__

#include "os/mynewt.h"
#include "sensor/sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

struct sgp40_cfg {
};


struct sgp40 {
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    struct bus_i2c_node i2c_node;
#else
    struct os_dev dev;
#endif
    struct sensor sensor;
    struct sgp40_cfg cfg;

};

int sgp40_init(struct os_dev *, void *arg);
int sgp40_config(struct sgp40 *sgp40, const struct sgp40_cfg *cfg);

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
int sgp40_create_i2c_sensor_dev(struct bus_i2c_node *node, const char *name,
                              const struct bus_i2c_node_cfg *i2c_cfg,
                              struct sensor_itf *sensor_itf);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_SGP40_H__ */
