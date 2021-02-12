
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
    struct os_dev dev;
    struct sensor sensor;
    struct sgp40_cfg cfg;
};

int sgp40_init(struct os_dev *, void *arg);
int sgp40_config(struct sgp40 *sgp40, const struct sgp40_cfg *cfg);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_SGP40_H__ */
