
#ifndef __SENSOR_SGP30_H__
#define __SENSOR_SGP30_H__

#include "os/mynewt.h"
#include "sensor/sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

struct sgp30_cfg {
};

struct sgp30 {
    struct os_dev dev;
    struct sensor sensor;
    struct sgp30_cfg cfg;
};

int sgp30_init(struct os_dev *, void *arg);
int sgp30_config(struct sgp30 *sgp30, const struct sgp30_cfg *cfg);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_SGP30_H__ */
