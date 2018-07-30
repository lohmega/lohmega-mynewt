#ifndef ZOEM8_H_
#define ZOEM8_H_

#include <gps/gps.h>

#ifdef __cplusplus
extern "C" {
#endif

struct os_mutex;

struct zoem8_gps_dev_cfg {
    uint8_t  i2c_num;
    uint8_t  i2c_addr;
    struct os_mutex *i2c_mutex;
};

int zoem8_gps_init(struct os_dev *odev, void *arg);
int zoem8_gps_config(struct gps_dev *dev, void *arg);

#ifdef __cplusplus
}
#endif

#endif
