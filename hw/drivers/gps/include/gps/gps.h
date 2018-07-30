#ifndef __GPS_H__
#define __GPS_H__

#include "os/os_dev.h"
#include "gps/nmea.h"

#ifdef __cplusplus
extern "C" {
#endif

struct gps_dev;

typedef int
(*gps_get_time_func_t)(struct gps_dev *dev, struct nmea_time *tm);

typedef int
(*gps_set_systime_func_t)(struct gps_dev *dev);

typedef int
(*gps_update_func_t)(struct gps_dev *dev, int *nmea_updates);

typedef void gps_nmea_updated_cb(struct nmea *ns);

    
struct gps_driver_funcs {
    gps_get_time_func_t gf_get_time;
    gps_set_systime_func_t gf_set_systime;
    gps_update_func_t gf_update;
};

struct gps_dev {
    struct os_dev gps_dev;           /* Has to be here for cast in create_dev to work*/
    struct gps_driver_funcs gf_funcs;
    struct nmea ns;
};

/**
 * Read time from gps.
 *
 * @param dev The GPS device
 * @param tm  gps_time structure pointer
 *
 * @return 0 on success, non-zero error code on failure.
 */
static inline int
gps_get_time(struct gps_dev *dev, struct nmea_time *tm)
{
    return (dev->gf_funcs.gf_get_time(dev, tm));
}

/**
 * Set the systemtime from gps if gpstime is valid.
 *
 * @param dev The GPS device
 *
 * @return 0 on success, non-zero error code on failure.
 */
static inline int
gps_set_systime(struct gps_dev *dev)
{
    return (dev->gf_funcs.gf_set_systime(dev));
}

/**
 * Read and interpret current nmea buffer from GPS.
 *
 * @param dev The GPS device
 * @param nmea_s Nmea structure  
 *
 * @return 0 on success, non-zero error code on failure.
 */
static inline int
gps_update(struct gps_dev *dev, int *nmea_updates)
{
    return (dev->gf_funcs.gf_update(dev, nmea_updates));
}

int gps_cli_register(void);

void gps_task_kickoff(void);
void gps_task_stop(void);
void gps_set_cb(gps_nmea_updated_cb *cb);


    
#ifdef __cplusplus
}
#endif

#endif
