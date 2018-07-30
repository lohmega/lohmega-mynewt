#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <os/os.h>
#include <hal/hal_bsp.h>
#include <gps/gps.h>

static gps_nmea_updated_cb *gps_updated_cb = 0;
#if MYNEWT_VAL(GPS_TASK)

/* Task structures */
static struct os_task gps_task_task_str;
OS_TASK_STACK_DEFINE(gps_task_stack, MYNEWT_VAL(GPS_TASK_STACK_SZ));
static struct os_eventq gps_task_eventq;
static struct os_callout update_callout;
static bool update_enabled = false;

#define GPS_TIMER_RATE  (OS_TICKS_PER_SEC / MYNEWT_VAL(GPS_UPDATE_RATE))

static void
gps_timer_ev_cb(struct os_event *ev) {
    struct gps_dev *gpsdev;
    int nmea_updates = 0;
    gpsdev = (struct gps_dev *) os_dev_open("gps0", OS_TIMEOUT_NEVER, NULL);
    assert(gpsdev != NULL);
    gps_update(gpsdev, &nmea_updates);
    os_dev_close((struct os_dev*) gpsdev);

    if (nmea_updates && gps_updated_cb) {
        gps_updated_cb(&gpsdev->ns);
    }
    
    if (update_enabled) {
        os_callout_reset(&update_callout, GPS_TIMER_RATE);
    }
}
    
static void
gps_task(void *arg)
{
    /* Use a dedicate event queue for timer and interrupt events */
    os_eventq_init(&gps_task_eventq);
    assert(os_eventq_inited(&gps_task_eventq));

    os_callout_init(&update_callout, &gps_task_eventq,
                    gps_timer_ev_cb, NULL);

    while (1) {
        os_eventq_run(&gps_task_eventq);
    }
}


static int
gps_task_init(void)
{
    os_task_init(&gps_task_task_str, "gps", 
                 gps_task, 0, 
                 MYNEWT_VAL(GPS_TASK_PRIO), OS_WAIT_FOREVER, 
                 gps_task_stack, MYNEWT_VAL(GPS_TASK_STACK_SZ));

    return (OS_OK);
}

#endif /* GPS_TASK */

void
gps_task_kickoff(void)
{
#if MYNEWT_VAL(GPS_TASK)
    if (!update_enabled) {
        update_enabled = true;
        os_callout_reset(&update_callout, GPS_TIMER_RATE);
    }
#endif
}

void
gps_task_stop(void)
{
#if MYNEWT_VAL(GPS_TASK)
    update_enabled = false;
#endif
}

void
gps_set_cb(gps_nmea_updated_cb *cb)
{
    gps_updated_cb = cb;
}

void
gps_pkg_init(void)
{
#if MYNEWT_VAL(GPS_TASK)
    gps_task_init();
#endif
    
#if MYNEWT_VAL(GPS_CLI)
    gps_cli_register();
#endif
}    
