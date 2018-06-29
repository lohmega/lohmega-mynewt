#include "sysinit/sysinit.h"
#include "os/os.h"

/**
 * Depending on the type of package, there are different
 * compilation rules for this directory.  This comment applies
 * to packages of type "app."  For other types of packages,
 * please view the documentation at http://mynewt.apache.org/.
 *
 * Put source files in this directory.  All files that have a *.c
 * ending are recursively compiled in the src/ directory and its
 * descendants.  The exception here is the arch/ directory, which
 * is ignored in the default compilation.
 *
 * The arch/<your-arch>/ directories are manually added and
 * recursively compiled for all files that end with either *.c
 * or *.a.  Any directories in arch/ that don't match the
 * architecture being compiled are not compiled.
 *
 * Architecture is set by the BSP/MCU combination.
 */

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#include "sensor/sensor.h"
#include "sensor/accel.h"
#include "sensor/gyro.h"
#include "sensor/mag.h"
#include "sensor/pressure.h"
#include "sensor/humidity.h"
#include "sensor/temperature.h"
#include "sensor/light.h"
#include "console/console.h"

/* The timer callout */
static struct os_callout sensor_callout;

int sensor_data_cb(struct sensor* sensor, void *arg, void *data, sensor_type_t type)
{
    struct sensor_accel_data *sad;
    struct sensor_mag_data *smd;
    struct sensor_gyro_data *sgd;
    struct sensor_press_data *spd;
    struct sensor_temp_data *std;
    struct sensor_humid_data *shd;
    struct sensor_light_data *sld;
    char tmpstr[13];

    if (type == SENSOR_TYPE_ACCELEROMETER ||
        type == SENSOR_TYPE_LINEAR_ACCEL  ||
        type == SENSOR_TYPE_GRAVITY) {

        console_printf("accel (m/s^2) ");

        sad = (struct sensor_accel_data *) data;
        if (sad->sad_x_is_valid) {
            console_printf("x = %s ", sensor_ftostr(sad->sad_x, tmpstr, 13));
        }
        if (sad->sad_y_is_valid) {
            console_printf("y = %s ", sensor_ftostr(sad->sad_y, tmpstr, 13));
        }
        if (sad->sad_z_is_valid) {
            console_printf("z = %s", sensor_ftostr(sad->sad_z, tmpstr, 13));
        }
        console_printf("\n");
    }

    if (type == SENSOR_TYPE_MAGNETIC_FIELD) {
        smd = (struct sensor_mag_data *) data;
        console_printf("compass (uT)  ");
        if (smd->smd_x_is_valid) {
            console_printf("x = %s ", sensor_ftostr(smd->smd_x, tmpstr, 13));
        }
        if (smd->smd_y_is_valid) {
            console_printf("y = %s ", sensor_ftostr(smd->smd_y, tmpstr, 13));
        }
        if (smd->smd_z_is_valid) {
            console_printf("z = %s ", sensor_ftostr(smd->smd_z, tmpstr, 13));
        }
        console_printf("\n");
    }

    if (type == SENSOR_TYPE_GYROSCOPE) {
        sgd = (struct sensor_gyro_data *) data;
        console_printf("gyro (deg/s)  ");
        
        if (sgd->sgd_x_is_valid) {
            console_printf("x = %s ", sensor_ftostr(sgd->sgd_x, tmpstr, 13));
        }
        if (sgd->sgd_y_is_valid) {
            console_printf("y = %s ", sensor_ftostr(sgd->sgd_y, tmpstr, 13));
        }
        if (sgd->sgd_z_is_valid) {
            console_printf("z = %s ", sensor_ftostr(sgd->sgd_z, tmpstr, 13));
        }
        console_printf("\n");
    }
    
    if (type == SENSOR_TYPE_PRESSURE) {
        spd = (struct sensor_press_data *) data;
        if (spd->spd_press_is_valid) {
            console_printf("pressure = %s Pa",
                           sensor_ftostr(spd->spd_press, tmpstr, 13));
        }
        console_printf("\n");
    }

    if (type == SENSOR_TYPE_TEMPERATURE      ||
        type == SENSOR_TYPE_AMBIENT_TEMPERATURE) {

        std = (struct sensor_temp_data *) data;
        if (std->std_temp_is_valid) {
            console_printf("temperature = %s Deg C", sensor_ftostr(std->std_temp, tmpstr, 13));
        }
        console_printf("\n");
    }

    if (type == SENSOR_TYPE_RELATIVE_HUMIDITY) {
        shd = (struct sensor_humid_data *) data;
        if (shd->shd_humid_is_valid) {
            console_printf("relative humidity = %s%%rh",
                           sensor_ftostr(shd->shd_humid, tmpstr, 13));
        }
        console_printf("\n");
    }

    if (type == SENSOR_TYPE_LIGHT){
        sld = (struct sensor_light_data *) data;
        console_printf("ambient light");
        if (sld->sld_full_is_valid){ 
            console_printf("full = %s ", sensor_ftostr(sld->sld_full, tmpstr, 13));
        }
        if (sld->sld_ir_is_valid){ 
            console_printf("ir = %s ", sensor_ftostr(sld->sld_ir, tmpstr, 13));
        }
        if (sld->sld_lux_is_valid){ 
            console_printf("lux = %s ", sensor_ftostr(sld->sld_lux, tmpstr, 13));
        }
    console_printf("\n");
    }

    return (0);
}

/*
 * Event callback function for timer events. 
*/
static void sensor_timer_ev_cb(struct os_event *ev) {
    int rc;
    struct sensor *s;
    sensor_type_t sensor_types[] = {SENSOR_TYPE_ACCELEROMETER,
                                    SENSOR_TYPE_GYROSCOPE,
                                    SENSOR_TYPE_MAGNETIC_FIELD,
                                    SENSOR_TYPE_PRESSURE,
                                    SENSOR_TYPE_RELATIVE_HUMIDITY,
                                    SENSOR_TYPE_TEMPERATURE,
                                    SENSOR_TYPE_LIGHT,
                                    SENSOR_TYPE_NONE};
        
    assert(ev != NULL);
    int i=0;
    while (sensor_types[i] != SENSOR_TYPE_NONE)
    {
        console_printf("i: %d \n", i);
        s = sensor_mgr_find_next_bytype(sensor_types[i], NULL);
        if (s)
        {
            rc = sensor_read(s,
                             sensor_types[i],
                             &sensor_data_cb,
                             0,
                             OS_TICKS_PER_SEC/10);
            if (rc) console_printf("Error: failed to read sensor\r\n");
        }

        i++;
    }
    
    os_callout_reset(&sensor_callout, OS_TICKS_PER_SEC*5);
}


static void init_timer(void) {
    /*
     * Initialize the callout for a timer event.
     */
    os_callout_init(&sensor_callout, os_eventq_dflt_get(), sensor_timer_ev_cb, NULL);
    os_callout_reset(&sensor_callout, OS_TICKS_PER_SEC);

}


int
main(int argc, char **argv)
{
    /* Perform some extra setup if we're running in the simulator. */
#ifdef ARCH_sim
    mcu_sim_parse_args(argc, argv);
#endif

    /* Initialize all packages. */
    sysinit();

    init_timer();

    os_time_delay(OS_TICKS_PER_SEC*5);
    
    /* As the last thing, process events from default event queue. */
    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}
