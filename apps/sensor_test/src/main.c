#include "os/os.h"
#include "sysinit/sysinit.h"

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

#include "bsp/bsp.h"
#include "hal/hal_bsp.h"
#include "hal/hal_gpio.h"
#include "os/os.h"
#include "sysinit/sysinit.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#include "console/console.h"
#include "sensor/sensor.h"
#include "sensor/accel.h"
#include "sensor/gyro.h"
#include "sensor/humidity.h"
#include "sensor/light.h"
#include "sensor/mag.h"
#include "sensor/pressure.h"
#include "sensor/temperature.h"

#if MYNEWT_VAL(SENSOR_TYPE_VOC)
#include "sensor/voc.h"
#endif

static struct os_callout sensor_callout;

static int sensor_data_printf(struct sensor *sensor, void *arg, void *data,
                              sensor_type_t type)
{

    struct os_dev *dev = SENSOR_GET_DEVICE(sensor);
    console_printf("%s:", (dev && dev->od_name) ? dev->od_name : "<noname>");
    const char *sv;
    static const char *nan = "N/A";

    char tmpstr[13];

#define FTOSTR(X) sensor_ftostr((float)(X), tmpstr, sizeof(tmpstr))
#define UTOSTR(X) ({ \
            snprintf(tmpstr, sizeof(tmpstr), "0x%X", (unsigned int) (X)); \
            tmpstr; \
        })


    switch (type) {
        case SENSOR_TYPE_ACCELEROMETER:
        case SENSOR_TYPE_LINEAR_ACCEL:
        case SENSOR_TYPE_GRAVITY: {
            console_printf("accel: ");
            struct sensor_accel_data *sad = data;

            sv = sad->sad_x_is_valid ? FTOSTR(sad->sad_x) : nan;
            console_printf("x = %s, ", sv);

            sv = sad->sad_y_is_valid ? FTOSTR(sad->sad_y) : nan;
            console_printf("y = %s, ", sv);

            sv = sad->sad_z_is_valid ? FTOSTR(sad->sad_z) : nan;
            console_printf("z = %s ", sv);
            console_printf(" (m/s^2)");
        } break;

        case SENSOR_TYPE_MAGNETIC_FIELD: {
            console_printf("compass:  ");
            struct sensor_mag_data *smd = data;

            sv = smd->smd_x_is_valid ? FTOSTR(smd->smd_x) : nan;
            console_printf("x = %s, ", sv);

            sv = smd->smd_y_is_valid ? FTOSTR(smd->smd_y) : nan;
            console_printf("y = %s, ", sv);

            sv = smd->smd_z_is_valid ? FTOSTR(smd->smd_z) : nan;
            console_printf("z = %s ", sv);
            console_printf(" (mG)");
        } break;

        case SENSOR_TYPE_GYROSCOPE: {
            console_printf("gyro: ");
            struct sensor_gyro_data *sgd = data;

            sv = sgd->sgd_x_is_valid ? FTOSTR(sgd->sgd_x) : nan;
            console_printf("x = %s, ", sv);

            sv = sgd->sgd_y_is_valid ? FTOSTR(sgd->sgd_y) : nan;
            console_printf("y = %s, ", sv);

            sv = sgd->sgd_z_is_valid ? FTOSTR(sgd->sgd_z) : nan;
            console_printf("z = %s ", sv);
            console_printf(" (deg/s)");
        } break;

        case SENSOR_TYPE_PRESSURE: {
            struct sensor_press_data *spd = data;

            console_printf("pressure: ");
            sv = spd->spd_press_is_valid ? FTOSTR(spd->spd_press) : nan;
            console_printf("P = %s (Pa)", sv);

        } break;

        case SENSOR_TYPE_TEMPERATURE:
        case SENSOR_TYPE_AMBIENT_TEMPERATURE: {
            console_printf("temperature: ");
            struct sensor_temp_data *std = data;
            sv = std->std_temp_is_valid ? FTOSTR(std->std_temp) : nan;
            console_printf("T = %s (deg.C)", sv);
        } break;

        case SENSOR_TYPE_RELATIVE_HUMIDITY: {
            struct sensor_humid_data *shd = data;
            sv = shd->shd_humid_is_valid ? FTOSTR(shd->shd_humid) : nan;
            console_printf("relative_humidity = %s (%%rh)", sv);
        } break;

        case SENSOR_TYPE_LIGHT: {
            struct sensor_light_data *sld = data;
            console_printf("ambient_light: ");

            sv = sld->sld_full_is_valid ? UTOSTR(sld->sld_full) : nan;
            console_printf("Full = %s, ", sv);

            sv = (sld->sld_lux_is_valid) ? FTOSTR((float)sld->sld_lux / 1000)
                                         : nan;
            console_printf("Lux = %s, ", sv);

            sv = (sld->sld_ir_is_valid) ? FTOSTR((float)sld->sld_ir / 1000)
                                        : nan;
            console_printf("UV = %s ", sv);
        } break;

#if MYNEWT_VAL(SENSOR_TYPE_VOC)
        case SENSOR_TYPE_VOC: {
            console_printf("air_VOC: ");
            struct sensor_voc_data *svd = data;

            /* Total Volatile Organic Compounds in ppb */
            sv = svd->svd_tvoc_is_valid ? FTOSTR(svd->svd_tvoc) : nan;
            console_printf("TVOC = %s (ppb), ", sv);

            /* carbon dioxide equivalent in ppm */
            sv = svd->svd_co2eq_is_valid ? FTOSTR(svd->svd_co2eq) : nan;
            console_printf("CO2EQ = %s (ppm), ", sv);

            /* IAQ baseline in ? */
            sv = svd->svd_iaqbl_is_valid ? UTOSTR(svd->svd_iaqbl) : nan;
            console_printf("IAQBL = %s ", sv);
        } break;
#endif
        default: {
            uint32_t v = type;
            assert(v && !(v & (v - 1))); // assert single bit set
            console_printf("unhandled_sensor: s_type = %u ",
                           (unsigned int)type);
        } break;
    }

    console_printf("\n");

    return 0;
}

static int sensor_data_cb(struct sensor *sensor, void *arg, void *data,
                          sensor_type_t type)
{
    uint32_t remaining = type;
    for (uint32_t s_type = 1; (s_type && remaining); s_type <<= 1) {
        if (remaining & s_type) {
            sensor_data_printf(sensor, arg, data, s_type);
            remaining &= ~s_type;
        }
    }

    return 0;
}

static void sensor_timer_ev_cb(struct os_event *ev)
{
    assert(ev != NULL);
    int rc;

    for (uint32_t s_type = 1; s_type > 0; s_type <<= 1) {

        struct sensor *s = sensor_mgr_find_next_bytype(s_type, NULL);
        if (s) {
            rc = sensor_read(s, s_type, &sensor_data_cb, 0,
                             OS_TICKS_PER_SEC / 10);
            if (rc) {
                console_printf("Error: sensor_read rc=%i for s_type=%u\n", rc,
                               (unsigned int)s_type);
            }
        }
    }

    os_callout_reset(&sensor_callout, OS_TICKS_PER_SEC * 5);
}

static void list_avaible_sensors(void)
{
    console_printf("== SENSORS ==\n");
    uint32_t s_type = 1;
    for (int i = 0; i < (sizeof(s_type) * 8); i++) {
        struct sensor *s = NULL;
        while (1) {
            s = sensor_mgr_find_next_bytype(s_type, s);
            if (!s)
                break;
            struct os_dev *dev = SENSOR_GET_DEVICE(s);
            console_printf("  dev_name:%s, s_types:0x%X\n", dev->od_name,
                           (unsigned int)s->s_types);
        }

        s_type <<= 1;
    }
    console_printf("== END SENSORS ==\n");
}

int main(int argc, char **argv)
{
#ifdef ARCH_sim
    mcu_sim_parse_args(argc, argv);
#endif

    sysinit();

    list_avaible_sensors();

    os_callout_init(&sensor_callout, os_eventq_dflt_get(), sensor_timer_ev_cb,
                    NULL);
    os_callout_reset(&sensor_callout, OS_TICKS_PER_SEC);

    os_time_delay(OS_TICKS_PER_SEC * 5);

    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}
