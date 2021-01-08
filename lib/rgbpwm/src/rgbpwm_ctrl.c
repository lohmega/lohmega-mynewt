#include <os/mynewt.h>
#include <console/console.h>

#include <config/config.h>
#include "rgbpwm/rgbpwm.h"
#include "rgbpwm_smp_priv.h"

#if MYNEWT_VAL(UWB_DEVICE_0)
#include <uwb/uwb.h>
#include <dw1000/dw1000_hal.h>
#if MYNEWT_VAL(SMP_UWB_ENABLED)
#include <smp_uwb/smp_uwb.h>
#endif
#endif

#if MYNEWT_VAL(RGBPWM_CTRL)

/*
 * Config
 */
static int rgbpwm_conf_commit(void);
static int rgbpwm_conf_export(void (*export_func)(char *name, char *val),
  enum conf_export_tgt tgt);

static struct rgbpwm_config_s {
    char verbose[8];
    char mode[8];
    char pwm_freq[8];
    char local_delay_ms[12];
    char master_delay_ms[12];
    char colour_change_duration_ms[12];
    char colours0[64];
    char colours1[64];
    char colours2[64];
    char colours3[64];
} rgbpwm_config = {
    "0x0", "0x0", "1000",         /* Verbose, pwm freq, mode */
    "600000", "0", "300000",      /* local delay(10min), master delay, change duration(5min) */
    "#ff0000,#00ff00,#0000ff,#000000",
    "", "", ""
};
static struct rgbpwm_inst_s {
    uint16_t verbose;
    uint16_t mode;
    uint8_t colour_index;       /* Only used in sequential mode */
    int32_t local_colour_change_delay;
    int32_t master_colour_change_delay;
    int32_t colour_change_duration;
    uint32_t approved_colours[MYNEWT_VAL(RGBPWM_MAX_NUM_APPROVED_COLOURS)];
    int approved_colours_len;
    struct os_callout local_colour_change_callout;
    struct os_callout master_colour_change_callout;
} rgbpwm_inst = {0};

static struct conf_handler rgbpwm_conf_cbs = {
    .ch_name = "rgbpwm",
    .ch_get = rgbpwm_conf_get,
    .ch_set = rgbpwm_conf_set,
    .ch_commit = rgbpwm_conf_commit,
    .ch_export = rgbpwm_conf_export,
};

char *
rgbpwm_conf_get(int argc, char **argv, char *val, int val_len_max)
{
    if (argc == 1) {
        if (!strcmp(argv[0], "pwm_freq")) {
            return rgbpwm_config.pwm_freq;
        } else if (!strcmp(argv[0], "verbose")) {
            return rgbpwm_config.verbose;
        } else if (!strcmp(argv[0], "mode")) {
            return rgbpwm_config.mode;
        } else if (!strcmp(argv[0], "local_delay")) {
            return rgbpwm_config.local_delay_ms;
        } else if (!strcmp(argv[0], "master_delay")) {
            return rgbpwm_config.master_delay_ms;
        } else if (!strcmp(argv[0], "change_duration")) {
            return rgbpwm_config.colour_change_duration_ms;
        } else if (!strcmp(argv[0], "colours0")) {
            return rgbpwm_config.colours0;
        } else if (!strcmp(argv[0], "colours1")) {
            return rgbpwm_config.colours1;
        } else if (!strcmp(argv[0], "colours2")) {
            return rgbpwm_config.colours2;
        } else if (!strcmp(argv[0], "colours3")) {
            return rgbpwm_config.colours3;
        }
    }
    return NULL;
}

int
rgbpwm_conf_set(int argc, char **argv, char *val)
{
    if (argc == 1) {
        if (!strcmp(argv[0], "pwm_freq")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.pwm_freq);
        } else if (!strcmp(argv[0], "verbose")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.verbose);
        } else if (!strcmp(argv[0], "mode")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.mode);
        } else if (!strcmp(argv[0], "local_delay")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.local_delay_ms);
        } else if (!strcmp(argv[0], "master_delay")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.master_delay_ms);
        } else if (!strcmp(argv[0], "change_duration")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.colour_change_duration_ms);
        } else if (!strcmp(argv[0], "colours0")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.colours0);
        } else if (!strcmp(argv[0], "colours1")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.colours1);
        } else if (!strcmp(argv[0], "colours2")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.colours2);
        } else if (!strcmp(argv[0], "colours3")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.colours3);
        }
    }
    return OS_ENOENT;
}

static int
extract_colours(char* s, int len, uint32_t *colours, int maxnum)
{
    char tmp[len];
    memcpy(tmp, s, len);
    char* rest = tmp;
    char* token = strtok_r(tmp, ",", &rest);
    int c = 0;
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (token != NULL) {
        if (!strchr(token, '#')) {
            goto next;
        }
        char *p = token;
        while(*p != '#') {
            p++;
        }
        p++;
        uint32_t v = strtol(p, NULL, 16);
        if (rgbpwm_inst.verbose&RGBPWM_VERBOSE_COMMIT) {
            console_printf("  [%d] '%s' -> #%08lX\n", c, p, v);
        }
        if (c >= maxnum) {
            return c;
        }
        colours[c] = v;
        c++;
    next:
        token = strtok_r(rest, ",", &rest);
    }
    return c;
}

static int
rgbpwm_conf_commit(void)
{
    int rc;
    uint32_t pwm_freq;

    conf_value_from_str(rgbpwm_config.pwm_freq, CONF_INT32, (void*)&pwm_freq, 0);
    conf_value_from_str(rgbpwm_config.verbose, CONF_INT16, (void*)&rgbpwm_inst.verbose, 0);
    conf_value_from_str(rgbpwm_config.mode, CONF_INT16, (void*)&rgbpwm_inst.mode, 0);
    conf_value_from_str(rgbpwm_config.local_delay_ms, CONF_INT32,
                        (void*)&rgbpwm_inst.local_colour_change_delay, 0);
    conf_value_from_str(rgbpwm_config.master_delay_ms, CONF_INT32,
                        (void*)&rgbpwm_inst.master_colour_change_delay, 0);
    conf_value_from_str(rgbpwm_config.colour_change_duration_ms, CONF_INT32,
                        (void*)&rgbpwm_inst.colour_change_duration, 0);

    /* set the PWM frequency */
    rc = rgbpwm_set_freq(pwm_freq);
    assert(rc>0);

    /* Interpret colours from colours strings */
    int tot_spots = sizeof(rgbpwm_inst.approved_colours)/sizeof(rgbpwm_inst.approved_colours[0]);
    int idx = 0;
    idx += extract_colours(rgbpwm_config.colours0, sizeof(rgbpwm_config.colours0), &rgbpwm_inst.approved_colours[idx], tot_spots - idx);
    idx += extract_colours(rgbpwm_config.colours1, sizeof(rgbpwm_config.colours1), &rgbpwm_inst.approved_colours[idx], tot_spots - idx);
    idx += extract_colours(rgbpwm_config.colours2, sizeof(rgbpwm_config.colours2), &rgbpwm_inst.approved_colours[idx], tot_spots - idx);
    idx += extract_colours(rgbpwm_config.colours3, sizeof(rgbpwm_config.colours3), &rgbpwm_inst.approved_colours[idx], tot_spots - idx);
    rgbpwm_inst.approved_colours_len = idx;
    if (rgbpwm_inst.verbose&RGBPWM_VERBOSE_COMMIT) {
        console_printf("# init %d approved colours\n", idx);
    }

    if (rgbpwm_inst.local_colour_change_delay) {
        os_callout_reset(&rgbpwm_inst.local_colour_change_callout, 5*OS_TICKS_PER_SEC);
    }

    if (rgbpwm_inst.mode & RGBPWM_MODE_FIRE) {
        os_callout_reset(&rgbpwm_inst.local_colour_change_callout, OS_TICKS_PER_SEC/10);
    }

    if (rgbpwm_inst.master_colour_change_delay) {
        os_callout_reset(&rgbpwm_inst.master_colour_change_callout, OS_TICKS_PER_SEC);
    }

    return 0;
}

static int
rgbpwm_conf_export(void (*export_func)(char *name, char *val),
                   enum conf_export_tgt tgt)
{
    export_func("rgbpwm/pwm_freq", rgbpwm_config.pwm_freq);
    export_func("rgbpwm/verbose", rgbpwm_config.verbose);
    export_func("rgbpwm/mode", rgbpwm_config.mode);
    export_func("rgbpwm/local_delay", rgbpwm_config.local_delay_ms);
    export_func("rgbpwm/master_delay", rgbpwm_config.master_delay_ms);
    export_func("rgbpwm/change_duration", rgbpwm_config.colour_change_duration_ms);
    export_func("rgbpwm/colours0", rgbpwm_config.colours0);
    export_func("rgbpwm/colours1", rgbpwm_config.colours1);
    export_func("rgbpwm/colours2", rgbpwm_config.colours2);
    export_func("rgbpwm/colours3", rgbpwm_config.colours3);
    return 0;
}

uint32_t
rgbpwm_get_random_approved_colour(void)
{
    return rgbpwm_inst.approved_colours[rand()%rgbpwm_inst.approved_colours_len];
}

uint32_t
rgbpwm_get_sequential_approved_colour(void)
{
    return rgbpwm_inst.approved_colours[(rgbpwm_inst.colour_index++)%rgbpwm_inst.approved_colours_len];
}



static void
local_change_timer_ev_cb(struct os_event *ev)
{
    int32_t change_duration = rgbpwm_inst.colour_change_duration;
    uint64_t wrgb;
    int dly_ticks = 0;
    if (!change_duration) {
        change_duration = rgbpwm_inst.local_colour_change_delay;
    }

    if (rgbpwm_inst.local_colour_change_delay) {
        dly_ticks = (OS_TICKS_PER_SEC*rgbpwm_inst.local_colour_change_delay)/1000;
    }

    if (rgbpwm_inst.mode & RGBPWM_MODE_SEQUENTIAL) {
        wrgb = rgbpwm_get_sequential_approved_colour();
    } else if (rgbpwm_inst.mode & RGBPWM_MODE_FIRE) {
        uint32_t intensity = rand()&0x7F;
        int redmod = rgbpwm_inst.mode >> 12;
        int blue_int = (intensity + (rand()&0xF)-0x7)/32;
        blue_int = (blue_int<0 || blue_int > 255)? 0:blue_int;
        wrgb = (intensity/4)<<24 | (intensity+128)<<16 | (intensity/(redmod+1))<<8 | blue_int;
        dly_ticks = rand()&0x6F;
        change_duration = 10 + (rand()&0x6F);
        dly_ticks = (OS_TICKS_PER_SEC*change_duration)/1000;
        if ((rand()&0x5F) == 0) {
            /* Spark */
            intensity = 255 - 0x2F + (rand()&0x2F);
            blue_int = (rand()&0x1F);
            wrgb = (intensity&0xff)<<24 | (intensity&0xff)<<16 | (intensity&0xff)<<8 | blue_int;
            change_duration = 1;
            dly_ticks = 1;
        }
        // printf("Fire: 0x%6llX %ldms I:%ld bI:%d\n", wrgb, change_duration, intensity, blue_int);
    } else {
        wrgb = rgbpwm_get_random_approved_colour();
    }
    rgbpwm_set_target32(wrgb, change_duration);

    if (dly_ticks) {
        os_callout_reset(&rgbpwm_inst.local_colour_change_callout, dly_ticks);
    }
}

static void
master_timer_ev_cb(struct os_event *ev)
{
    uint64_t wrgb = RGBPWM_RANDOM;
    /* Reset timer */
    if (rgbpwm_inst.master_colour_change_delay) {
        os_callout_reset(&rgbpwm_inst.master_colour_change_callout,
                         (OS_TICKS_PER_SEC*rgbpwm_inst.master_colour_change_delay)/1000);
    }

    /* Prevent local timer from triggering in the meantime */
    rgbpwm_delay_local_change_timer(rgbpwm_inst.master_colour_change_delay *2);

    if (rgbpwm_inst.mode & RGBPWM_MODE_COMMON) {
        wrgb = rgbpwm_get_random_approved_colour();
        if (rgbpwm_inst.verbose&RGBPWM_VERBOSE_MASTER &&
            !(rgbpwm_inst.mode & RGBPWM_MODE_SEQUENTIAL)) {
            console_printf("# master: common rng target #%08llx\n", wrgb);
        }
    }
    if (rgbpwm_inst.mode & RGBPWM_MODE_SEQUENTIAL) {
        wrgb = rgbpwm_get_sequential_approved_colour();
        if (rgbpwm_inst.verbose&RGBPWM_VERBOSE_MASTER) {
            console_printf("# master: common seq target #%08llx\n", wrgb);
        }
    }

    if (wrgb == RGBPWM_RANDOM) {
        if (rgbpwm_inst.verbose&RGBPWM_VERBOSE_MASTER) {
            console_printf("# master: random target \n");
        }
    }
    /* If the change duration is zero it will fall back to the local
     * node's local_delay */
    int32_t change_duration = rgbpwm_inst.colour_change_duration;

#if MYNEWT_VAL(UWB_DEVICE_0) && MYNEWT_VAL(SMP_UWB_ENABLED) && MYNEWT_VAL(RGBPWM_SMP)
    smp_uwb_instance_t *smpuwb = (smp_uwb_instance_t*)uwb_mac_find_cb_inst_ptr(uwb_dev_idx_lookup(0), UWBEXT_SMP_UWB);
    if (!smpuwb) {
        return;
    }

    struct os_mbuf *om = rgbpwm_get_txcolour_mbuf(wrgb, change_duration);
    if (!om) {
        return;
    }

    uwb_smp_queue_tx(smpuwb, 0xffff, UWB_DATA_CODE_SMP_REQUEST, om);
#endif
    int start_num_free = os_msys_num_free();

    /* Wait until packet sent to network before we change our local colour */
    int timeout=OS_TICKS_PER_SEC;
    while (os_msys_num_free() == start_num_free && --timeout>0) {
        os_time_delay(1);
    }

    if (!change_duration) {
        change_duration = rgbpwm_inst.master_colour_change_delay;
    }

    /* Local change synced with network change */
    if (wrgb == RGBPWM_RANDOM) {
        rgbpwm_set_random(change_duration);
    } else {
        rgbpwm_set_target32(wrgb, change_duration);
    }

}

void
rgbpwm_set_random(int32_t delay_ms)
{
    /*  */
    uint32_t wrgb = rgbpwm_get_random_approved_colour();
    rgbpwm_set_target32(wrgb, delay_ms);
}


void
rgbpwm_stop_local_change_timer()
{
    os_callout_stop(&rgbpwm_inst.local_colour_change_callout);
}

void
rgbpwm_delay_local_change_timer(int arg_ms)
{
    os_callout_stop(&rgbpwm_inst.local_colour_change_callout);
    int ms = arg_ms;
    if (!ms) {
        ms = rgbpwm_inst.local_colour_change_delay;
    }
    os_callout_reset(&rgbpwm_inst.local_colour_change_callout, (OS_TICKS_PER_SEC*ms)/1000);
}

/* Forward declaration */
int rgbpwm_cli_register(void);

int
rgbpwm_ctrl_pkg_init(void)
{
    int rc;
    /* Init config */
    rc = conf_register(&rgbpwm_conf_cbs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_callout_init(&rgbpwm_inst.local_colour_change_callout, os_eventq_dflt_get(),
                    local_change_timer_ev_cb, NULL);
    os_callout_init(&rgbpwm_inst.master_colour_change_callout, os_eventq_dflt_get(),
                    master_timer_ev_cb, NULL);

    return 0;
}

#endif
