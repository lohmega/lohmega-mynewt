#include <limits.h>
#include <assert.h>
#include <pwm/pwm.h>
#include <bsp/bsp.h>
#include <math.h>
#include <easing/easing.h>
#include <console/console.h>

#include <os/mynewt.h>
#include <console/console.h>

#include <log/log.h>
#include <config/config.h>
#include "rgbpwm/rgbpwm.h"
#include "rgbpwm_nmgr_priv.h"

#if MYNEWT_VAL(DW1000_DEVICE_0)
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#if MYNEWT_VAL(NMGR_UWB_ENABLED)
#include <nmgr_uwb/nmgr_uwb.h> 
#endif
#endif

#define  PWM_TEST_CH_CFG_INV  false
#define  PWM_TEST_IRQ_PRIO    3
#define  PWM_NUM_CHANNELS (4)

/** Log data. */
#define LOG_MODULE_RGBPWM    (67)
#define RGBPWM_INFO(...)     LOG_INFO(&_log,  LOG_MODULE_RGBPWM, __VA_ARGS__)
#define RGBPWM_DEBUG(...)    LOG_DEBUG(&_log, LOG_MODULE_RGBPWM, __VA_ARGS__)
#define RGBPWM_WARN(...)     LOG_WARN(&_log,  LOG_MODULE_RGBPWM, __VA_ARGS__)
#define RGBPWM_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_RGBPWM, __VA_ARGS__)
struct log _log;

static uint8_t channels[PWM_NUM_CHANNELS] = {0,1,2,3};
static struct pwm_dev *pwm = {0};
static uint32_t pwm_freq = 100;
static uint32_t max_steps[PWM_NUM_CHANNELS] = {1024,1024,1024,1024};
static uint16_t top_val[PWM_NUM_CHANNELS] = {0};
static volatile uint32_t step[PWM_NUM_CHANNELS] = {0,0,0,0};
static easing_int_func_t easing_funct[PWM_NUM_CHANNELS] = {sine_int_in, sine_int_in, sine_int_in, sine_int_in};
static int16_t start_value[PWM_NUM_CHANNELS] = {0,0,0,0};
static int16_t target_value[PWM_NUM_CHANNELS] = {0,0,0,0};
static int16_t current_value[PWM_NUM_CHANNELS] = {0,0,0,0};

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

    if (pwm==0) return 0;
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
    rc = pwm_set_frequency(pwm, pwm_freq);
    if (rgbpwm_inst.verbose&RGBPWM_VERBOSE_COMMIT) {
        console_printf("# init clock:%d top_val:%d res:%d, rc:%d\n",
                       pwm_get_clock_freq(pwm), pwm_get_top_value(pwm),
                       pwm_get_resolution_bits(pwm), rc);
    }
    assert(rc>0);

    for (int i=0;i<sizeof(channels);i++) {
        uint16_t new_top_val = (uint16_t) pwm_get_top_value(pwm);
        if (new_top_val != top_val[i]) {
            float target =  (float)target_value[i]/top_val[i];
            top_val[i] = new_top_val;
            target_value[i] = (int16_t)roundf(target*top_val[i]);
        }
    }

    for (int i=0;i<sizeof(channels);i++) {
        rc = pwm_enable(pwm);
        assert(rc == 0);
    }

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
    if (!change_duration) {
        change_duration = rgbpwm_inst.local_colour_change_delay;
    }

    if (rgbpwm_inst.mode & RGBPWM_MODE_SEQUENTIAL) {
        wrgb = rgbpwm_get_sequential_approved_colour();
    } else {
        wrgb = rgbpwm_get_random_approved_colour();
    }
    rgbpwm_set_target32(wrgb, change_duration);

    if (rgbpwm_inst.local_colour_change_delay) {
        os_callout_reset(&rgbpwm_inst.local_colour_change_callout,
                         (OS_TICKS_PER_SEC*rgbpwm_inst.local_colour_change_delay)/1000);
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

    struct os_mbuf *om = rgbpwm_get_txcolour_mbuf(wrgb, change_duration);
    if (!om) {
        return;
    }
    int start_num_free = os_msys_num_free();

#if MYNEWT_VAL(DW1000_DEVICE_0)
    nmgr_uwb_instance_t *nmgruwb = (nmgr_uwb_instance_t*)dw1000_mac_find_cb_inst_ptr(hal_dw1000_inst(0), DW1000_NMGR_UWB);
    if (!nmgruwb) {
        return;
    }

#if MYNEWT_VAL(NMGR_UWB_ENABLED)
    uwb_nmgr_queue_tx(nmgruwb, 0xffff, NMGR_CMD_STATE_SEND, om);
#else
    console_printf("ERR, no NMGR-UWB enabled\n");
#endif // MYNEWT_VAL(NMGR_UWB_ENABLED)
#else 
    console_printf("ERR, no UWB tranceiver present\n");
#endif // MYNEWT_VAL(DW1000_DEVICE_0)

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


/* For each channel:
 *   Start value -> End value over delay
 *   max_steps = f(pwm_freq, delay)
 *   for (i=0;i<max_steps;i++) out = start + easing_funct(i, max_steps, target-start);
 * */

static void
pwm_cycle_handler(void* input_arg)
{
    for (int i = 0;i<PWM_NUM_CHANNELS;i++) {
        if (step[i] > max_steps[i]) {
            continue;
        } else if (step[i] == max_steps[i]) {
            current_value[i] = target_value[i];
            pwm_set_duty_cycle(pwm, i, current_value[i]);
            continue;
        }

        if (start_value[i] < target_value[i]) {
            current_value[i] = start_value[i] +
                easing_funct[i](step[i], max_steps[i], target_value[i] - start_value[i]);
            pwm_set_duty_cycle(pwm, i, current_value[i]);
        } else {
            current_value[i] = start_value[i] -
                easing_funct[i](step[i], max_steps[i], start_value[i] - target_value[i]);
            pwm_set_duty_cycle(pwm, i, current_value[i]);
        }

        step[i] += 1;
    }
}

static void
pwm_end_seq_handler(void* input_arg)
{
    int rc;
#if 0
    for (int i = 0;i<PWM_NUM_CHANNELS;i++) {
        printf("  step[%i]=%ld\n", i, step[i]);
    }
    printf("end_seq ()\n");
#endif
    rc = pwm_disable(pwm); /* Not needed but used for testing purposes. */
    assert(rc == 0);

    rc = pwm_enable(pwm);
    assert(rc == 0);
}

int
rgbpwm_set_target(float *value, float *delay, int len)
{
    os_sr_t sr;
    for (int i=0;i<len && i< PWM_NUM_CHANNELS;i++) {
        OS_ENTER_CRITICAL(sr);
        start_value[i] = current_value[i];
        target_value[i] = (int16_t)roundf(value[i]*top_val[i]);
        /* steps to take = time wanted / time_per_step */
        step[i]=0;
        max_steps[i] = (uint32_t)roundf(delay[i]*pwm_freq/2.0f);
        if (max_steps[i]==0) max_steps[i] = 1;
        OS_EXIT_CRITICAL(sr);
        if (rgbpwm_inst.verbose&RGBPWM_VERBOSE_TARGET) {
            printf("# set_target c:%d: %04X -> %04X over %ld steps\n", i, start_value[i], target_value[i], max_steps[i]);
        }
    }

    int rc = pwm_disable(pwm);
    assert(rc == 0);
    rc = pwm_enable(pwm);
    assert(rc == 0);
    return 0;
}

void
rgbpwm_set_target32(uint32_t wrgb, int32_t delay_ms)
{
    /*  */
    float t[4];
    float d[4];

    if (delay_ms == 0) {
        delay_ms = rgbpwm_inst.colour_change_duration;
    }
    if (delay_ms == 0) {
        delay_ms = rgbpwm_inst.local_colour_change_delay;
    }
    
    t[3] = (0xff&(wrgb>>24)) / ((float)0xff);
    t[0] = (0xff&(wrgb>>16)) / ((float)0xff);
    t[1] = (0xff&(wrgb>>8))  / ((float)0xff);
    t[2] = (0xff&(wrgb>>0))  / ((float)0xff);
    d[3] = delay_ms/1000.0f;
    d[0] = delay_ms/1000.0f;
    d[1] = delay_ms/1000.0f;
    d[2] = delay_ms/1000.0f;

    rgbpwm_set_target(t, d, 4);
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

#if MYNEWT_VAL(RGBPWM_RED_LED_PIN) < 0 || MYNEWT_VAL(RGBPWM_GREEN_LED_PIN) < 0 || MYNEWT_VAL(RGBPWM_BLUE_LED_PIN) < 0
#warning "please set RGBPWM_XXX_LED_PINs"
#endif
int
pwm_init(void)
{
    int rc;
    char* device_str = MYNEWT_VAL(RGBPWM_PWM_DEVICE);
    struct pwm_chan_cfg chan_conf[] = {
        {
            .pin = MYNEWT_VAL(RGBPWM_RED_LED_PIN),
            .inverted = PWM_TEST_CH_CFG_INV,
            .data = NULL,
        },
        {
            .pin = MYNEWT_VAL(RGBPWM_GREEN_LED_PIN),
            .inverted = PWM_TEST_CH_CFG_INV,
            .data = NULL,
        },
        {
            .pin = MYNEWT_VAL(RGBPWM_BLUE_LED_PIN),
            .inverted = PWM_TEST_CH_CFG_INV,
            .data = NULL,
        },
        {
            .pin = MYNEWT_VAL(RGBPWM_WHITE_LED_PIN),
            .inverted = PWM_TEST_CH_CFG_INV,
            .data = NULL,
        }
    };
    struct pwm_dev_cfg dev_conf = {
            .n_cycles = 10*pwm_freq,
            .int_prio = PWM_TEST_IRQ_PRIO,
            .cycle_handler = pwm_cycle_handler,     /* this won't work on soft_pwm */
            .seq_end_handler = pwm_end_seq_handler,
            .cycle_data = &(channels[0]),
            .seq_end_data = 0,
            .data = NULL
        };

    pwm = (struct pwm_dev *) os_dev_open(device_str, 0, NULL);
    assert(pwm);
    rc = pwm_configure_device(pwm, &dev_conf);
    assert(rc==0);
    /* set the PWM frequency */
    rc = pwm_set_frequency(pwm, pwm_freq);
    assert(rc>0);

    for (int i=0;i<sizeof(channels);i++) {
        top_val[i] = (uint16_t) pwm_get_top_value(pwm);

        /* setup led */
        start_value[i] = 0;
        rc = pwm_configure_channel(pwm, i, &chan_conf[i]);
        assert(rc == 0);
        rc = pwm_set_duty_cycle(pwm, i, start_value[0]);
        assert(rc == 0);
    }

    for (int i=0;i<sizeof(channels);i++) {
        target_value[i] = 0;
        rc = pwm_enable(pwm);
        assert(rc == 0);
    }

    return rc;
}

/* Forward declaration */
int rgbpwm_cli_register(void);

int
rgbpwm_pkg_init(void)
{
    int rc;
    /* Init log and Config */

    log_register("rgb", &_log, &log_console_handler, NULL, LOG_SYSLEVEL);
    rc = conf_register(&rgbpwm_conf_cbs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_callout_init(&rgbpwm_inst.local_colour_change_callout, os_eventq_dflt_get(),
                    local_change_timer_ev_cb, NULL);
    os_callout_init(&rgbpwm_inst.master_colour_change_callout, os_eventq_dflt_get(),
                    master_timer_ev_cb, NULL);
    
    pwm_init();
#if MYNEWT_VAL(RGBPWM_CLI)
    rgbpwm_cli_register();
#endif
#if MYNEWT_VAL(RGBPWM_NMGR)
    rgbpwm_nmgr_init();
#endif
    return 0;
}
