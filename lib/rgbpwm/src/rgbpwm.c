#include <limits.h>
#include <assert.h>
#include <pwm/pwm.h>
#include <bsp/bsp.h>
#include <math.h>
#include <easing/easing.h>
#include <console/console.h>

#include <os/mynewt.h>
#include <console/console.h>

#include <config/config.h>
#include "rgbpwm/rgbpwm.h"
#include "rgbpwm_smp_priv.h"

#define  PWM_TEST_IRQ_PRIO    3
#define  PWM_MAX_NUM_CHANNELS (4)

static uint8_t num_channels = 4;
static struct pwm_dev *pwm = {0};
static uint32_t pwm_freq = 800;
static uint32_t max_steps[PWM_MAX_NUM_CHANNELS] = {0};
static uint16_t top_val[PWM_MAX_NUM_CHANNELS] = {0};
static volatile uint32_t step[PWM_MAX_NUM_CHANNELS] = {0};
static easing_int_func_t easing_funct[PWM_MAX_NUM_CHANNELS] = {0};
static int16_t start_value[PWM_MAX_NUM_CHANNELS] = {0};
static int16_t target_value[PWM_MAX_NUM_CHANNELS] = {0};
static int16_t current_value[PWM_MAX_NUM_CHANNELS] = {0};
static uint8_t target_reached = 0;
static struct os_callout* target_callout = 0;

/* For each channel:
 *   Start value -> End value over delay
 *   max_steps = f(pwm_freq, delay)
 *   for (i=0;i<max_steps;i++) out = start + easing_funct(i, max_steps, target-start);
 * */
static void
pwm_cycle_handler(void* input_arg)
{
    uint8_t n_at_target = 0;
    for (int i = 0;i<num_channels;i++) {
        if (step[i] > max_steps[i]) {
            continue;
        } else if (step[i] == max_steps[i]) {
            current_value[i] = target_value[i];
            pwm_set_duty_cycle(pwm, i, current_value[i]);
            n_at_target++;
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

    if (n_at_target == num_channels && !target_reached) {
        target_reached = 1;
        if (target_callout) {
            os_callout_reset(target_callout, 0);
            target_callout = 0;
        }
    }
}

static void
pwm_end_seq_handler(void* input_arg)
{
    int rc;
    rc = pwm_disable(pwm); /* Not needed but used for testing purposes. */
    assert(rc == 0);

    rc = pwm_enable(pwm);
    assert(rc == 0);
}

int
rgbpwm_set_target(float *value, float *delay, int len)
{
    os_sr_t sr;
    target_reached = 0;
    for (int i=0;i<len && i < num_channels;i++) {
        OS_ENTER_CRITICAL(sr);
        start_value[i] = current_value[i];
        target_value[i] = (int16_t)roundf(value[i]*top_val[i]);
        /* steps to take = time wanted / time_per_step */
        step[i]=0;
        max_steps[i] = (uint32_t)roundf(delay[i]*pwm_freq/2.0f);
        if (max_steps[i]==0) max_steps[i] = 1;
        OS_EXIT_CRITICAL(sr);
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
        //delay_ms = rgbpwm_inst.colour_change_duration;
        delay_ms = 1000;
    }
    if (delay_ms == 0) {
        //delay_ms = rgbpwm_inst.local_colour_change_delay;
        delay_ms = 1000;
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
rgbpwm_set_target32_wco(uint32_t wrgb, int32_t delay_ms, struct os_callout *co)
{
    /*  */
    float t[4];
    float d[4];

    if (delay_ms == 0) {
        //delay_ms = rgbpwm_inst.colour_change_duration;
        delay_ms = 1000;
    }
    if (delay_ms == 0) {
        //delay_ms = rgbpwm_inst.local_colour_change_delay;
        delay_ms = 1000;
    }

    t[3] = (0xff&(wrgb>>24)) / ((float)0xff);
    t[0] = (0xff&(wrgb>>16)) / ((float)0xff);
    t[1] = (0xff&(wrgb>>8))  / ((float)0xff);
    t[2] = (0xff&(wrgb>>0))  / ((float)0xff);
    d[3] = delay_ms/1000.0f;
    d[0] = delay_ms/1000.0f;
    d[1] = delay_ms/1000.0f;
    d[2] = delay_ms/1000.0f;

    target_callout = co;
    rgbpwm_set_target(t, d, 4);
}

int
rgbpwm_set_freq(int freq)
{
    int rc;
    if (!pwm) {
        return OS_EINVAL;
    }

    rc = pwm_set_frequency(pwm, freq);
    for (int i=0;i<num_channels;i++) {
        uint16_t new_top_val = (uint16_t) pwm_get_top_value(pwm);
        if (new_top_val != top_val[i]) {
            float target =  (float)target_value[i]/top_val[i];
            top_val[i] = new_top_val;
            target_value[i] = (int16_t)roundf(target*top_val[i]);
        }
    }
    return rc;
}

int
rgbpwm_init(const struct rgbpwm_cfg *cfg)
{
    int rc;
    assert(cfg != 0);
    struct pwm_chan_cfg chan_conf[] = {
        {
            .pin = cfg->red_pin,
            .inverted = cfg->red_inverted,
            .data = NULL,
        },
        {
            .pin = cfg->green_pin,
            .inverted = cfg->green_inverted,
            .data = NULL,
        },
        {
            .pin = cfg->blue_pin,
            .inverted = cfg->blue_inverted,
            .data = NULL,
        },
        {
            .pin = cfg->white_pin,
            .inverted = cfg->white_inverted,
            .data = NULL,
        }
    };
    struct pwm_dev_cfg dev_conf = {
        .n_cycles = 10*pwm_freq,
        .int_prio = PWM_TEST_IRQ_PRIO,
        .cycle_handler = pwm_cycle_handler,     /* this won't work on soft_pwm */
        .seq_end_handler = pwm_end_seq_handler,
        .cycle_data = 0,
        .seq_end_data = 0,
        .data = NULL
    };
    num_channels = cfg->num_channels;

    pwm = (struct pwm_dev *) os_dev_open(cfg->pwm_device_str, 0, NULL);
    assert(pwm);
    rc = pwm_configure_device(pwm, &dev_conf);
    assert(rc==0);

    /* set the PWM frequency */
    rc = rgbpwm_set_freq(pwm_freq);
    assert(rc>0);

    for (int i=0;i<num_channels;i++) {
        top_val[i] = (uint16_t) pwm_get_top_value(pwm);
        easing_funct[i] = sine_int_in;

        /* setup led */
        start_value[i] = 0;
        rc = pwm_configure_channel(pwm, i, &chan_conf[i]);
        assert(rc == 0);
        rc = pwm_set_duty_cycle(pwm, i, start_value[0]);
        assert(rc == 0);
    }

    for (int i=0;i<num_channels;i++) {
        target_value[i] = 0;
    }
    rc = pwm_enable(pwm);
    assert(rc == 0);

    return rc;
}

/* Forward declaration */
int rgbpwm_cli_register(void);

int
rgbpwm_pkg_init(void)
{
#if MYNEWT_VAL(RGBPWM_SMP)
    rgbpwm_smp_init();
#endif
#if MYNEWT_VAL(RGBPWM_CLI)
    rgbpwm_cli_register();
#endif
    return 0;
}
