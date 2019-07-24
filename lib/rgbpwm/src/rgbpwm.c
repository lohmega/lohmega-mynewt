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

/* 
 * Config 
 */
static char *rgbpwm_conf_get(int argc, char **argv, char *val, int val_len_max);
static int rgbpwm_conf_set(int argc, char **argv, char *val);
static int rgbpwm_conf_commit(void);
static int rgbpwm_conf_export(void (*export_func)(char *name, char *val),
  enum conf_export_tgt tgt);

static struct rgbpwm_config_s {
    char tx_power[16];
} rgbpwm_config = {0};

static struct conf_handler rgbpwm_conf_cbs = {
    .ch_name = "rgbpwm",
    .ch_get = rgbpwm_conf_get,
    .ch_set = rgbpwm_conf_set,
    .ch_commit = rgbpwm_conf_commit,
    .ch_export = rgbpwm_conf_export,
};

static char *
rgbpwm_conf_get(int argc, char **argv, char *val, int val_len_max)
{
    if (argc == 1) {
        if (!strcmp(argv[0], "tx_power")) {
            return rgbpwm_config.tx_power;
        }
    }
    return NULL;
}

static int
rgbpwm_conf_set(int argc, char **argv, char *val)
{
    if (argc == 1) {
        if (!strcmp(argv[0], "tx_power")) {
            return CONF_VALUE_SET(val, CONF_STRING, rgbpwm_config.tx_power);
        }
    }
    return OS_ENOENT;
}

static int
rgbpwm_conf_commit(void)
{
    return 0;
}

static int
rgbpwm_conf_export(void (*export_func)(char *name, char *val),
  enum conf_export_tgt tgt)
{
    export_func("rgbpwm/tx_power", rgbpwm_config.tx_power);
    return 0;
}


static uint8_t channels[PWM_NUM_CHANNELS] = {0,1,2,3};
struct pwm_dev *pwm = {0};
static uint32_t pwm_freq = 1000;
static uint32_t max_steps[PWM_NUM_CHANNELS] = {1024,1024,1024,1024};
static uint16_t top_val[PWM_NUM_CHANNELS] = {0};
static volatile uint32_t step[PWM_NUM_CHANNELS] = {0,0,0,0};
static easing_int_func_t easing_funct[PWM_NUM_CHANNELS] = {sine_int_in, sine_int_in, sine_int_in, sine_int_in};

/* For each channel:
 *   Start value -> End value over delay
 *   max_steps = f(pwm_freq, delay)
 *   for (i=0;i<max_steps;i++) out = start + easing_funct(i, max_steps, target-start);
 * */

static int16_t start_value[PWM_NUM_CHANNELS] = {20000,20000,20000,20000};
static int16_t target_value[PWM_NUM_CHANNELS] = {0,0,0,0};

static void
pwm_cycle_handler(void* input_arg)
{
    int16_t eased=0;

    for (int i = 0;i<PWM_NUM_CHANNELS;i++) {
        if (step[i] > max_steps[i]) {
            continue;
        } else if (step[i] == max_steps[i]) {
            pwm_set_duty_cycle(pwm, i, target_value[i]);
            continue;
        }

        if (start_value[i] < target_value[i]) {
            eased = start_value[i] +
                easing_funct[i](step[i], max_steps[i], target_value[i] - start_value[i]);
            pwm_set_duty_cycle(pwm, i, eased);
        } else {
            eased = start_value[i] -
                easing_funct[i](step[i], max_steps[i], start_value[i] - target_value[i]);
            pwm_set_duty_cycle(pwm, i, eased);
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
    for (int i=0;i<len && i< PWM_NUM_CHANNELS;i++) {
        start_value[i] = target_value[i];
        target_value[i] = (int16_t)roundf(value[i]*top_val[i]);
        /* steps to take = time wanted / time_per_step */
        step[i]=0;
        max_steps[i] = (uint32_t)roundf(delay[i]*pwm_freq/2.0f);
        if (max_steps[i]==0) max_steps[i] = 1;
#if 0
        printf("[%d] s:%d -> t:%d, m:%ld\n", i, start_value[i], target_value[i], max_steps[i]);
        printf("  [%ld %ld]\n",
               start_value[i] + easing_funct[i](0, max_steps[i], target_value[i] - start_value[i]),
               start_value[i] + easing_funct[i](max_steps[i], max_steps[i], target_value[i] - start_value[i]));
#endif
    }
    int rc = pwm_disable(pwm); /* Not needed but used for testing purposes. */
    assert(rc == 0);

    rc = pwm_enable(pwm);
    assert(rc == 0);
    return 0;
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
    console_printf("init clock:%d top_val:%d res:%d, rc:%d\n",
                   pwm_get_clock_freq(pwm), pwm_get_top_value(pwm),
                   pwm_get_resolution_bits(pwm), rc);
    assert(rc>0);

    for (int i=0;i<sizeof(channels);i++) {
        top_val[i] = (uint16_t) pwm_get_top_value(pwm);
        console_printf("[%d] init tv:%d\n", i, top_val[i]);

        /* setup led */
        rc = pwm_configure_channel(pwm, i, &chan_conf[i]);
        assert(rc == 0);
    }

    for (int i=0;i<sizeof(channels);i++) {
        start_value[i] = 0;
        target_value[i] = 0;
        rc = pwm_set_duty_cycle(pwm, i, start_value[0]);
        assert(rc == 0);
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

    pwm_init();
#if MYNEWT_VAL(RGBPWM_CLI)
    rgbpwm_cli_register();
#endif
    return 0;
}
