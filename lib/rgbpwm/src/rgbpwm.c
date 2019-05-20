#include <assert.h>
#include <pwm/pwm.h>
#include <bsp/bsp.h>
#include <easing/easing.h>
#include <console/console.h>

#include <os/mynewt.h>
#include <console/console.h>

#include <log/log.h>
#include <config/config.h>

#define  PWM_TEST_CH_CFG_INV  true 
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
static uint32_t pwm_freq = 200;
static uint32_t max_steps = 200;
static uint16_t top_val[PWM_NUM_CHANNELS];
static volatile uint32_t step[PWM_NUM_CHANNELS] = {0};
static volatile bool up[PWM_NUM_CHANNELS] = {false, false, false};
static volatile int func_num[PWM_NUM_CHANNELS] = {1,1,1};
static easing_int_func_t easing_funct[PWM_NUM_CHANNELS] = {sine_int_io, sine_int_io, sine_int_io};

static void
pwm_cycle_handler(void* input_arg)
{
    uint8_t ch = *((uint8_t*)input_arg);
    int16_t eased;
    eased = easing_funct[ch](step[ch], max_steps, top_val[ch]);
    pwm_set_duty_cycle(pwm, ch, eased);

    if (step[ch] >= max_steps || step[ch] <= 0) {
        up[ch] = !up[ch];
    }

    step[ch] += (up[ch]) ? 1 : -1;
}


int
pwm_init(void)
{
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
            .n_cycles = pwm_freq,
            .int_prio = PWM_TEST_IRQ_PRIO,
            .cycle_handler = pwm_cycle_handler,     /* this won't work on soft_pwm */
            .seq_end_handler = 0,
            .cycle_data = &(channels[0]),
            .seq_end_data = 0,
            .data = NULL
        };

    int rc = 0;
    pwm = (struct pwm_dev *) os_dev_open(device_str, 0, NULL);
    assert(pwm);
    pwm_configure_device(pwm, &dev_conf);
    /* set the PWM frequency */
    pwm_set_frequency(pwm, pwm_freq);

    for (int i=0;i<sizeof(channels);i++) {
        console_printf("[%d] init\n", i);
        top_val[i] = (uint16_t) pwm_get_top_value(pwm);

        /* setup led */
        rc = pwm_configure_channel(pwm, i, &chan_conf[i]);
        assert(rc == 0);

        rc = pwm_set_duty_cycle(pwm, i, top_val[i]);
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
