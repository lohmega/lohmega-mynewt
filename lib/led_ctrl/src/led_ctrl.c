#include <os/os_mutex.h>
#include <rgbpwm/rgbpwm.h>
#include <led_ctrl/led_ctrl.h>

static struct os_callout led_callout;

typedef enum {
    LED_SM_INIT = 0,
    LED_SM_IDLE,
    LED_SM_SCRIPT_INIT,
    LED_SM_SCRIPT_RUN,
    LED_SM_SCRIPT_END,
    LED_SM_MAX,
} led_sm_states_t;

static struct os_mutex script_lock;
static struct led_script *default_script_p = 0;
static int default_script_len = 0;
static struct led_script *script_p = 0;
static int script_i = 0;
static int script_len = 0;
static int led_sm_state = LED_SM_INIT;
static struct led_script *heap_script = 0;

static void
terminate_script(void)
{
    led_sm_state++;
    os_callout_reset(&led_callout, 0);
    script_p = 0;
}

static void
led_ev_cb(struct os_event *ev)
{
    struct led_script *sp;

    os_error_t rc = os_mutex_pend(&script_lock, OS_TIMEOUT_NEVER);
    if (rc!=0) {
        return;
    }
    switch (led_sm_state) {
        case (LED_SM_INIT): {
            rgbpwm_set_target32_wco(0x0, 1, &led_callout);
            led_sm_state++;
            break;
        };
        case (LED_SM_IDLE): {
            break;
        };
        case (LED_SM_SCRIPT_INIT): {
            script_i = 0;
            if (!script_p || !script_len) {
                led_sm_state = LED_SM_SCRIPT_END;
                os_callout_reset(&led_callout, 0);
                break;
            }
            led_sm_state++;
            /* Fall through */
        };
        case (LED_SM_SCRIPT_RUN): {
            //printf("script_i:%d\n", script_i);
            if (script_i >= script_len) {
                terminate_script();
                break;
            }
            sp = &script_p[script_i];
            if (!sp->dly) {
                terminate_script();
                break;
            }
            //printf(" sp->i:%d / %d, jmp:%d, dly:%d\n", sp->i, sp->max, sp->jmp, sp->dly);
            rgbpwm_set_target32_wco(sp->c, sp->dly, &led_callout);

            /* Is this a looping entry */
            if (sp->max) {
                sp->i++;
                if (sp->i < sp->max) {
                    /* Finished */
                    if (sp->jmp_is_abs) {
                        script_i = sp->jmp;
                    } else {
                        script_i += sp->jmp;
                    }
                } else {
                    /* Reset counter */
                    sp->i = 0;
                    script_i++;
                }
            } else {
                script_i++;
            }
            break;
        };
        case (LED_SM_SCRIPT_END): {
            led_sm_state++;
            /* Fall through */
        };
        default: {
            if (heap_script) {
                free(heap_script);
                heap_script = 0;
            }
            if (default_script_p && default_script_len) {
                script_p = default_script_p;
                script_len = default_script_len;
                led_sm_state = LED_SM_SCRIPT_INIT;
            } else {
                led_sm_state = LED_SM_IDLE;
            }
            os_callout_reset(&led_callout, 0);
            break;
        };
    }

    if (led_sm_state >= LED_SM_MAX) {
        led_sm_state = LED_SM_IDLE;
    }
    os_mutex_release(&script_lock);
}

int
led_ctrl_set_default_script(struct led_script *sp, int sp_len)
{
    int rc = os_mutex_pend(&script_lock, OS_TIMEOUT_NEVER);
    if (rc!=0) {
        return OS_EBUSY;
    }
    default_script_p = sp;
    default_script_len = sp_len;
    os_mutex_release(&script_lock);
    return OS_OK;
}

int
led_ctrl_set_script(struct led_script *sp, int sp_len)
{
    int rc = os_mutex_pend(&script_lock, OS_TIMEOUT_NEVER);
    if (rc!=0) {
        return OS_EBUSY;
    }
    script_p = sp;
    script_len = sp_len;
    led_sm_state = LED_SM_SCRIPT_INIT;
    os_callout_reset(&led_callout, 0);
    os_mutex_release(&script_lock);
    return OS_OK;
}

int
led_ctrl_set_data(uint8_t *d, int d_len)
{
    int rc = os_mutex_pend(&script_lock, OS_TIMEOUT_NEVER);
    if (rc!=0) {
        return OS_EBUSY;
    }

    if (heap_script) {
        free(heap_script);
        heap_script = 0;
    }
    heap_script = malloc(d_len);
    if (!heap_script) {
        goto err;
    }
    memcpy(heap_script, d, d_len);
    led_ctrl_set_script(heap_script, d_len / sizeof(struct led_script));

err:
    os_mutex_release(&script_lock);
    return OS_OK;
}

void
led_ctrl_pkg_init(void)
{
    os_callout_init(&led_callout, os_eventq_dflt_get(), led_ev_cb, NULL);
    os_mutex_init(&script_lock);
}
