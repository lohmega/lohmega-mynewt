#ifndef __RGBPWM_H
#define __RGBPWM_H

#ifdef __cplusplus
extern "C" {
#endif

#define RGBPWM_RANDOM (0xFFFFFFFFFFFFFFFFULL)

#define RGBPWM_VERBOSE_COMMIT (0x0001)
#define RGBPWM_VERBOSE_TARGET (0x0002)
#define RGBPWM_VERBOSE_MASTER (0x0004)

#define RGBPWM_MODE_RANDOM        (0x0000)
#define RGBPWM_MODE_COMMON        (0x0001)
#define RGBPWM_MODE_SEQUENTIAL    (0x0002)
#define RGBPWM_MODE_FIRE          (0x0004)

struct rgbpwm_cfg {
    char* pwm_device_str;
    int red_pin;
    int green_pin;
    int blue_pin;
    int white_pin;
    uint8_t red_inverted:1;
    uint8_t green_inverted:1;
    uint8_t blue_inverted:1;
    uint8_t white_inverted:1;
};

int rgbpwm_init(const struct rgbpwm_cfg *cfg);

int rgbpwm_set_target(float *value, float *delay, int len);
void rgbpwm_set_target32(uint32_t wrgb, int32_t delay_ms);

void rgbpwm_set_random(int32_t delay_ms);
void rgbpwm_delay_local_change_timer(int ms);
void rgbpwm_stop_local_change_timer();
uint32_t rgbpwm_get_random_approved_colour(void);
struct os_mbuf* rgbpwm_get_txcolour_mbuf(uint64_t colour, uint32_t delay);
struct os_mbuf* rgbpwm_get_txcfg_mbuf(int cfg_index);

char *rgbpwm_conf_get(int argc, char **argv, char *val, int val_len_max);
int rgbpwm_conf_set(int argc, char **argv, char *val);

#ifdef __cplusplus
}
#endif

#endif
