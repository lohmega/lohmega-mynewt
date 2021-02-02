#ifndef __LED_CTRL_H
#define __LED_CTRL_H

#include <os/mynewt.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 10 byte per entry */
struct led_script {
    uint32_t c;
    uint16_t dly;
    uint8_t i;
    uint8_t max;
    uint8_t jmp;
    uint8_t jmp_is_abs:1;
} __attribute__((packed, aligned(1)));

int led_ctrl_set_default_script(struct led_script *sp, int sp_len);
int led_ctrl_set_script(struct led_script *sp, int sp_len);
int led_ctrl_set_data(uint8_t *d, int d_len);

#ifdef __cplusplus
}
#endif

#endif  /* LED_CTRL */
