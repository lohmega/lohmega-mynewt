#ifndef __RGBPWM_H
#define __RGBPWM_H

#ifdef __cplusplus
extern "C" {
#endif

int rgbpwm_set_target(float *value, float *delay, int len);
void rgbpwm_delay_local_change_timer(int ms);
void rgbpwm_stop_local_change_timer();
uint32_t rgbpwm_get_random_approved_colour(void);
struct os_mbuf* rgbpwm_get_txcolour_mbuf(uint32_t colour, uint32_t delay);
    
#ifdef __cplusplus
}
#endif

#endif
