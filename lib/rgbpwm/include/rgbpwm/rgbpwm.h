#ifndef __RGBPWM_H
#define __RGBPWM_H

#ifdef __cplusplus
extern "C" {
#endif

// int uwb_nmgr_tx(uint16_t src, uint16_t dst, uint16_t type, struct os_mbuf *om);
int rgbpwm_set_target(float *value, float *delay, int len);
    
#ifdef __cplusplus
}
#endif

#endif