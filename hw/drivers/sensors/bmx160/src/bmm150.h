/* sensor_bmm150.h - header file for BMM150 Geomagnetic sensor driver */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_BMM150_H__
#define __SENSOR_BMM150_H__


#include <stdint.h>
#include "bmm150_defs.h"

#define BMM150_REG_CHIP_ID         0x40
#define BMM150_REG_DATA_X_L        0x42
#define BMM150_REG_DATA_X_M        0x43
#define BMM150_REG_DATA_Y_L        0x44
#define BMM150_REG_DATA_Y_M        0x45
#define BMM150_REG_DATA_Z_L        0x46
#define BMM150_REG_DATA_Z_M        0x47
#define BMM150_REG_RHALL_L         0x48
#define BMM150_REG_RHALL_M         0x49
#define BMM150_REG_INT_STATUS      0x4A
#define BMM150_REG_POWER           0x4B
#define BMM150_REG_OPMODE_ODR      0x4C
#define BMM150_REG_LOW_THRESH      0x4F
#define BMM150_REG_HIGH_THRESH     0x50
#define BMM150_REG_REP_XY          0x51
#define BMM150_REG_REP_Z           0x52
#define BMM150_REG_REP_DATAMASK    0xFF
#define BMM150_REG_TRIM_START      0x5D
#define BMM150_REG_TRIM_END        0x71

float bmm150_compensate_xf(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_x);
float bmm150_compensate_yf(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_y);
float bmm150_compensate_zf(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_z);



int16_t bmm150_compensate_x(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_x);
int16_t bmm150_compensate_y(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_y);
int16_t bmm150_compensate_z(const struct bmm150_trim_regs *trim, uint16_t data_rhall, int16_t mag_data_z);

#endif /* __SENSOR_BMM150_H__ */

