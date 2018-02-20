/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef __HTS221_PRIV_H__
#define __HTS221_PRIV_H__

#ifdef __cplusplus
extern "C" {
#endif

enum hts221_registers {
    HTS221_WHO_AM_I       = 0x0F,
    HTS221_CTRL_REG1      = 0x20,
    HTS221_CTRL_REG2      = 0x21,
    HTS221_CTRL_REG3      = 0x22,
    HTS221_HUMIDITY_OUT_L = 0x28,
    HTS221_HUMIDITY_OUT_H = 0x29,
    HTS221_TEMP_OUT_L     = 0x2A,
    HTS221_TEMP_OUT_H     = 0x2B,
    
    HTS221_CAL_H0RHX2     = 0x30,
    HTS221_CAL_H1RHX2     = 0x31,
    HTS221_CAL_T0DEGCX8   = 0x32,
    HTS221_CAL_T1DEGCX8   = 0x33,
    HTS221_CAL_T1T0MSB    = 0x35,
    HTS221_CAL_H0T0OUT_L  = 0x36,
    HTS221_CAL_H0T0OUT_H  = 0x37,
    HTS221_CAL_H1T0OUT_L  = 0x3A,
    HTS221_CAL_H1T0OUT_H  = 0x3B,
    HTS221_CAL_T0OUT_L    = 0x3C,
    HTS221_CAL_T0OUT_H    = 0x3D,
    HTS221_CAL_T1OUT_L    = 0x3E,
    HTS221_CAL_T1OUT_H    = 0x3F,
};

#define HTS221_CTRL_REG1_PD_MASK 0x80
#define HTS221_CTRL_REG2_OS_MASK 0x01
#define HTS221_CTRL_REG3_DRDY_EN_MASK 0x04
    
int hts221_write8(struct sensor_itf *itf, uint8_t reg, uint32_t value);
int hts221_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value);
int hts221_read_calibration(struct sensor_itf *itf, hts221_cal_t *cal);
    
#ifdef __cplusplus
}
#endif

#endif /* __HTS221_PRIV_H__ */
