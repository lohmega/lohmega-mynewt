/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * resarding copyright ownership.  The ASF licenses this file
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

#ifndef __SENSOR_SI1133_H__
#define __SENSOR_SI1133_H__

#include <sensor/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

struct si1133_cfg{
    uint8_t int_enable;
    sensor_type_t mask;
};

struct si1133 {
    struct os_dev dev;
    struct sensor sensor;
    struct os_mutex *i2c_mutex;
    struct si1133_cfg cfg;
    os_time_t last_read_time;
};


typedef struct {
    uint8_t irq_status;
    int32_t ch0;
    int32_t ch1;
    int32_t ch2;
    int32_t ch3;
} si1133_Samples_TypeDef;


typedef struct {
    int16_t info;
    uint16_t mag;
} si1133_Coeff_TypeDef;


typedef struct {
    si1133_Coeff_TypeDef coeff_high[4];
    si1133_Coeff_TypeDef coeff_low[9];
} si1133_LuxCoeff_TypeDef;



uint32_t si1133_reset(struct si1133 *dev);
uint32_t si1133_resetCmdCtr(struct si1133 *dev);
uint32_t si1133_measurementForce(struct si1133 *dev);
uint32_t si1133_measurementPause(struct si1133 *dev);
uint32_t si1133_measurementStart(struct si1133 *dev);
uint32_t si1133_waitUntilSleep(struct si1133 *dev);
uint32_t si1133_paramSet(struct si1133 *dev, uint8_t address, uint8_t value);
uint32_t si1133_paramRead(struct si1133 *dev, uint8_t address);
uint32_t si1133_deInit(struct si1133 *dev);
uint32_t si1133_measurementGet(struct si1133 *dev,
        si1133_Samples_TypeDef *samples);
int32_t si1133_getUv(int32_t uv, si1133_Coeff_TypeDef *uk);
int32_t si1133_getLux(int32_t vis_high, int32_t vis_low, int32_t ir,
        si1133_LuxCoeff_TypeDef *lk);
uint32_t si1133_measureLuxUvif(struct si1133 *dev, float *lux, float *uvi);
uint32_t si1133_measureLuxUvi(struct si1133 *dev, int32_t *lux, int32_t *uvi);
uint32_t si1133_getHardwareID(struct si1133 *dev, uint8_t *hardwareID);
uint32_t si1133_getMeasurementf(struct si1133 *dev, float *lux, float *uvi);
uint32_t si1133_getMeasurement(struct si1133 *dev, int32_t *lux,
        int32_t *uvi);
uint32_t si1133_getIrqStatus(struct si1133 *dev, uint8_t *irqStatus);
uint32_t si1133_enableIrq0(struct si1133 *dev, bool enable);
uint32_t si1133_config(struct si1133 *si1, struct si1133_cfg *cfg);
int si1133_init(struct os_dev *dev, void *arg);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_HTS221_H__ */
