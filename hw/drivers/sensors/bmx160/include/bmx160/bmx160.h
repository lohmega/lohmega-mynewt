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

#ifndef __SENSOR_BMX160_H__
#define __SENSOR_BMX160_H__

#include "os/mynewt.h"
#include "sensor/sensor.h"
#include "sensor/accel.h"
#include "sensor/temperature.h"

#ifdef __cplusplus
extern "C" {
#endif

struct bmx160_cfg {
};
#if 0
struct bmm150_trim_regs
{
    int8_t dig_x1;
    int8_t dig_y1;
    int8_t dig_x2;
    int8_t dig_y2;
    uint16_t dig_z1;
    int16_t dig_z2;
    int16_t dig_z3;
    int16_t dig_z4;
    uint8_t dig_xy1;
    int8_t dig_xy2;
    uint16_t dig_xyz1;
};
#endif

struct bmx160 {
    struct os_dev dev;
    struct sensor sensor;
    struct bmx160_cfg cfg;
    uint8_t _txbuf[8];
    uint8_t _rxbuf[16];
    uint64_t _priv[32];
};

int bmx160_init(struct os_dev *, void *arg);
int bmx160_config(struct bmx160 *bmx160, const struct bmx160_cfg *cfg);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_BMX160_H__ */
