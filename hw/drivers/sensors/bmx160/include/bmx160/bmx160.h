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

struct bmm150_trim_regs {
	int8_t x1;
	int8_t y1;
	uint16_t reserved1;
	uint8_t reserved2;
	int16_t z4;
	int8_t x2;
	int8_t y2;
	uint16_t reserved3;
	int16_t z2;
	uint16_t z1;
	uint16_t xyz1;
	int16_t z3;
	int8_t xy2;
	uint8_t xy1;
} __attribute__((packed));

struct bmx160 {
    struct os_dev dev;
    struct sensor sensor;
    struct bmx160_cfg cfg;
    uint8_t _txbuf[8];
    uint8_t _rxbuf[12];
    struct bmm150_trim_regs _trim_regs;
    //uint64_t _priv_bmi160[74/8]; // private data 64 bit aligned
    //uint64_t _priv_bmm150[74/8]; // private data 64 bit aligned
};

int bmx160_init(struct os_dev *, void *arg);
int bmx160_config(struct bmx160 *bmx160, const struct bmx160_cfg *cfg);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_BMX160_H__ */
