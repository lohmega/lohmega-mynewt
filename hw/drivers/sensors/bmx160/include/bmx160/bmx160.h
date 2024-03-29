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
    uint8_t acc_mode;
    uint8_t acc_rate;
    uint8_t acc_range;
    uint8_t gyro_mode;
    uint8_t gyro_rate;
    uint8_t gyro_range;
    uint8_t mag_mode;
    uint8_t mag_rate;

    uint8_t int1_pin;
    uint8_t int2_pin;
    uint8_t int1_map;
    uint8_t int2_map;

    uint8_t fifo_enable;
    uint8_t fifo_water_level;   /* In units of 4 bytes */

    sensor_type_t en_mask;
};

struct bmx160 {
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    struct bus_i2c_node i2c_node;
#else
    struct os_dev dev;
#endif
    struct sensor sensor;
    struct bmx160_cfg cfg;
    uint32_t int1_ct;
    uint32_t int2_ct;

    uint32_t fifo_tbase;
    uint8_t _txbuf[8];
    uint8_t _rxbuf[64];
    uint64_t _priv[32];
};

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
int bmx160_create_i2c_sensor_dev(struct bus_i2c_node *node, const char *name,
                              const struct bus_i2c_node_cfg *i2c_cfg,
                              struct sensor_itf *sensor_itf);
#endif

int bmx160_init(struct os_dev *dev, void *arg);
int bmx160_config(struct bmx160 *bmx160, const struct bmx160_cfg *cfg);

int bmx160_reg_read(struct bmx160 *bmx160, uint8_t addr, uint8_t *data, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_BMX160_H__ */
