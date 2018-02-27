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
 *
 */

#ifndef __SENSOR_HTS221_H__
#define __SENSOR_HTS221_H__

#include "os/os.h"
#include "os/os_dev.h"
#include "sensor/sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  H0_rH_x2;
    uint8_t  H1_rH_x2;
    uint16_t T0_decC_x8;
    uint16_t T1_decC_x8;
    int16_t  H0_T0_out;
    int16_t  H1_T0_out;
    int16_t  T0_out;
    int16_t  T1_out;
} hts221_cal_t;

    
#define HTS221_WHO_AM_I_VAL 0xBC
    
struct hts221_cfg {
    uint8_t int_enable;
    sensor_type_t mask;
};

struct hts221 {
    struct os_dev dev;
    struct sensor sensor;
    struct os_mutex *i2c_mutex;
    struct hts221_cfg cfg;
    os_time_t last_read_time;
    hts221_cal_t calibration;
};

int hts221_start_conv(struct hts221 *dev);
    
int hts221_enable_interrupt(struct hts221 *dev, uint8_t enable);

int hts221_init(struct os_dev *, void *);
int hts221_config(struct hts221 *, struct hts221_cfg *);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_HTS221_H__ */
