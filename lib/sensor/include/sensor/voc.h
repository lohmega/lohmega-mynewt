/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __SENSOR_VOC_H__
#define __SENSOR_VOC_H__

#include "os/mynewt.h"
#include "sensor/sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SENSOR_TYPE_VOC 
#define SENSOR_TYPE_VOC SENSOR_TYPE_USER_DEFINED_1
#endif

/**  VOC - Volatile Organic Compounds sensor data
 */ 
struct sensor_voc_data {

    /* Total Volatile Organic Compounds in ppb */
    float svd_tvoc;
    /* carbon dioxide equivalent in ppm */
    float svd_co2eq;
    /* Indoor Air Quality (IAQ) Baseline */
    float svd_iaqbl;
    /* Validity */
    uint8_t svd_tvoc_is_valid:1;
    uint8_t svd_co2eq_is_valid:1;
    uint8_t svd_iaqbl_is_valid:1;
};

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_VOC_H__ */
