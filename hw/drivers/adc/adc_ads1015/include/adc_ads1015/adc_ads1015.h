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

#ifndef __ADC_ADS1015_H__
#define __ADC_ADS1015_H__

#include <adc/adc.h>

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
#define ADS1015_CONVERSIONDELAY         (1)
#define ADS1115_CONVERSIONDELAY         (8)
/*=========================================================================*/

#define ADS1015_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
#define ADS1015_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
#define ADS1015_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
#define ADS1015_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
#define ADS1015_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
#define ADS1015_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

struct os_mutex;

typedef enum
{
    GAIN_TWOTHIRDS    = ADS1015_REG_CONFIG_PGA_6_144V,
    GAIN_ONE          = ADS1015_REG_CONFIG_PGA_4_096V,
    GAIN_TWO          = ADS1015_REG_CONFIG_PGA_2_048V,
    GAIN_FOUR         = ADS1015_REG_CONFIG_PGA_1_024V,
    GAIN_EIGHT        = ADS1015_REG_CONFIG_PGA_0_512V,
    GAIN_SIXTEEN      = ADS1015_REG_CONFIG_PGA_0_256V
} ads_gain_t;

#define ADS1015_ADC_DEV_CFG_DEFAULT {.conversion_delay = ADS1115_CONVERSIONDELAY,\
            .bit_shift = 4,\
            .gain = GAIN_ONE}

struct ads1015_adc_dev_cfg {
    uint8_t  i2c_num;
    uint8_t  i2c_addr;
    struct os_mutex *i2c_mutex;

    uint8_t   conversion_delay;
    uint8_t   bit_shift;
    ads_gain_t gain;
};

#ifdef __cplusplus
extern "C" {
#endif

int ads1015_adc_dev_init(struct os_dev *, void *);
int ads1015_adc_hwtest(struct adc_dev *);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
