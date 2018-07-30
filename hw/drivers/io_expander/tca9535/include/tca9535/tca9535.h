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

#ifndef __TCA9535_H__
#define __TCA9535_H__

#include <io_expander/io_expander.h>

#ifdef __cplusplus
extern "C" {
#endif

struct os_mutex;

struct tca9535_io_expander_dev_cfg {
    uint8_t  i2c_num;
    uint8_t  i2c_addr;
    struct os_mutex *i2c_mutex;
    uint8_t  irq_pin;
    uint16_t direction;

    uint16_t last_interrupt_input;
};

int tca9535_io_expander_dev_init(struct os_dev *, void *);
int tca9535_io_expander_task_init(struct io_expander_dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* __TCA95xx_H__ */
