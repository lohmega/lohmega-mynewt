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

#ifndef __MCP23008_H__
#define __MCP23008_H__

#include <io_expander/io_expander.h>

#ifdef __cplusplus
extern "C" {
#endif

struct os_mutex;

struct mcp23008_io_expander_dev_cfg {
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    struct bus_i2c_node_cfg i2c_node_cfg;
#else
    uint8_t i2c_num;
    uint8_t i2c_addr;
    struct os_mutex *i2c_mutex;
#endif
    uint8_t inst_id;
    uint8_t irq_pin;
    uint8_t rst_pin;
    uint8_t direction;
    uint8_t pull_up;

    uint8_t last_interrupt_input;
    struct os_eventq *interrupt_eventq;
    struct os_event interrupt_event;
};

int mcp23008_io_expander_dev_init(struct os_dev *, void *);
int mcp23008_io_expander_cfg(struct io_expander_dev *dev);
int mcp23008_io_expander_create_i2c_dev(struct bus_i2c_node *node, const char *name,
                                        struct mcp23008_io_expander_dev_cfg *cfg);

#ifdef __cplusplus
}
#endif

#endif /* __MCP23008_H__ */
