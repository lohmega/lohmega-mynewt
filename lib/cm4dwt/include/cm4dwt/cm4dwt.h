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

#ifndef __CM4DWT_H__
#define __CM4DWT_H__

#include "os/mynewt.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum cm4dwt_func {
    // From DDI0403E_e_armv7m_arm.pdf
    CM4DWT_FUNC_OFF = 0,
    CM4DWT_FUNC_ONREAD = (1 << 0) | (1 << 2),
    CM4DWT_FUNC_ONWRITE = (1 << 1) | (1 << 2),
    CM4DWT_FUNC_ONREADWRITE = (1 << 0) | (1 << 1) | (1 << 2),
} cm4dwt_func_t;

void cm4dwt_watchpoint_enable(uint8_t watchpoint_index, uint32_t *word_address, uint32_t mask_2_pow_bytes, cm4dwt_func_t f);
void cm4dwt_watchpoint_disable(uint8_t watchpoint_index);
void cm4dwt_clear_stack(uint32_t margin);

#ifdef __cplusplus
}
#endif

#endif /* __CM4DWT_H__ */
