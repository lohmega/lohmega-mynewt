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
#ifndef __SYS_PERSISTENT_STATS_H__
#define __SYS_PERSISTENT_STATS_H__

#include <stddef.h>
#include <stdint.h>
#include <os/mynewt.h>
#include <stats/stats.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*persistent_stats_cb_func_t)(uint8_t* data, uint16_t length, uint32_t offset, void* arg);

struct pstats_list {
    struct stats_hdr* shdr;
    SLIST_ENTRY(pstats_list) ps_next;
};

struct persistent_stats_inst {
    persistent_stats_cb_func_t save_fn;
    void *save_fn_arg;
    persistent_stats_cb_func_t load_fn;
    void *load_fn_arg;
    uint32_t offset;
    uint32_t max_length;
};

#define PERSISTENT_STATS_VARNAME(__name)        \
    ps_##__name

#define PERSISTENT_STATS_DECL(__name)                       \
    struct pstats_list PERSISTENT_STATS_VARNAME(__name) =   \
    {.shdr = &(__name.s_hdr)}

int persistent_stats_save(char *name, uint32_t offset);
int persistent_stats_load(char *name, uint32_t offset);
void persistent_stats_set_save_cb(persistent_stats_cb_func_t cb, void *arg, uint32_t max_length);
void persistent_stats_set_load_cb(persistent_stats_cb_func_t cb, void *arg);
int persistent_stats_register(struct pstats_list *shdr);

void persistent_stats_load_all();
void persistent_stats_save_all();

#ifdef __cplusplus
}
#endif

#endif /* __SYS_PERSISTENT_STATS_H__ */
