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


#include <errno.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <hal/hal_bsp.h>
#include <hal/hal_system.h>
#include <hal/hal_watchdog.h>
#include <rgbpwm/rgbpwm.h>
#include "defs/error.h"
#include <shell/shell.h>
#include <console/console.h>

#if MYNEWT_VAL(DW1000_DEVICE_0)
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#if MYNEWT_VAL(NMGR_UWB_ENABLED)
#include <nmgr_uwb/nmgr_uwb.h> 
#endif
#endif

#if MYNEWT_VAL(RGBPWM_CLI)

static int rgbpwm_cli_cmd(int argc, char **argv);

#if MYNEWT_VAL(SHELL_CMD_HELP)
const struct shell_param cmd_rgbpwm_param[] = {
    {"set", "<colour, 32-bit hex, WRGB> [ms to get there]"},
    {"tx", "<addr> <colour, 32-bit hex, WRGB> [ms to get there]"},
    {NULL,NULL},
};

const struct shell_cmd_help cmd_rgbpwm_help = {
	"rgbpwm cli", "Manual control of pwm intenisies", cmd_rgbpwm_param
};
#endif

static struct shell_cmd shell_rgbpwm_cmd = {
    .sc_cmd = "rgb",
    .sc_cmd_func = rgbpwm_cli_cmd,
#if MYNEWT_VAL(SHELL_CMD_HELP)
    &cmd_rgbpwm_help
#endif
};

static void
rgbpwm_cli_too_few_args(void)
{
    console_printf("Too few args\n");
}

static void
set_target(uint32_t wrgb, int delay_ms)
{
    float t[4];
    float d[4];
    console_printf("setting rgb to %lX with delay %d\n", wrgb, delay_ms);
    t[3] = (0xff&(wrgb>>24)) / ((float)0xff);
    t[0] = (0xff&(wrgb>>16)) / ((float)0xff);
    t[1] = (0xff&(wrgb>>8))  / ((float)0xff);
    t[2] = (0xff&(wrgb>>0))  / ((float)0xff);
    d[3] = delay_ms/1000.0f;
    d[0] = delay_ms/1000.0f;
    d[1] = delay_ms/1000.0f;
    d[2] = delay_ms/1000.0f;
    rgbpwm_set_target(t, d, 4);
}

static int
rgbpwm_cli_cmd(int argc, char **argv)
{
    uint16_t addr;
    uint32_t target_wrgb;
    uint32_t delay_ms = 0;

    if (!strcmp(argv[1], "set")) {

        if (argc < 3) {
            rgbpwm_cli_too_few_args();
            return 0;
        }
        target_wrgb = strtol(argv[2], NULL, 16);
        if (argc > 3) {
            delay_ms = strtol(argv[3], NULL, 0);
        }
        set_target(target_wrgb, delay_ms);
    } else if (!strcmp(argv[1], "tx")) {

        if (argc < 4) {
            rgbpwm_cli_too_few_args();
            return 0;
        }
        addr = strtol(argv[2], NULL, 16);
        target_wrgb = strtol(argv[3], NULL, 16);
        if (argc > 4) {
            delay_ms = strtol(argv[4], NULL, 0);
        }
        struct os_mbuf *om = rgbpwm_get_txcolour_mbuf(target_wrgb, delay_ms);
        if (!om) return 0;
        nmgr_uwb_instance_t *nmgruwb = (nmgr_uwb_instance_t*)dw1000_mac_find_cb_inst_ptr(hal_dw1000_inst(0), DW1000_NMGR_UWB);

        if (!nmgruwb) return 0;
#if MYNEWT_VAL(NMGR_UWB_ENABLED)
        uwb_nmgr_queue_tx(nmgruwb, addr, NMGR_CMD_STATE_SEND, om);
#else
        console_printf("ERR, no UWB tranceiver present\n");
#endif
        if (addr == 0xffff) {
            set_target(target_wrgb, delay_ms);
        }
    } else if (!strcmp(argv[1], "rst")) {
        // Unused
    }
    return 0;
}

#endif


int
rgbpwm_cli_register(void)
{
#if MYNEWT_VAL(RGBPWM_CLI)
    return shell_cmd_register(&shell_rgbpwm_cmd);
#else
    return 0;
#endif
}
