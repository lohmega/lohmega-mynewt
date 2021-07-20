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

/**
 * @file bmx160_cli.c
 *
 */

#include <stdlib.h>
#include <stdio.h>

#include <bmx160/bmx160.h>
#include <bmx160/bmx160_defs.h>

#if MYNEWT_VAL(BMX160_CLI)

#include <shell/shell.h>
#include <console/console.h>
#include <console/ticks.h>
#include <hal/hal_gpio.h>

static int bmx160_cli_cmd(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *streamer);

#if MYNEWT_VAL(SHELL_CMD_HELP)
const struct shell_param cmd_bmx160_param[] = {
    {"dump", "[inst] dump all registers"},
    {NULL,NULL},
};

const struct shell_cmd_help cmd_bmx160_help = {
	"bmx160 dbg", "bmx160 debug", cmd_bmx160_param
};
#endif

static struct shell_cmd shell_bmx160_cmd =
    SHELL_CMD_EXT("bmx160", bmx160_cli_cmd, &cmd_bmx160_help);

void
bmx160_cli_dump_registers(struct bmx160 *bmx160, struct streamer *streamer)
{
    uint8_t reg = 0;
    int i;

    streamer_printf(streamer, "BMX160\n");
    for(i=BMX160_REG_CHIP_ID; i<BMX160_REG_SPI_COMM_TEST; i++)
    {
        bmx160_reg_read(bmx160, i, &reg, 1);
        streamer_printf(streamer, " [%02X] = 0x%02X\n", i, reg&0xff);
    }
    streamer_printf(streamer, " int1 = %d \n", hal_gpio_read(bmx160->cfg.int1_pin));
    streamer_printf(streamer, " int2 = %d \n", hal_gpio_read(bmx160->cfg.int2_pin));
}


static int
bmx160_cli_cmd(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *streamer)
{
    struct bmx160 * inst = 0;

    if (argc < 2) {
        return 0;
    }

    inst = (struct bmx160*)os_dev_lookup("bmx160_0");
    if (!inst) {
        streamer_printf(streamer, "ERROR, no device\n");
        return OS_EINVAL;
    }

    if (!strcmp(argv[1], "dump")) {
        console_no_ticks();
        bmx160_cli_dump_registers(inst, streamer);
        console_yes_ticks();
    }

    return 0;
}

int
bmx160_cli_register(void)
{
    int rc;
    rc = shell_cmd_register(&shell_bmx160_cmd);
    return rc;
}
#endif



