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
#include <string.h>

#include "defs/error.h"
#include <shell/shell.h>
#include <console/console.h>
#include <io_expander/io_expander.h>

#if MYNEWT_VAL(IO_EXPANDER_CLI)

static int ioexp_cli_cmd(int argc, char **argv);

#if MYNEWT_VAL(SHELL_CMD_HELP)
const struct shell_param cmd_ioexp_param[] = {
    {"status", "<none>"},
    {"set", "<pin> <dir 0=out,1=in> [value]"},
    {NULL,NULL},
};

const struct shell_cmd_help cmd_ioexp_help = {
	"ioexp commands", "status | set <pin> <dir 0=out,1=in> [value]", cmd_ioexp_param
};
#endif

static struct shell_cmd shell_ioexp_cmd = {
    .sc_cmd = "ioexp",
    .sc_cmd_func = ioexp_cli_cmd,
#if MYNEWT_VAL(SHELL_CMD_HELP)
    &cmd_ioexp_help
#endif
};

static void
ioexp_cli_too_few_args(void)
{
    console_printf("Too few args\n");
}

static void
ioexp_cli_print_status(struct io_expander_dev* dev)
{
    int i, direction, value;
    console_printf("io expander status:\n");
    console_printf("-------------------\n");
    console_printf(" pin dir value\n");
    for (i=0;i<dev->number_of_pins;i++)
    {
        io_expander_pin_dir(dev, i, &direction);
        value = io_expander_read(dev, i);
        console_printf("  %02d %s %d\n", i, (direction)? "in ": "out", value);
    }
    console_printf("-------------------\n");
}


static int
ioexp_cli_cmd(int argc, char **argv)
{
    int pin, dir, val;
    struct io_expander_dev *dev;
        
    dev = (struct io_expander_dev *) os_dev_open("io_expander0", OS_TIMEOUT_NEVER, NULL);

    if (argc < 2) {
        ioexp_cli_print_status(dev);
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        ioexp_cli_print_status(dev);
    } else if (!strcmp(argv[1], "set")) {
        if (argc < 4) {
            ioexp_cli_too_few_args();
            return 0;
        }
        pin = strtol(argv[2], NULL, 0);
        dir = strtol(argv[3], NULL, 0);
        val = 0;
        if (argc > 4)
        {
            val = strtol(argv[4], NULL, 0);
        }
        if (dir==1)
        {
            io_expander_init_in(dev, pin, HAL_GPIO_PULL_NONE);
            console_printf("set pin %d as input\n", pin);
        }
        else
        {
            io_expander_init_out(dev, pin, val);
            console_printf("set pin %d as output with value %d\n", pin, val);
        }
    } else {
        console_printf("Unknown cmd\n");
    }

    os_dev_close((struct os_dev*) dev);
    return 0;
}

#endif

int
ioexp_cli_register(void)
{
#if MYNEWT_VAL(IO_EXPANDER_CLI)
    return shell_cmd_register(&shell_ioexp_cmd);
#else
    return 0;
#endif
}
