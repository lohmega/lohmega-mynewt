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

static int ioexp_cli_cmd(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *streamer);

#if MYNEWT_VAL(SHELL_CMD_HELP)
const struct shell_param cmd_ioexp_param[] = {
    {"status", "<none>"},
    {"set", "<pin> <dir 0=out,1=in> [value]"},
    {"irq", "<pin> <enable 0=ff,1=on>"},
    {NULL,NULL},
};

const struct shell_cmd_help cmd_ioexp_help = {
	"ioexp commands", "status | set <pin> <dir 0=out,1=in> [value]", cmd_ioexp_param
};
#endif

static struct shell_cmd shell_ioexp_cmd = SHELL_CMD_EXT("ioexp", ioexp_cli_cmd, &cmd_ioexp_help);

static void
ioexp_cli_too_few_args(void)
{
    console_printf("Too few args\n");
}

static void
ioexp_cli_print_status(struct io_expander_dev* dev, struct streamer *streamer)
{
    int i, direction, value;
    streamer_printf(streamer, "-------------------\n");
    streamer_printf(streamer, " pin dir value\n");
    for (i=0;i<dev->number_of_pins;i++)
    {
        io_expander_pin_dir(dev, i, &direction);
        value = io_expander_read(dev, i);
        streamer_printf(streamer, "  %02d %s %d\n", i, (direction)? "in ": "out", value);
    }
    streamer_printf(streamer, "-------------------\n");
}

static void
ioexp_cli_irq_test(void *arg)
{
    printf("%s:%d: %p\n", __func__, __LINE__, arg);
}


static int
ioexp_cli_cmd(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *streamer)
{
    int pin, dir, val;
    struct io_expander_dev *dev;

    dev = (struct io_expander_dev *) os_dev_open("io_expander0", OS_TIMEOUT_NEVER, NULL);
    if (!dev) {
        return OS_ENOENT;
    }

    if (argc < 2) {
        ioexp_cli_print_status(dev, streamer);
        return 0;
    }

    for (uint8_t i=0;i<8;i++) {
        char dev_name[16];
        snprintf(dev_name, sizeof(dev_name), "io_expander%d", i&0xF);
        dev = (struct io_expander_dev *) os_dev_open(dev_name, OS_TIMEOUT_NEVER, NULL);

        if (!dev) {
            continue;
        }
        streamer_printf(streamer, "%s:\n", dev_name);
        if (!strcmp(argv[1], "status")) {
            ioexp_cli_print_status(dev, streamer);
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
                io_expander_init_in(dev, pin, (val==0) ? HAL_GPIO_PULL_NONE : HAL_GPIO_PULL_UP);
                streamer_printf(streamer, "set pin %d as input, pull-up=%d\n", pin, val);
            }
            else
            {
                io_expander_init_out(dev, pin, val);
                streamer_printf(streamer, "set pin %d as output with value %d\n", pin, val);
            }
        } else if (!strcmp(argv[1], "irq")) {
            pin = strtol(argv[2], NULL, 0);
            dir = strtol(argv[3], NULL, 0);

            if (dir) {
                io_expander_irq_init(dev, pin, ioexp_cli_irq_test, (void*)(pin),
                                     HAL_GPIO_TRIG_RISING, HAL_GPIO_PULL_NONE);
                io_expander_irq_enable(dev, pin);
            } else {
                io_expander_irq_disable(dev, pin);
                io_expander_irq_release(dev, pin);
            }

            streamer_printf(streamer, "IRQ %s on pin %d\n", dir?"enabled":"disabled", pin);
        } else {
            console_printf("Unknown cmd\n");
        }

        os_dev_close((struct os_dev*) dev);
    }
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
