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

#include <limits.h>
#include <assert.h>
#include <string.h>

#include "os/mynewt.h"
#include "hal/hal_bsp.h"
#include "flash_map/flash_map.h"
#include "cborattr/cborattr.h"
#include "mgmt/mgmt.h"

#include <console/console.h>
#include <hal/hal_system.h>

#include "base64/hex.h"
#include "rgbpwm_nmgr_priv.h"

static int rgbpwm_set(struct mgmt_cbuf *cb);

#define RGBPWM_NMGR_ID_SET 0

static const struct mgmt_handler pwmrgb_nmgr_handlers[] = {
    [RGBPWM_NMGR_ID_SET] = {
        .mh_read = NULL,
        .mh_write = rgbpwm_set
    },
};

#define PWMRGB_HANDLER_CNT                                                \
    sizeof(pwmrgb_nmgr_handlers) / sizeof(pwmrgb_nmgr_handlers[0])

static struct mgmt_group pwmrgb_nmgr_group = {
    .mg_handlers = (struct mgmt_handler *)pwmrgb_nmgr_handlers,
    .mg_handlers_count = PWMRGB_HANDLER_CNT,
    .mg_group_id = MGMT_GROUP_ID_RGBPWM,
};

static int
rgbpwm_set(struct mgmt_cbuf *cb)
{
    uint64_t colour = UINT_MAX;
    uint64_t delay = UINT_MAX;

    const struct cbor_attr_t off_attr[] = {
        [0] = {
            .attribute = "c",
            .type = CborAttrUnsignedIntegerType,
            .addr.uinteger = &colour,
            .nodefault = true
        },
        [1] = {
            .attribute = "d",
            .type = CborAttrUnsignedIntegerType,
            .addr.uinteger = &delay,
            .nodefault = true
        },
        [2] = { 0 },
    };
    int rc;
    CborError g_err = CborNoError;
        
    rc = cbor_read_object(&cb->it, off_attr);
    if (rc) {
        return MGMT_ERR_EINVAL;
    }

    // rgbpwm_set(colour, delay);

    g_err |= cbor_encode_text_stringz(&cb->encoder, "rc");
    g_err |= cbor_encode_int(&cb->encoder, MGMT_ERR_EOK);

    if (g_err) {
        return MGMT_ERR_ENOMEM;
    }
    return 0;
}

void
pwmrgb_nmgr_init(void)
{
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    rc = mgmt_group_register(&pwmrgb_nmgr_group);
    SYSINIT_PANIC_ASSERT(rc == 0);
}
