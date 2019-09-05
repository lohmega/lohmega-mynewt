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
#include "mgmt/mgmt.h"

#include <tinycbor/cbor.h>
#include <tinycbor/cborjson.h>
#include <tinycbor/cbor_mbuf_writer.h>
#include <tinycbor/cbor_mbuf_reader.h>
#include <cborattr/cborattr.h>

#include <config/config.h>
#include <console/console.h>
#include <hal/hal_system.h>

#include "base64/hex.h"
#include "rgbpwm_nmgr_priv.h"
#include <rgbpwm/rgbpwm.h>

static int rgbpwm_set(struct mgmt_cbuf *cb);

#define RGBPWM_NMGR_ID_SET    0
#define RGBPWM_NMGR_ID_CFGSET 1

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
    uint64_t wrgb = 0xFFFFFFFFFFFFFFFFULL;
    uint64_t delay_ms = UINT_MAX;

    const struct cbor_attr_t off_attr[] = {
        [0] = {
            .attribute = "colour",
            .type = CborAttrUnsignedIntegerType,
            .addr.uinteger = &wrgb,
            .nodefault = true
        },
        [1] = {
            .attribute = "delay",
            .type = CborAttrUnsignedIntegerType,
            .addr.uinteger = &delay_ms,
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

    /* If the colour isn't specified, use a random, approved colour */
    if (wrgb == RGBPWM_RANDOM) {
        wrgb = rgbpwm_get_random_approved_colour();
    }

    /* Reset the local timer */
    rgbpwm_delay_local_change_timer(0);
    rgbpwm_set_target32(wrgb, delay_ms);

    g_err |= cbor_encode_text_stringz(&cb->encoder, "rc");
    g_err |= cbor_encode_int(&cb->encoder, MGMT_ERR_EOK);

    if (g_err) {
        return MGMT_ERR_ENOMEM;
    }
    return 0;
}


/**
 * @fn rgbpwm_get_txcolour_mbuf(uint64_t colour, uint32_t delay)
 *
 * @brief creates a mbuf packet to set the colour remotely.
 *
 * input parameters:
 * @param colour - The colour (0x00WWRRGGBBUL), if set to 
 * ULONG_MAX a random colour is used
 * @param delay - time in ms to complete transition
 *
 */
struct os_mbuf*
rgbpwm_get_txcolour_mbuf(uint64_t colour, uint32_t delay)
{
    int rc;
    CborEncoder payload_enc;
    struct mgmt_cbuf n_b;
    struct cbor_mbuf_writer writer;
    struct nmgr_hdr *hdr;
    struct os_mbuf *rsp;
    
    rsp = os_msys_get_pkthdr(0, 0);

    if (!rsp) {
        return 0;
    }

    hdr = (struct nmgr_hdr *) os_mbuf_extend(rsp, sizeof(struct nmgr_hdr));
    if (!hdr) {
        goto exit_err;
    }
    hdr->nh_len = 0;
    hdr->nh_flags = 0;
    hdr->nh_op = NMGR_OP_WRITE;
    hdr->nh_group = htons(MGMT_GROUP_ID_RGBPWM);
    hdr->nh_seq = 0;
    hdr->nh_id = RGBPWM_NMGR_ID_SET;

    cbor_mbuf_writer_init(&writer, rsp);
    cbor_encoder_init(&n_b.encoder, &writer.enc, 0);
    rc = cbor_encoder_create_map(&n_b.encoder, &payload_enc, CborIndefiniteLength);
    if (rc != 0) {
        goto exit_err;
    }

    struct mgmt_cbuf *cb = &n_b;
    CborError g_err = CborNoError;

    if (colour != RGBPWM_RANDOM) {
        g_err |= cbor_encode_text_stringz(&payload_enc, "colour");
        g_err |= cbor_encode_uint(&payload_enc, colour);
    }
    g_err |= cbor_encode_text_stringz(&payload_enc, "delay");
    g_err |= cbor_encode_uint(&payload_enc, delay);
    
    rc = cbor_encoder_close_container(&cb->encoder, &payload_enc);
    if (rc != 0) {
        goto exit_err;
    }
    hdr->nh_len += cbor_encode_bytes_written(&cb->encoder);
    hdr->nh_len = htons(hdr->nh_len);

    return rsp;
exit_err:
    console_printf("something went wrong\n");
    os_mbuf_free_chain(rsp);
    return 0;
}


/**
 * @fn rgbpwm_get_txcfg_mbuf()
 *
 * @brief creates a mbuf packet to set the colour configuration remotely
 * @param cfg_idx - index into vector of config names
 * @returns struct os_mbuf* on success, NULL on fail
 *
 */
struct os_mbuf*
rgbpwm_get_txcfg_mbuf(int cfg_index)
{
    int rc;
    CborEncoder payload_enc;
    struct mgmt_cbuf n_b;
    struct cbor_mbuf_writer writer;
    struct nmgr_hdr *hdr;
    struct os_mbuf *rsp;
    int num_cfgs = 7;
    char *cfg_ptrs[] = {"pwm_freq", "mode", "local_delay", "colours0","colours1","colours2","colours3"};
    char name_str[CONF_MAX_NAME_LEN];

    if (cfg_index >= num_cfgs) {
        return 0;
    }
    rsp = os_msys_get_pkthdr(0, 0);
    if (!rsp) {
        return 0;
    }

    hdr = (struct nmgr_hdr *) os_mbuf_extend(rsp, sizeof(struct nmgr_hdr));
    if (!hdr) {
        goto exit_err;
    }
    hdr->nh_len = 0;
    hdr->nh_flags = 0;
    hdr->nh_op = NMGR_OP_WRITE;
    hdr->nh_group = htons(MGMT_GROUP_ID_CONFIG);
    hdr->nh_seq = 0;
    hdr->nh_id = CONF_NMGR_OP;

    cbor_mbuf_writer_init(&writer, rsp);
    cbor_encoder_init(&n_b.encoder, &writer.enc, 0);
    rc = cbor_encoder_create_map(&n_b.encoder, &payload_enc, CborIndefiniteLength);
    if (rc != 0) {
        goto exit_err;
    }

    struct mgmt_cbuf *cb = &n_b;
    CborError g_err = CborNoError;

    g_err |= cbor_encode_text_stringz(&payload_enc, "name");
    snprintf(name_str, sizeof(name_str)-1, "rgbpwm/%s", cfg_ptrs[cfg_index]);
    g_err |= cbor_encode_text_stringz(&payload_enc, name_str);
    g_err |= cbor_encode_text_stringz(&payload_enc, "val");
    g_err |= cbor_encode_text_stringz(&payload_enc, rgbpwm_conf_get(1, &cfg_ptrs[cfg_index], 0, 0));
    g_err |= cbor_encode_text_stringz(&payload_enc, "save");
    g_err |= cbor_encode_boolean(&payload_enc, true);
    console_printf("# txcfg: '%s' -> '%s'\n", name_str, rgbpwm_conf_get(1, &cfg_ptrs[cfg_index], 0, 0));

    rc = cbor_encoder_close_container(&cb->encoder, &payload_enc);
    if (rc != 0) {
        goto exit_err;
    }
    hdr->nh_len += cbor_encode_bytes_written(&cb->encoder);
    hdr->nh_len = htons(hdr->nh_len);

    return rsp;
exit_err:
    console_printf("something went wrong\n");
    os_mbuf_free_chain(rsp);
    return 0;
}


void
rgbpwm_nmgr_init(void)
{
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    rc = mgmt_group_register(&pwmrgb_nmgr_group);
    SYSINIT_PANIC_ASSERT(rc == 0);
}
