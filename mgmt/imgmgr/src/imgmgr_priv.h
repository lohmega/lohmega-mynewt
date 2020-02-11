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

#ifndef __IMGMGR_PRIV_H_
#define __IMGMGR_PRIV_H_

#include <stdint.h>
#include "os/mynewt.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMGMGR_MAX_IMGS		2

#define IMGMGR_HASH_STR		48

#define IMGMGR_DATA_SHA_LEN     32 /* SHA256 */

#define IMAGE_MAGIC_V1 0x96f3b83eUL
#define IMAGE_MAGIC_V2 0x96f3b83dUL
#define IMAGE_TLV_INFO_MAGIC        0x6907

/** Image TLV header.  All fields in little endian. */
struct image_tlv_info {
    uint16_t it_magic;
    uint16_t it_tlv_tot;  /* size of TLV area (including tlv_info header) */
};

/** Image header.  All fields are in little endian byte order. */
struct image_header_v2 {
    uint32_t ih_magic;
    uint32_t ih_load_addr;
    uint16_t ih_hdr_size; /* Size of image header (bytes). */
    uint16_t _pad1;
    uint32_t ih_img_size; /* Does not include header. */
    uint32_t ih_flags;    /* IMAGE_F_[...]. */
    struct image_version ih_ver;
    uint32_t _pad2;
};
/*
 * When accompanied by image, it's this structure followed by data.
 * Response contains just the offset.
 */
struct imgmgr_upload_cmd {
    uint32_t iuc_off;
};

/*
 * Response to list:
 * {
 *      "images":[ <version1>, <version2>]
 * }
 *
 *
 * Request to boot to version:
 * {
 *      "test":<version>
 * }
 *
 *
 * Response to boot read:
 * {
 *	"test":<version>,
 *	"main":<version>,
 *      "active":<version>
 * }
 *
 *
 * Request to image upload:
 * {
 *      "off":<offset>,
 *      "len":<img_size>		inspected when off = 0
 *      "data":<base64encoded binary>
 * }
 *
 *
 * Response to upload:
 * {
 *      "off":<offset>
 * }
 *
 *
 * Request to image upload:
 * {
 *      "off":<offset>
 *	"name":<filename>		inspected when off = 0
 *      "len":<file_size>		inspected when off = 0
 *      "data":<base64encoded binary>
 * }
 */

struct mgmt_cbuf;

int imgr_core_list(struct mgmt_cbuf *);
int imgr_core_load(struct mgmt_cbuf *);
int imgr_core_erase(struct mgmt_cbuf *);
int imgmgr_state_read(struct mgmt_cbuf *cb);
int imgmgr_state_write(struct mgmt_cbuf *njb);
int imgr_find_by_ver(struct image_version *find, uint8_t *hash);
int imgr_find_by_hash(uint8_t *find, struct image_version *ver);
int imgr_cli_register(void);
int image_to_bl(int src_fa_id);
uint32_t imgr_magic(int image_slot);
int imgr_v2image_set_confirmed(void);
int image_to_bl(int src_fa_id);

#ifdef __cplusplus
}
#endif

#endif /* __IMGMGR_PRIV_H */
