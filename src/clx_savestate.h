#ifndef CLX_SAVESTATE_H
#define CLX_SAVESTATE_H

#include <stdint.h>
#include <stddef.h>
#include "kernel/xbox_memory_layout.h"
#include "../../xboxrecomp/include/xbox/recomp_regs.h"

#define CLX_SAVESTATE_MAGIC 0x54534C43 /* "CLST" */
#define CLX_SAVESTATE_VERSION 6
#define CLX_SAVESTATE_NUM_SLOTS 5

typedef struct ClxSaveStateHeader {
    uint32_t magic;              /* CLST (0x54534C43) */
    uint32_t version;            /* 6 */
    uint32_t header_size;        /* sizeof(ClxSaveStateHeader) */
    uint32_t slot;               /* 1..5 */
    uint64_t timestamp;          /* time(NULL) */

    /* Uncompressed and compressed payload sizes & CRCs */
    uint32_t ram_size;           /* typically 64MB */
    uint32_t ram_compressed_size;
    uint32_t ram_crc32;

    uint32_t contig_size;        /* 0 if contiguous arena not saved */
    uint32_t contig_compressed_size;
    uint32_t contig_crc32;

    XboxMemLayoutSnapshot layout;
    uint32_t layout_blocks_size;
    uint32_t layout_blocks_crc32;

    RecompRegs regs;

    /* NV2A pushbuffer cursors live outside low and contiguous RAM. */
    uint32_t nv2a_dma_put;
    uint32_t nv2a_dma_get;

    /* Total payload size and CRC32 of payload following header */
    uint32_t payload_size;
    uint32_t payload_crc32;

    /* CRC32 of header fields up to header_crc32 */
    uint32_t header_crc32;

    uint8_t reserved[64];
} ClxSaveStateHeader;

#ifdef __cplusplus
extern "C" {
#endif

void clx_savestate_init(void);
const char *clx_savestate_dir(void);
int  clx_savestate_exists(int slot);
int  clx_savestate_slot_info(int slot, char *out_desc, size_t max_desc);

void clx_savestate_request_save(int slot);
void clx_savestate_request_load(int slot);
int  clx_savestate_is_pending(void);

/* Called at the per-frame safe point in sub_0053C190 */
void clx_savestate_tick(void);

/* Low-level routines (usable directly and in unit tests) */
int clx_savestate_do_save(int slot);
int clx_savestate_do_load(int slot);

#ifdef __cplusplus
}
#endif

#endif /* CLX_SAVESTATE_H */
