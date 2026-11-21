#include "clx_savestate.h"
#include "clx_settings.h"
#include "kernel/xbox_memory_layout.h"
#include "kernel/nv2a_backend.h"
#include "apu/apu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include <stdatomic.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <zlib.h>

#if defined(_WIN32)
#include <direct.h>
#include <windows.h>
#define mkdir_portable(p) _mkdir(p)
#else
#include <unistd.h>
#define mkdir_portable(p) mkdir(p, 0755)
#endif

#define CLX_D3D_DEVICE_PTR 0x005499E8u
#define CLX_NV2A_USER_DMA_PUT 0xFD800040u
#define CLX_NV2A_USER_DMA_GET 0xFD800044u
#define CLX_SAVESTATE_PATH_MAX 576

extern void nv2a_pb_set_semaphore_target(uint32_t guest_va);
extern void nv2a_pb_resync(uint32_t get_phys);
extern volatile int g_audio_muted;
extern int xbox_gil_suspend(void);
extern void xbox_gil_resume(int depth);
extern void xbox_gil_enter(void);
extern void xbox_gil_leave(void);
extern uint8_t xbox_CurrentIrql(void);
extern void xbox_RestoreCurrentIrql(uint8_t irql);
extern int xbox_IrqlRaisedCount(void);
extern void xbox_IrqlDumpHolders(void);

enum {
    STATE_ACTION_NONE = 0,
    STATE_ACTION_SAVE = 1,
    STATE_ACTION_LOAD = 2
};

static atomic_int s_pending_action = STATE_ACTION_NONE;
static atomic_int s_pending_slot = 1;

static inline uintptr_t guest_ptr(uint32_t addr)
{
    return (uintptr_t)addr + (uintptr_t)xbox_GetMemoryOffset();
}

static inline uint32_t guest_mem32_rd(uint32_t addr)
{
    return *(volatile uint32_t *)guest_ptr(addr);
}

static inline void guest_mem32_wr(uint32_t addr, uint32_t val)
{
    *(volatile uint32_t *)guest_ptr(addr) = val;
}

static void save_irql_extension(ClxSaveStateHeader *hdr, uint8_t irql)
{
    uint32_t crc;
    uint32_t raised_count = (uint32_t)xbox_IrqlRaisedCount();
    uint8_t *safe_ext = hdr->reserved + 33;
    hdr->reserved[0] = irql;
    memcpy(hdr->reserved + 1, "IRQL", 4);
    crc = (uint32_t)crc32(crc32(0L, Z_NULL, 0), hdr->reserved, 5);
    memcpy(hdr->reserved + 5, &crc, sizeof(crc));
    memcpy(safe_ext, "IRQ0", 4);
    memcpy(safe_ext + 4, &raised_count, sizeof(raised_count));
    crc = (uint32_t)crc32(crc32(0L, Z_NULL, 0), safe_ext, 8);
    memcpy(safe_ext + 8, &crc, sizeof(crc));
}

static void save_audio_extension(ClxSaveStateHeader *hdr, uint32_t audio_size,
                                 uint32_t audio_compressed_size, uint32_t audio_crc32)
{
    uint8_t *ext = hdr->reserved + 9;
    uint32_t version = 1;
    uint32_t ext_crc;

    memcpy(ext, "APU3", 4);
    memcpy(ext + 4, &version, sizeof(version));
    memcpy(ext + 8, &audio_size, sizeof(audio_size));
    memcpy(ext + 12, &audio_compressed_size, sizeof(audio_compressed_size));
    memcpy(ext + 16, &audio_crc32, sizeof(audio_crc32));
    ext_crc = (uint32_t)crc32(crc32(0L, Z_NULL, 0), ext, 20);
    memcpy(ext + 20, &ext_crc, sizeof(ext_crc));
}

static void save_gpu_extension(ClxSaveStateHeader *hdr, uint32_t gpu_size,
                               uint32_t gpu_compressed_size, uint32_t gpu_crc32)
{
    uint8_t *ext = hdr->reserved + 45;
    memcpy(ext, "GPU0", 4);
    memcpy(ext + 4, &gpu_size, sizeof(gpu_size));
    memcpy(ext + 8, &gpu_compressed_size, sizeof(gpu_compressed_size));
    memcpy(ext + 12, &gpu_crc32, sizeof(gpu_crc32));
}

static int load_gpu_extension(const ClxSaveStateHeader *hdr, uint32_t *gpu_size,
                              uint32_t *gpu_compressed_size, uint32_t *gpu_crc32)
{
    const uint8_t *ext = hdr->reserved + 45;
    *gpu_size = 0;
    *gpu_compressed_size = 0;
    *gpu_crc32 = 0;
    if (hdr->version != CLX_SAVESTATE_VERSION || memcmp(ext, "GPU0", 4) != 0)
        return 0;
    memcpy(gpu_size, ext + 4, sizeof(*gpu_size));
    memcpy(gpu_compressed_size, ext + 8, sizeof(*gpu_compressed_size));
    memcpy(gpu_crc32, ext + 12, sizeof(*gpu_crc32));
    return *gpu_size > 0 && *gpu_compressed_size > 0;
}

static int load_audio_extension(const ClxSaveStateHeader *hdr,
                                uint32_t *audio_size,
                                uint32_t *audio_compressed_size,
                                uint32_t *audio_crc32)
{
    const uint8_t *ext = hdr->reserved + 9;
    uint32_t version, stored_crc, actual_crc;

    if (hdr->version != CLX_SAVESTATE_VERSION || memcmp(ext, "APU3", 4) != 0)
        return 0;

    memcpy(&version, ext + 4, sizeof(version));
    memcpy(audio_size, ext + 8, sizeof(*audio_size));
    memcpy(audio_compressed_size, ext + 12, sizeof(*audio_compressed_size));
    memcpy(audio_crc32, ext + 16, sizeof(*audio_crc32));
    memcpy(&stored_crc, ext + 20, sizeof(stored_crc));
    actual_crc = (uint32_t)crc32(crc32(0L, Z_NULL, 0), ext, 20);
    return version == 1 && *audio_size > 0 &&
           *audio_compressed_size > 0 && stored_crc == actual_crc;
}

static int load_irql_extension(const ClxSaveStateHeader *hdr, uint8_t *irql)
{
    const uint8_t *safe_ext = hdr->reserved + 33;
    uint32_t crc, raised_count, safe_crc;
    if (!memcmp(hdr->reserved, "\0\0\0\0\0\0\0\0\0", 9)) {
        *irql = 0;
    } else {
        if (memcmp(hdr->reserved + 1, "IRQL", 4) != 0 || hdr->reserved[0] > 31)
            return 0;
        memcpy(&crc, hdr->reserved + 5, sizeof(crc));
        if (crc != (uint32_t)crc32(crc32(0L, Z_NULL, 0), hdr->reserved, 5))
            return 0;
        *irql = hdr->reserved[0];
    }

    if (memcmp(safe_ext, "IRQ0", 4) != 0)
        return 0;
    memcpy(&raised_count, safe_ext + 4, sizeof(raised_count));
    memcpy(&safe_crc, safe_ext + 8, sizeof(safe_crc));
    crc = (uint32_t)crc32(crc32(0L, Z_NULL, 0), safe_ext, 8);
    if (raised_count != 0 || safe_crc != crc)
        return 0;
    return 1;
}

static int wait_for_irql_idle(unsigned timeout_ms)
{
    struct timespec delay = { 0, 1000000 };
    int gil_depth;

    if (xbox_CurrentIrql() != 0) {
        fprintf(stderr, "[savestate] cannot snapshot while current thread IRQL is raised\n");
        xbox_IrqlDumpHolders();
        return 0;
    }

    for (unsigned elapsed = 0; elapsed < timeout_ms; elapsed++) {
        if (xbox_IrqlRaisedCount() == 0)
            return 1;
        gil_depth = xbox_gil_suspend();
#if defined(_WIN32)
        Sleep(1);
#else
        nanosleep(&delay, NULL);
#endif
        xbox_gil_resume(gil_depth);
        if (xbox_CurrentIrql() != 0) {
            fprintf(stderr, "[savestate] current thread IRQL changed while waiting\n");
            xbox_IrqlDumpHolders();
            return 0;
        }
    }

    if (xbox_IrqlRaisedCount() == 0)
        return 1;
    fprintf(stderr, "[savestate] guest IRQL did not return to PASSIVE within %u ms\n",
            timeout_ms);
    xbox_IrqlDumpHolders();
    return 0;
}

static int wait_for_nv2a_idle(int release_guest)
{
    volatile uint32_t *put = (volatile uint32_t *)guest_ptr(CLX_NV2A_USER_DMA_PUT);
    volatile uint32_t *get = (volatile uint32_t *)guest_ptr(CLX_NV2A_USER_DMA_GET);
    struct timespec delay = { 0, 1000000 };
    int gil_depth = release_guest ? xbox_gil_suspend() : 0;
    int idle = 0;

    for (unsigned elapsed = 0; elapsed < 2000; elapsed++) {
        uint32_t put_now = *put;
        uint32_t get_now = *get;
        if (put_now == get_now) {
            idle = 1;
            break;
        }
#if defined(_WIN32)
        Sleep(1);
#else
        nanosleep(&delay, NULL);
#endif
    }

    if (release_guest)
        xbox_gil_resume(gil_depth);
    if (!idle)
        fprintf(stderr, "[savestate] pushbuffer did not drain (PUT 0x%08X, GET 0x%08X)\n",
                *put, *get);
    return idle;
}

static void ensure_dir_exists(const char *path)
{
    struct stat st;
    if (stat(path, &st) != 0) {
        mkdir_portable(path);
    }
}

const char *clx_savestate_dir(void)
{
    static char s_dir[512] = { 0 };
    if (s_dir[0]) return s_dir;

    const char *env = getenv("CLX_SAVESTATE_DIR");
    if (env && *env) {
        snprintf(s_dir, sizeof(s_dir), "%s", env);
    } else {
#if defined(__SWITCH__)
        snprintf(s_dir, sizeof(s_dir), "sdmc:/switch/clx/savestates");
#elif defined(__ANDROID__)
        snprintf(s_dir, sizeof(s_dir), "/storage/emulated/0/ConkerLR/savestates");
#else
        snprintf(s_dir, sizeof(s_dir), "savestates");
#endif
    }
    return s_dir;
}

void clx_savestate_init(void)
{
    ensure_dir_exists(clx_savestate_dir());
}

static void get_slot_path(int slot, char *out, size_t max_len, int tmp)
{
    const char *dir = clx_savestate_dir();
    snprintf(out, max_len, "%s/conker_slot%d.state%s", dir, slot, tmp ? ".tmp" : "");
}

int clx_savestate_exists(int slot)
{
    if (slot < 1 || slot > CLX_SAVESTATE_NUM_SLOTS) return 0;
    char path[CLX_SAVESTATE_PATH_MAX];
    get_slot_path(slot, path, sizeof(path), 0);
    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    ClxSaveStateHeader hdr;
    size_t rd = fread(&hdr, 1, sizeof(hdr), f);
    fclose(f);
    if (rd < sizeof(hdr)) return 0;
    if (hdr.magic != CLX_SAVESTATE_MAGIC || hdr.version != CLX_SAVESTATE_VERSION)
        return 0;

    return 1;
}

int clx_savestate_slot_info(int slot, char *out_desc, size_t max_desc)
{
    if (!out_desc || max_desc == 0) return 0;
    out_desc[0] = '\0';
    if (slot < 1 || slot > CLX_SAVESTATE_NUM_SLOTS) {
        snprintf(out_desc, max_desc, "INVALID");
        return 0;
    }
    char path[CLX_SAVESTATE_PATH_MAX];
    get_slot_path(slot, path, sizeof(path), 0);
    FILE *f = fopen(path, "rb");
    if (!f) {
        snprintf(out_desc, max_desc, "EMPTY");
        return 0;
    }

    ClxSaveStateHeader hdr;
    size_t rd = fread(&hdr, 1, sizeof(hdr), f);
    fclose(f);
    if (rd < sizeof(hdr) || hdr.magic != CLX_SAVESTATE_MAGIC ||
        hdr.version != CLX_SAVESTATE_VERSION) {
        snprintf(out_desc, max_desc, "CORRUPT");
        return 0;
    }

    time_t t = (time_t)hdr.timestamp;
    struct tm *tm_info = localtime(&t);
    if (tm_info) {
        snprintf(out_desc, max_desc, "%02d/%02d %02d:%02d",
                 tm_info->tm_mon + 1, tm_info->tm_mday,
                 tm_info->tm_hour, tm_info->tm_min);
    } else {
        snprintf(out_desc, max_desc, "OCCUPIED");
    }
    return 1;
}

void clx_savestate_request_save(int slot)
{
    if (slot < 1 || slot > CLX_SAVESTATE_NUM_SLOTS) return;
    atomic_store(&s_pending_slot, slot);
    atomic_store(&s_pending_action, STATE_ACTION_SAVE);
}

void clx_savestate_request_load(int slot)
{
    if (slot < 1 || slot > CLX_SAVESTATE_NUM_SLOTS) return;
    atomic_store(&s_pending_slot, slot);
    atomic_store(&s_pending_action, STATE_ACTION_LOAD);
}

int clx_savestate_is_pending(void)
{
    return atomic_load(&s_pending_action) != STATE_ACTION_NONE;
}

static int write_compressed(FILE *f, const void *data, size_t size,
                            uLong *payload_crc, uint32_t *compressed_size)
{
    z_stream strm;
    unsigned char out_chunk[256 * 1024];
    uint64_t bytes_written = 0;
    int ret;

    if (size > UINT_MAX)
        return 0;

    memset(&strm, 0, sizeof(strm));
    if (deflateInit(&strm, 1) != Z_OK)
        return 0;

    strm.next_in = (Bytef *)data;
    strm.avail_in = (uInt)size;
    do {
        size_t have;
        strm.next_out = out_chunk;
        strm.avail_out = sizeof(out_chunk);
        ret = deflate(&strm, strm.avail_in ? Z_NO_FLUSH : Z_FINISH);
        if (ret != Z_OK && ret != Z_STREAM_END) {
            deflateEnd(&strm);
            return 0;
        }

        have = sizeof(out_chunk) - strm.avail_out;
        if (have && fwrite(out_chunk, 1, have, f) != have) {
            deflateEnd(&strm);
            return 0;
        }
        *payload_crc = crc32(*payload_crc, out_chunk, (uInt)have);
        bytes_written += have;
        if (bytes_written > UINT32_MAX) {
            deflateEnd(&strm);
            return 0;
        }
    } while (ret != Z_STREAM_END);

    deflateEnd(&strm);
    *compressed_size = (uint32_t)bytes_written;
    return 1;
}

int clx_savestate_do_save(int slot)
{
    int ok = 0;
    int pb_paused = 0;
    int apu_paused = 0;
    FILE *f = NULL;
    void *blocks_buf = NULL, *ram_copy = NULL, *contig_copy = NULL;
    void *audio_copy = NULL, *gpu_copy = NULL;
    size_t ram_size, contig_size, audio_size, gpu_size, needed_blocks_size;
    uint64_t payload_size;
    uint32_t audio_crc32, audio_compressed_size = 0;
    uint32_t gpu_crc32, gpu_compressed_size = 0;
    uLong payload_crc = crc32(0L, Z_NULL, 0);
    char tmp_path[CLX_SAVESTATE_PATH_MAX], final_path[CLX_SAVESTATE_PATH_MAX];
    ClxSaveStateHeader hdr;

    if (slot < 1 || slot > CLX_SAVESTATE_NUM_SLOTS)
        return 0;
    xbox_gil_enter();
    ensure_dir_exists(clx_savestate_dir());
    get_slot_path(slot, tmp_path, sizeof(tmp_path), 1);
    get_slot_path(slot, final_path, sizeof(final_path), 0);

    memset(&hdr, 0, sizeof(hdr));
    hdr.magic = CLX_SAVESTATE_MAGIC;
    hdr.version = CLX_SAVESTATE_VERSION;
    hdr.header_size = (uint32_t)sizeof(hdr);
    hdr.slot = (uint32_t)slot;
    hdr.timestamp = (uint64_t)time(NULL);
    if (!wait_for_nv2a_idle(1))
        goto done;
    if (!xbox_Nv2aPausePushbuffer(2000)) {
        fprintf(stderr, "[savestate] timed out pausing the pushbuffer executor\n");
        goto done;
    }
    pb_paused = 1;
    if (!mcpx_apu_pause_for_savestate(2000))
        goto done;
    apu_paused = 1;
    if (!wait_for_irql_idle(2000))
        goto done;
    save_irql_extension(&hdr, xbox_CurrentIrql());
    audio_size = mcpx_apu_savestate_size();
    if (audio_size == 0 || audio_size > UINT32_MAX || audio_size > UINT_MAX) {
        fprintf(stderr, "[savestate] invalid APU snapshot size\n");
        goto done;
    }
    audio_copy = malloc(audio_size);
    if (!audio_copy || !mcpx_apu_savestate_save(audio_copy, audio_size)) {
        fprintf(stderr, "[savestate] APU snapshot failed\n");
        goto done;
    }
    audio_crc32 = (uint32_t)crc32(crc32(0L, Z_NULL, 0),
                                  (const Bytef *)audio_copy, (uInt)audio_size);
    gpu_size = nv2a_pb_exec_savestate_size();
    if (gpu_size == 0 || gpu_size > UINT32_MAX || gpu_size > UINT_MAX) {
        fprintf(stderr, "[savestate] invalid NV2A snapshot size\n");
        goto done;
    }
    gpu_copy = malloc(gpu_size);
    if (!gpu_copy || !nv2a_pb_exec_savestate_save(gpu_copy, gpu_size)) {
        fprintf(stderr, "[savestate] NV2A snapshot failed\n");
        goto done;
    }
    gpu_crc32 = (uint32_t)crc32(crc32(0L, Z_NULL, 0),
                                (const Bytef *)gpu_copy, (uInt)gpu_size);
    hdr.nv2a_dma_put = guest_mem32_rd(CLX_NV2A_USER_DMA_PUT);
    hdr.nv2a_dma_get = guest_mem32_rd(CLX_NV2A_USER_DMA_GET);
    if (hdr.nv2a_dma_put != hdr.nv2a_dma_get) {
        fprintf(stderr, "[savestate] pushbuffer changed before snapshot (PUT 0x%08X, GET 0x%08X)\n",
                hdr.nv2a_dma_put, hdr.nv2a_dma_get);
        goto done;
    }
    hdr.regs = _rr[0];

    needed_blocks_size = xbox_MemLayoutSave(&hdr.layout, NULL, 0);
    if (needed_blocks_size > 0) {
        blocks_buf = malloc(needed_blocks_size);
        if (!blocks_buf || xbox_MemLayoutSave(&hdr.layout, blocks_buf, needed_blocks_size) != needed_blocks_size)
            goto done;
        if (needed_blocks_size > UINT32_MAX || needed_blocks_size > UINT_MAX)
            goto done;
        hdr.layout_blocks_size = (uint32_t)needed_blocks_size;
        hdr.layout_blocks_crc32 = (uint32_t)crc32(
            crc32(0L, Z_NULL, 0), (const Bytef *)blocks_buf, (uInt)needed_blocks_size);
    }

    ram_size = g_xbox_total_ram ? g_xbox_total_ram : (64 * 1024 * 1024);
    if (!xbox_GetMemoryBase() || ram_size > UINT32_MAX || ram_size > UINT_MAX)
        goto done;
    ram_copy = malloc(ram_size);
    if (!ram_copy)
        goto done;
    memcpy(ram_copy, xbox_GetMemoryBase(), ram_size);
    hdr.ram_size = (uint32_t)ram_size;
    hdr.ram_crc32 = (uint32_t)crc32(crc32(0L, Z_NULL, 0), ram_copy, (uInt)ram_size);

    contig_size = xbox_GetContigMemoryUsed();
    if (contig_size > UINT32_MAX || contig_size > UINT_MAX)
        goto done;
    if (contig_size > 0) {
        if (!xbox_GetContigMemoryBase())
            goto done;
        contig_copy = malloc(contig_size);
        if (!contig_copy)
            goto done;
        memcpy(contig_copy, xbox_GetContigMemoryBase(), contig_size);
    }
    hdr.contig_size = (uint32_t)contig_size;
    if (contig_copy)
        hdr.contig_crc32 = (uint32_t)crc32(crc32(0L, Z_NULL, 0), contig_copy, (uInt)contig_size);

    f = fopen(tmp_path, "wb");
    if (!f) {
        fprintf(stderr, "[savestate] failed to open for writing: %s\n", tmp_path);
        goto done;
    }
    if (fwrite(&hdr, sizeof(hdr), 1, f) != 1)
        goto done;
    if (blocks_buf && needed_blocks_size > 0) {
        if (fwrite(blocks_buf, 1, needed_blocks_size, f) != needed_blocks_size)
            goto done;
        payload_crc = crc32(payload_crc, (const Bytef *)blocks_buf, (uInt)needed_blocks_size);
    }
    if (!write_compressed(f, ram_copy, ram_size, &payload_crc, &hdr.ram_compressed_size))
        goto done;
    if (contig_copy &&
        !write_compressed(f, contig_copy, contig_size, &payload_crc, &hdr.contig_compressed_size))
        goto done;
    if (!write_compressed(f, audio_copy, audio_size, &payload_crc,
                          &audio_compressed_size)) {
        goto done;
    }
    if (!write_compressed(f, gpu_copy, gpu_size, &payload_crc,
                          &gpu_compressed_size)) {
        goto done;
    }

    payload_size = (uint64_t)hdr.layout_blocks_size +
                   hdr.ram_compressed_size + hdr.contig_compressed_size +
                   audio_compressed_size + gpu_compressed_size;
    if (payload_size > UINT32_MAX)
        goto done;
    save_audio_extension(&hdr, (uint32_t)audio_size, audio_compressed_size,
                         audio_crc32);
    save_gpu_extension(&hdr, (uint32_t)gpu_size, gpu_compressed_size, gpu_crc32);
    hdr.payload_size = (uint32_t)payload_size;
    hdr.payload_crc32 = (uint32_t)payload_crc;
    hdr.header_crc32 = (uint32_t)crc32(crc32(0L, Z_NULL, 0),
                                      (const Bytef *)&hdr,
                                      offsetof(ClxSaveStateHeader, header_crc32));

    if (fseek(f, 0, SEEK_SET) != 0 || fwrite(&hdr, sizeof(hdr), 1, f) != 1 ||
        fflush(f) != 0) {
        goto done;
    }
    if (fclose(f) != 0) {
        f = NULL;
        goto done;
    }
    f = NULL;

#if defined(_WIN32)
    remove(final_path);
#endif
    if (rename(tmp_path, final_path) != 0) {
        fprintf(stderr, "[savestate] failed to rename %s to %s\n", tmp_path, final_path);
        goto done;
    }
    ok = 1;
done:
    if (f)
        fclose(f);
    free(blocks_buf);
    free(ram_copy);
    free(contig_copy);
    free(audio_copy);
    free(gpu_copy);
    if (!ok)
        remove(tmp_path);
    if (apu_paused)
        mcpx_apu_resume_after_savestate(0, 0);
    if (pb_paused)
        xbox_Nv2aResumePushbuffer();
    xbox_gil_leave();
    return ok;
}

static int read_compressed(FILE *f, uint32_t compressed_size, void *output,
                           uint32_t output_size, uint32_t expected_crc,
                           const char *section)
{
    unsigned char *compressed;
    uLongf actual_size = output_size;
    int ret;

    if (compressed_size == 0 || output_size == 0)
        return 0;
    compressed = malloc(compressed_size);
    if (!compressed)
        return 0;
    if (fread(compressed, 1, compressed_size, f) != compressed_size) {
        free(compressed);
        fprintf(stderr, "[savestate] unexpected EOF reading %s stream\n", section);
        return 0;
    }

    ret = uncompress((Bytef *)output, &actual_size, compressed, compressed_size);
    free(compressed);
    if (ret != Z_OK || actual_size != output_size) {
        fprintf(stderr, "[savestate] %s decompression failed (%d, %lu/%u bytes)\n",
                section, ret, (unsigned long)actual_size, output_size);
        return 0;
    }

    uint32_t actual_crc = (uint32_t)crc32(crc32(0L, Z_NULL, 0),
                                          (const Bytef *)output, output_size);
    if (actual_crc != expected_crc) {
        fprintf(stderr, "[savestate] %s CRC mismatch (0x%08X != 0x%08X)\n",
                section, actual_crc, expected_crc);
        return 0;
    }
    return 1;
}

int clx_savestate_do_load(int slot)
{
    int ok = 0;
    int pb_paused = 0;
    int apu_paused = 0;
    int audio_restored = 0;
    FILE *f = NULL;
    void *blocks_buf = NULL, *ram_copy = NULL, *contig_copy = NULL;
    void *audio_copy = NULL, *gpu_copy = NULL, *apu_restore_plan = NULL;
    size_t cur_ram_size = g_xbox_total_ram ? g_xbox_total_ram : (64 * 1024 * 1024);
    size_t layout_bytes;
    uint64_t expected_payload_size;
    char path[CLX_SAVESTATE_PATH_MAX];
    uint8_t saved_irql;
    uint32_t audio_size, audio_compressed_size, audio_crc32;
    uint32_t gpu_size, gpu_compressed_size, gpu_crc32;

    if (slot < 1 || slot > CLX_SAVESTATE_NUM_SLOTS)
        return 0;
    xbox_gil_enter();
    get_slot_path(slot, path, sizeof(path), 0);

    f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[savestate] could not open %s\n", path);
        goto done;
    }

    ClxSaveStateHeader hdr;
    if (fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr)) {
        goto done;
    }

    if (hdr.magic != CLX_SAVESTATE_MAGIC ||
        hdr.version != CLX_SAVESTATE_VERSION ||
        hdr.header_size != sizeof(hdr)) {
        fprintf(stderr, "[savestate] header signature mismatch (magic 0x%08X, ver %u)\n", hdr.magic, hdr.version);
        goto done;
    }

    uint32_t calc_hdr_crc = (uint32_t)crc32(crc32(0L, Z_NULL, 0),
                                            (const Bytef *)&hdr,
                                            offsetof(ClxSaveStateHeader, header_crc32));
    if (calc_hdr_crc != hdr.header_crc32) {
        fprintf(stderr, "[savestate] header CRC mismatch (calc 0x%08X != hdr 0x%08X)\n", calc_hdr_crc, hdr.header_crc32);
        goto done;
    }
    if (!load_irql_extension(&hdr, &saved_irql)) {
        fprintf(stderr, "[savestate] slot %d lacks valid safe-IRQL metadata; create a fresh state\n",
                slot);
        goto done;
    }
    if (!load_audio_extension(&hdr, &audio_size, &audio_compressed_size,
                              &audio_crc32)) {
        fprintf(stderr, "[savestate] invalid APU snapshot metadata in slot %d\n", slot);
        goto done;
    }
    if (!load_gpu_extension(&hdr, &gpu_size, &gpu_compressed_size, &gpu_crc32)) {
        fprintf(stderr, "[savestate] slot %d lacks an NV2A snapshot; create a fresh state\n",
                slot);
        goto done;
    }
    if (hdr.slot != (uint32_t)slot) {
        fprintf(stderr, "[savestate] slot mismatch (state is slot %u, requested %d)\n", hdr.slot, slot);
        goto done;
    }
    if (hdr.payload_size > (uint32_t)512 * 1024 * 1024u ||
        hdr.layout_blocks_size > (uint32_t)32 * 1024 * 1024u ||
        hdr.ram_compressed_size > (uint32_t)512 * 1024 * 1024u ||
        hdr.contig_compressed_size > (uint32_t)512 * 1024 * 1024u ||
        audio_size > (uint32_t)256 * 1024 * 1024u ||
        audio_compressed_size > (uint32_t)256 * 1024 * 1024u ||
        gpu_size > (uint32_t)64 * 1024 * 1024u ||
        gpu_compressed_size > (uint32_t)64 * 1024 * 1024u) {
        fprintf(stderr, "[savestate] unreasonable payload size in slot %d\n", slot);
        goto done;
    }
    expected_payload_size = (uint64_t)hdr.layout_blocks_size +
                            hdr.ram_compressed_size + hdr.contig_compressed_size +
                            audio_compressed_size + gpu_compressed_size;
    if (hdr.payload_size != expected_payload_size ||
        hdr.layout.heap_block_count < 0 || hdr.layout.contig_count < 0) {
        fprintf(stderr, "[savestate] payload size mismatch (header %u, sections %u)\n",
                hdr.payload_size, (uint32_t)expected_payload_size);
        goto done;
    }
    if (hdr.nv2a_dma_put != hdr.nv2a_dma_get) {
        fprintf(stderr, "[savestate] state has an in-flight pushbuffer (PUT 0x%08X, GET 0x%08X)\n",
                hdr.nv2a_dma_put, hdr.nv2a_dma_get);
        goto done;
    }
    layout_bytes = ((size_t)hdr.layout.heap_block_count + (size_t)hdr.layout.contig_count) *
                   sizeof(XboxMemLayoutBlock);
    if (layout_bytes != hdr.layout_blocks_size ||
        hdr.ram_size != cur_ram_size ||
        (hdr.contig_size == 0) != (hdr.contig_compressed_size == 0) ||
        hdr.ram_compressed_size == 0 ||
        ((audio_size == 0) != (audio_compressed_size == 0)) ||
        (hdr.nv2a_dma_put & 0x0FFFFFFCu) >= 64u * 1024u * 1024u ||
        (hdr.nv2a_dma_get & 0x0FFFFFFCu) >= 64u * 1024u * 1024u ||
        !xbox_GetMemoryBase() ||
        (hdr.contig_size > 0 && (!xbox_GetContigMemoryBase() ||
                                 hdr.contig_size > 64u * 1024u * 1024u))) {
        fprintf(stderr, "[savestate] incompatible memory layout in slot %d\n", slot);
        goto done;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fprintf(stderr, "[savestate] could not seek to end of slot %d\n", slot);
        goto done;
    }
    long file_size = ftell(f);
    if (file_size < 0 || (uint64_t)file_size != sizeof(hdr) + expected_payload_size) {
        fprintf(stderr, "[savestate] file size mismatch in slot %d\n", slot);
        goto done;
    }

    /* Verify the complete payload before allocating or restoring game memory. */
    uLong check_payload_crc = crc32(0L, Z_NULL, 0);
    unsigned char check_buf[256 * 1024];
    uint32_t bytes_to_check = hdr.payload_size;
    if (fseek(f, sizeof(hdr), SEEK_SET) != 0)
        goto done;
    while (bytes_to_check > 0) {
        size_t to_read = bytes_to_check < sizeof(check_buf) ? bytes_to_check : sizeof(check_buf);
        if (fread(check_buf, 1, to_read, f) != to_read) {
            fprintf(stderr, "[savestate] unexpected EOF reading payload\n");
            goto done;
        }
        check_payload_crc = crc32(check_payload_crc, check_buf, (uInt)to_read);
        bytes_to_check -= (uint32_t)to_read;
    }

    if ((uint32_t)check_payload_crc != hdr.payload_crc32) {
        fprintf(stderr, "[savestate] payload CRC mismatch (calc 0x%08X != hdr 0x%08X)\n", (uint32_t)check_payload_crc, hdr.payload_crc32);
        goto done;
    }

    if (fseek(f, sizeof(hdr), SEEK_SET) != 0)
        goto done;
    if (hdr.layout_blocks_size > 0) {
        blocks_buf = malloc(hdr.layout_blocks_size);
        if (!blocks_buf)
            goto done;
        if (fread(blocks_buf, 1, hdr.layout_blocks_size, f) != hdr.layout_blocks_size) {
            fprintf(stderr, "[savestate] unexpected EOF reading layout blocks\n");
            goto done;
        }
        uint32_t calc_layout_crc = (uint32_t)crc32(
            crc32(0L, Z_NULL, 0), (const Bytef *)blocks_buf, hdr.layout_blocks_size);
        if (calc_layout_crc != hdr.layout_blocks_crc32) {
            fprintf(stderr, "[savestate] layout block CRC mismatch (0x%08X != 0x%08X)\n",
                    calc_layout_crc, hdr.layout_blocks_crc32);
            goto done;
        }
    }

    ram_copy = malloc(hdr.ram_size);
    if (!ram_copy)
        goto done;
    if (!read_compressed(f, hdr.ram_compressed_size, ram_copy, hdr.ram_size,
                         hdr.ram_crc32, "Low RAM"))
        goto done;

    if (hdr.contig_size > 0) {
        contig_copy = malloc(hdr.contig_size);
        if (!contig_copy)
            goto done;
        if (!read_compressed(f, hdr.contig_compressed_size, contig_copy,
                             hdr.contig_size, hdr.contig_crc32, "Contiguous RAM"))
            goto done;
    }
    if (audio_size > 0) {
        audio_copy = malloc(audio_size);
        if (!audio_copy)
            goto done;
        if (!read_compressed(f, audio_compressed_size, audio_copy, audio_size,
                             audio_crc32, "APU"))
            goto done;
        apu_restore_plan = mcpx_apu_savestate_prepare(audio_copy, audio_size);
        if (!apu_restore_plan)
            goto done;
        audio_restored = 1;
    }
    gpu_copy = malloc(gpu_size);
    if (!gpu_copy ||
        !read_compressed(f, gpu_compressed_size, gpu_copy, gpu_size,
                         gpu_crc32, "NV2A") ||
        !nv2a_pb_exec_savestate_validate(gpu_copy, gpu_size))
        goto done;

    if (!wait_for_nv2a_idle(1))
        goto done;
    if (!xbox_Nv2aPausePushbuffer(2000)) {
        fprintf(stderr, "[savestate] timed out pausing the pushbuffer executor\n");
        goto done;
    }
    pb_paused = 1;
    if (!mcpx_apu_pause_for_savestate(2000))
        goto done;
    apu_paused = 1;
    if (!wait_for_irql_idle(2000))
        goto done;
    uint32_t live_put = guest_mem32_rd(CLX_NV2A_USER_DMA_PUT);
    uint32_t live_get = guest_mem32_rd(CLX_NV2A_USER_DMA_GET);
    if (live_put != live_get) {
        fprintf(stderr, "[savestate] pushbuffer changed before restore (PUT 0x%08X, GET 0x%08X)\n",
                live_put, live_get);
        goto done;
    }

    g_audio_muted = 1;
    if (!xbox_MemLayoutRestore(&hdr.layout, blocks_buf, hdr.layout_blocks_size)) {
        fprintf(stderr, "[savestate] memory layout restore failed\n");
        g_audio_muted = 0;
        goto done;
    }

    /* All validation is complete; only now replace live game memory. */
    uint32_t cur_ret = guest_mem32_rd(_rr[0].r_esp);
    memcpy(xbox_GetMemoryBase(), ram_copy, hdr.ram_size);
    if (contig_copy)
        memcpy(xbox_GetContigMemoryBase(), contig_copy, hdr.contig_size);
    if (!nv2a_pb_exec_savestate_restore(gpu_copy, gpu_size)) {
        fprintf(stderr, "[savestate] NV2A state restore failed\n");
        g_audio_muted = 0;
        goto done;
    }
    guest_mem32_wr(CLX_NV2A_USER_DMA_PUT, hdr.nv2a_dma_put);
    guest_mem32_wr(CLX_NV2A_USER_DMA_GET, hdr.nv2a_dma_get);
    nv2a_pb_resync(hdr.nv2a_dma_get);
    _rr[0] = hdr.regs;
    guest_mem32_wr(_rr[0].r_esp, cur_ret);

    /* Synchronize D3D GPU semaphore with device state */
    uint32_t dev = guest_mem32_rd(CLX_D3D_DEVICE_PTR);
    if (dev) {
        uint32_t sem = guest_mem32_rd(dev + 0x30);
        if (sem) {
            guest_mem32_wr(sem, guest_mem32_rd(dev + 0x2C));
            nv2a_pb_set_semaphore_target(sem);
        }
    }

    uint8_t live_irql = xbox_CurrentIrql();
    xbox_RestoreCurrentIrql(saved_irql);
    fprintf(stderr, "[savestate] restored thread IRQL %u -> %u (global holds %d)\n",
            live_irql, saved_irql, xbox_IrqlRaisedCount());
    if (apu_restore_plan)
        mcpx_apu_savestate_commit(apu_restore_plan);
    g_audio_muted = 0;
    mcpx_apu_resume_after_savestate(1, audio_restored);
    apu_paused = 0;
    xbox_Nv2aResumePushbuffer();
    pb_paused = 0;
    ok = 1;
done:
    if (apu_paused)
        mcpx_apu_resume_after_savestate(0, 0);
    if (pb_paused)
        xbox_Nv2aResumePushbuffer();
    if (f)
        fclose(f);
    free(blocks_buf);
    free(ram_copy);
    free(contig_copy);
    free(audio_copy);
    free(gpu_copy);
    mcpx_apu_savestate_discard(apu_restore_plan);
    xbox_gil_leave();
    return ok;
}

void clx_savestate_tick(void)
{
    int action = atomic_load(&s_pending_action);
    int slot = atomic_load(&s_pending_slot);
    if (action == STATE_ACTION_NONE) return;

    atomic_store(&s_pending_action, STATE_ACTION_NONE);

    if (action == STATE_ACTION_SAVE) {
        int ok = clx_savestate_do_save(slot);
        char msg[64];
        clx_menu_close();
        if (ok) {
            snprintf(msg, sizeof(msg), "STATE %d SAVED", slot);
        } else {
            snprintf(msg, sizeof(msg), "STATE %d SAVE FAILED", slot);
        }
        clx_prompt(msg, 2000);
        fprintf(stderr, "[savestate] slot %d save %s\n", slot, ok ? "succeeded" : "failed");
    } else if (action == STATE_ACTION_LOAD) {
        if (!clx_savestate_exists(slot)) {
            char msg[64];
            snprintf(msg, sizeof(msg), "STATE %d NOT FOUND", slot);
            clx_menu_close();
            clx_prompt(msg, 2000);
            fprintf(stderr, "[savestate] slot %d not found\n", slot);
            return;
        }
        int ok = clx_savestate_do_load(slot);
        char msg[64];
        if (ok) {
            snprintf(msg, sizeof(msg), "STATE %d LOADED", slot);
            clx_menu_close();
        } else {
            snprintf(msg, sizeof(msg), "STATE %d LOAD FAILED / CORRUPT", slot);
            clx_menu_close();
        }
        clx_prompt(msg, 2000);
        fprintf(stderr, "[savestate] slot %d load %s\n", slot, ok ? "succeeded" : "failed");
    }
}
