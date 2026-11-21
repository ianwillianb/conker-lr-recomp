#include "clx_savestate.h"
#include "kernel/xbox_memory_layout.h"
#include "kernel/nv2a_backend.h"
#include "apu/apu.h"
#include "xbox/recomp_regs.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

static volatile uint32_t dma_registers[2];
static int gil_held;
static int guest_running;
static atomic_int audio_paused;
static atomic_int audio_clock_stop;
static atomic_uint_fast64_t audio_frames;
static int pushbuffer_paused;
static int irql_holds = 1;
static int failures;
static int guest_yields;
static int audio_pause_calls;
static int audio_resume_calls;
static uint64_t audio_frames_at_snapshot;
static uint64_t audio_frames_at_resume;
static uint8_t *guest_ram;
static int snapshot_taken;

size_t g_xbox_total_ram;
volatile int g_audio_muted;
RECOMP_TLS RecompRegs _rr[1];

#define CHECK(condition, message) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL: %s\n", message); \
        failures++; \
    } \
} while (0)

static void sleep_ms(unsigned ms)
{
    struct timespec delay = { (time_t)(ms / 1000), (long)(ms % 1000) * 1000000L };
    nanosleep(&delay, NULL);
}

static void *audio_clock(void *unused)
{
    (void)unused;
    while (!atomic_load(&audio_clock_stop)) {
        if (!atomic_load(&audio_paused))
            atomic_fetch_add(&audio_frames, 1);
        sleep_ms(1);
    }
    return NULL;
}

void xbox_gil_enter(void)
{
    gil_held = 1;
    guest_running = 0;
}

void xbox_gil_leave(void)
{
    gil_held = 0;
    guest_running = 0;
}

int xbox_gil_suspend(void)
{
    int depth = gil_held;
    gil_held = 0;
    guest_running = 1;
    guest_yields++;
    CHECK(!atomic_load(&audio_paused),
          "audio was paused while the guest lock was released");

    if (dma_registers[0] != dma_registers[1]) {
        sleep_ms(30);
        dma_registers[1] = dma_registers[0];
    } else if (irql_holds) {
        sleep_ms(10);
        irql_holds = 0;
    }
    return depth;
}

void xbox_gil_resume(int depth)
{
    if (depth) {
        gil_held = 1;
        guest_running = 0;
    }
}

int mcpx_apu_pause_for_savestate(unsigned timeout_ms)
{
    (void)timeout_ms;
    CHECK(gil_held && !guest_running,
          "APU pause must begin only after guest execution is parked");
    CHECK(dma_registers[0] == dma_registers[1],
          "APU pause must follow pushbuffer drain");
    atomic_store(&audio_paused, 1);
    audio_pause_calls++;
    return 1;
}

void mcpx_apu_resume_after_savestate(int loaded, int audio_restored)
{
    (void)loaded;
    (void)audio_restored;
    if (snapshot_taken)
        audio_frames_at_resume = atomic_load(&audio_frames);
    atomic_store(&audio_paused, 0);
    audio_resume_calls++;
}

size_t mcpx_apu_savestate_size(void)
{
    return 1;
}

int mcpx_apu_savestate_save(void *buffer, size_t buffer_size)
{
    CHECK(atomic_load(&audio_paused),
          "APU snapshot must be captured with host playback paused");
    if (buffer_size != 1)
        return 0;
    *(uint8_t *)buffer = 0xA5;
    audio_frames_at_snapshot = atomic_load(&audio_frames);
    snapshot_taken = 1;
    return 1;
}

int xbox_Nv2aPausePushbuffer(unsigned timeout_ms)
{
    (void)timeout_ms;
    CHECK(gil_held && atomic_load(&audio_paused) && !guest_running,
          "audio and guest must both be parked while pausing the pushbuffer");
    sleep_ms(30);
    CHECK(gil_held && atomic_load(&audio_paused) && !guest_running,
          "audio and guest must stay parked during pushbuffer pause");
    pushbuffer_paused = 1;
    return 1;
}

void xbox_Nv2aResumePushbuffer(void)
{
    pushbuffer_paused = 0;
}

ptrdiff_t xbox_GetMemoryOffset(void)
{
    return (ptrdiff_t)((intptr_t)(uintptr_t)dma_registers -
                       (intptr_t)UINT32_C(0xFD800040));
}

void *xbox_GetMemoryBase(void)
{
    return guest_ram;
}

void *xbox_GetContigMemoryBase(void)
{
    return NULL;
}

size_t xbox_GetContigMemoryUsed(void)
{
    return 0;
}

size_t xbox_MemLayoutSave(XboxMemLayoutSnapshot *snapshot,
                          void *blocks_buf, size_t max_buf_size)
{
    (void)snapshot;
    (void)blocks_buf;
    (void)max_buf_size;
    return 0;
}

uint8_t xbox_CurrentIrql(void)
{
    return 0;
}

void xbox_RestoreCurrentIrql(uint8_t irql)
{
    (void)irql;
}

int xbox_IrqlRaisedCount(void)
{
    return irql_holds;
}

void xbox_IrqlDumpHolders(void)
{
}

size_t nv2a_pb_exec_savestate_size(void)
{
    return 1;
}

int nv2a_pb_exec_savestate_save(void *dst, size_t size)
{
    CHECK(atomic_load(&audio_paused),
          "audio must remain paused through GPU-state capture");
    if (size != 1)
        return 0;
    *(uint8_t *)dst = 0x5A;
    sleep_ms(150);
    return 1;
}

int main(void)
{
    const size_t ram_size = 8u * 1024u * 1024u;
    pthread_t clock_thread;
    uint32_t random_state = 0xC0FFEEu;

    setenv("CLX_SAVESTATE_DIR", "states", 1);
    guest_ram = (uint8_t *)malloc(ram_size);
    if (!guest_ram) {
        fprintf(stderr, "could not allocate mock guest RAM\n");
        return 2;
    }
    g_xbox_total_ram = ram_size;
    for (size_t i = 0; i < ram_size; i++) {
        random_state = random_state * 1664525u + 1013904223u;
        guest_ram[i] = (uint8_t)(random_state >> 24);
    }
    dma_registers[0] = 1;
    dma_registers[1] = 0;

    if (pthread_create(&clock_thread, NULL, audio_clock, NULL) != 0) {
        fprintf(stderr, "could not start mock audio clock\n");
        free(guest_ram);
        return 2;
    }

    CHECK(clx_savestate_do_save(1) == 1,
          "save should complete with the mocked guest and devices");
    CHECK(guest_yields >= 2,
          "test should exercise both GPU-drain and IRQL guest yields");
    CHECK(audio_pause_calls == 2,
          "audio should pause before pushbuffer parking and again before snapshot");
    CHECK(audio_resume_calls == 2,
          "audio should resume across the IRQL yield and after save");
    CHECK(audio_frames_at_snapshot != 0 &&
          audio_frames_at_resume == audio_frames_at_snapshot,
          "audio timeline advanced during the save snapshot and file write");
    CHECK(!atomic_load(&audio_paused) && !pushbuffer_paused &&
          !gil_held && !guest_running,
          "save completion should resume audio, pushbuffer, and guest execution");

    atomic_store(&audio_clock_stop, 1);
    pthread_join(clock_thread, NULL);
    remove("states/conker_slot1.state");
    rmdir("states");
    free(guest_ram);

    if (failures) {
        fprintf(stderr, "Save/audio synchronization: %d failures\n", failures);
        return 1;
    }
    puts("Save/audio synchronization: passed");
    return 0;
}
