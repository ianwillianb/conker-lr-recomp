/*
 * MCPX APU Core - Standalone extraction from xemu
 *
 * Copyright (c) 2012 espes
 * Copyright (c) 2018-2019 Jannik Vogel
 * Copyright (c) 2019-2025 Matt Borgerson
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "apu_state.h"
#include "apu.h"
#include "apu_xaudio2.h"
#include "fpconv.h"

#include <math.h>

/* ============================================================
 * Globals
 * ============================================================ */

uint8_t *g_apu_ram_ptr = NULL;

MCPXAPUState *g_state = NULL;

/* Forward declarations for software mixer */
static void mixer_init(void);
static void mixer_render(int16_t frame_buf[][2], int num_samples);
static APUMixerVoice g_mixer_voices[APU_MIXER_MAX_VOICES];
static volatile int g_mixer_active_count = 0;
static CRITICAL_SECTION g_mixer_cs;
static bool g_mixer_initialized = false;
struct McpxApuDebug g_dbg;
struct McpxApuDebug g_dbg_cache;
int g_dbg_voice_monitor = -1;

#define MCPX_APU_SAVESTATE_MAGIC 0x31555041u
#define MCPX_APU_SAVESTATE_VERSION 2u
#define MCPX_APU_SAVESTATE_MAX_PCM_BYTES (128u * 1024u * 1024u)

typedef struct MCPXAPUMixerVoiceSnapshot {
    uint32_t pcm_bytes;
    uint32_t num_channels;
    uint32_t sample_rate;
    int32_t active;
    int32_t looping;
    uint64_t play_offset;
    float volume;
} MCPXAPUMixerVoiceSnapshot;

typedef struct MCPXAPUSavestate {
    uint32_t magic;
    uint32_t version;
    uint32_t total_size;
    uint32_t pcm_bytes;
    uint32_t output_queue_bytes;
    uint32_t regs[0x20000];
    uint32_t gp_regs[0x10000];
    uint32_t ep_regs[0x10000];
    uint8_t set_irq;
    uint8_t in_trap;
    uint8_t pause_requested;
    uint8_t test_tone_active;
    int32_t ep_frame_div;
    int32_t sleep_acc_us;
    int32_t frame_count;
    int32_t gp_realtime;
    int32_t ep_realtime;
    int32_t monitor_point;
    int32_t monitor_queued_bytes_low;
    int32_t monitor_queued_bytes_high;
    double test_tone_phase;
    double test_tone_phase_inc;
    int16_t test_tone_amplitude;
    uint16_t reserved;
    int16_t monitor_frame_buf[256][2];
    MCPXAPUMixerVoiceSnapshot mixer_voices[APU_MIXER_MAX_VOICES];
    MCPXAPUVPSnapshot vp;
    ApuLimiter dsp_limiter;
    ApuLimiter output_limiter;
} MCPXAPUSavestate;

typedef struct MCPXAPUSavestateRestorePlan {
    MCPXAPUSavestate snapshot;
    int16_t *pcm_data[APU_MIXER_MAX_VOICES];
    uint8_t *output_queue;
} MCPXAPUSavestateRestorePlan;

static int apu_limiter_state_valid(const ApuLimiter *limiter)
{
    if (!isfinite(limiter->gain) || !isfinite(limiter->target) ||
        limiter->gain < 0.0f || limiter->gain > 1.0f ||
        limiter->target < 0.0f || limiter->target > 1.0f ||
        limiter->pos < 0 || limiter->pos >= APU_LIMITER_LOOKAHEAD ||
        limiter->hold < 0 || limiter->hold > APU_LIMITER_LOOKAHEAD ||
        (limiter->init != 0 && limiter->init != 1))
        return 0;
    for (int i = 0; i < APU_LIMITER_LOOKAHEAD; i++)
        for (int ch = 0; ch < 2; ch++)
            if (!isfinite(limiter->delay[i][ch]))
                return 0;
    return 1;
}

/* APU frames run in the last second (1500 is real time: 48 kHz / 32) and the
 * share of that second the frame thread spent working, for hosts with no
 * other profiler. */
int mcpx_apu_frames_per_second(void) { return g_dbg.frames_processed; }
float mcpx_apu_utilization(void) { return g_dbg.utilization; }
uint64_t g_dbg_muted_voices[4] = { 0 };

void mcpx_apu_dsp_get_limiter_state(ApuLimiter *out);
void mcpx_apu_dsp_set_limiter_state(const ApuLimiter *state);

/* Global audio mute — disables all AWD/mixer sound playback */
volatile int g_audio_muted = 0;  /* 0 = audio enabled */

/* ============================================================
 * Debug frame markers (minimal stubs)
 * ============================================================ */

void mcpx_debug_begin_frame(void) {}
void mcpx_debug_end_frame(void) {}

/* ============================================================
 * IRQ handling: the APU's line goes to the kernel, which calls the
 * title's connected ISR (DirectSound's) on vector 5.
 * ============================================================ */

/* Every host: the line used to be raised on Windows only, and elsewhere the
 * no-op stub in qemu_shim.h left DirectSound's ISR uncalled, so no voice
 * ever started and the output stayed silent. */
extern void xbox_set_irq_line(uint32_t vector, int level);
#undef pci_irq_assert
#undef pci_irq_deassert
#define pci_irq_assert(dev)   xbox_set_irq_line(5, 1)
#define pci_irq_deassert(dev) xbox_set_irq_line(5, 0)

static void update_irq(MCPXAPUState *d)
{
    if (d->regs[NV_PAPU_FECTL] & NV_PAPU_FECTL_FEMETHMODE_TRAPPED) {
        qatomic_or(&d->regs[NV_PAPU_ISTS], NV_PAPU_ISTS_FETINTSTS);
    }
    if ((d->regs[NV_PAPU_IEN] & NV_PAPU_ISTS_GINTSTS) &&
        ((d->regs[NV_PAPU_ISTS] & ~NV_PAPU_ISTS_GINTSTS) &
         d->regs[NV_PAPU_IEN])) {
        qatomic_or(&d->regs[NV_PAPU_ISTS], NV_PAPU_ISTS_GINTSTS);
        /* In standalone mode we don't raise a PCI IRQ; the game's kernel
         * stub will poll ISTS directly or we'll signal via a flag. */
        pci_irq_assert(PCI_DEVICE(d));
    } else {
        qatomic_and(&d->regs[NV_PAPU_ISTS], ~NV_PAPU_ISTS_GINTSTS);
        pci_irq_deassert(PCI_DEVICE(d));
    }
}

/* ============================================================
 * MMIO Read / Write
 * ============================================================ */

uint64_t mcpx_apu_read(void *opaque, hwaddr addr, unsigned int size)
{
    MCPXAPUState *d = (MCPXAPUState *)opaque;
    uint64_t r = 0;

    switch (addr) {
    case NV_PAPU_XGSCNT:
        r = (uint64_t)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 100);
        break;
    default:
        if (addr < 0x20000) {
            r = qatomic_read(&d->regs[addr]);
        }
        break;
    }

    /* Uncomment for register tracing:
     * fprintf(stderr, "[APU] read  [0x%05llX] size=%u -> 0x%08llX\n",
     *         (unsigned long long)addr, size, (unsigned long long)r);
     */
    (void)size;
    return r;
}

void mcpx_apu_write(void *opaque, hwaddr addr, uint64_t val,
                     unsigned int size)
{
    MCPXAPUState *d = (MCPXAPUState *)opaque;

    /* Uncomment for register tracing:
     * fprintf(stderr, "[APU] write [0x%05llX] size=%u <- 0x%08llX\n",
     *         (unsigned long long)addr, size, (unsigned long long)val);
     */
    (void)size;

    switch (addr) {
    case NV_PAPU_ISTS:
        qatomic_and(&d->regs[NV_PAPU_ISTS], ~(uint32_t)val);
        update_irq(d);
        qemu_cond_broadcast(&d->cond);
        break;
    case NV_PAPU_FECTL:
    case NV_PAPU_SECTL:
        qatomic_set(&d->regs[addr], (uint32_t)val);
        /* Starting the APU has to start the frame thread.
         *
         * The thread idles on pause_requested, which init sets and only the
         * test tone ever cleared -- so a title that enabled the APU through
         * these registers got an APU that stayed asleep. Nothing then advanced
         * the front end, and a title waiting on a notify completion (the
         * FEMEMDATA magic write, which is how completion reaches guest memory)
         * waited forever. Wreckless hangs exactly there during DirectSound
         * init, and because it initialises its whole engine behind a
         * successful DirectSound create, that hang is not confined to audio.
         *
         * Resume whenever the write is not switching the block off; the thread
         * re-checks FECTL itself and idles again if it is halted or trapped. */
        {
            uint32_t sectl = qatomic_read(&d->regs[NV_PAPU_SECTL]);
            uint32_t fectl = qatomic_read(&d->regs[NV_PAPU_FECTL]);
            bool running =
                ((sectl & NV_PAPU_SECTL_XCNTMODE) != NV_PAPU_SECTL_XCNTMODE_OFF)
                && ((fectl & NV_PAPU_FECTL_FEMETHMODE)
                    != NV_PAPU_FECTL_FEMETHMODE_HALTED);
            if (running && d->pause_requested) {
                d->pause_requested = false;
                fprintf(stderr, "[APU] started by the title"
                                " (SECTL=%08X FECTL=%08X)\n", sectl, fectl);
            }
        }
        qemu_cond_broadcast(&d->cond);
        break;
    case NV_PAPU_FEMEMDATA:
        /* 'magic write' - value written to FEMEMADDR on notify completion */
        stl_le_phys(address_space_memory, d->regs[NV_PAPU_FEMEMADDR], (uint32_t)val);
        qatomic_set(&d->regs[addr], (uint32_t)val);
        break;
    default:
        if (addr < 0x20000) {
            qatomic_set(&d->regs[addr], (uint32_t)val);
        }
        break;
    }
}

/* ============================================================
 * Test tone state (used by monitor and test tone functions)
 * ============================================================ */

static struct {
    bool active;
    double phase;
    double phase_inc;
    int16_t amplitude;
} g_test_tone = { false, 0.0, 0.0, 0 };

/* ============================================================
 * Monitor - Audio output (XAudio2 primary, waveOut fallback)
 * ============================================================ */

#if defined(_WIN32)
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#endif
/* On Linux, waveOut* are inert stubs from win32_compat.h: the APU's
 * waveOut fallback path stays inactive and never produces audio. */

/* Ring of waveOut buffers for double-buffering */
#define WAVEOUT_NUM_BUFS 4
#define WAVEOUT_BUF_SAMPLES 2048  /* ~42.7ms at 48kHz, matches 8-frame delivery rate */
#define MIXER_FRAME_SAMPLES 256  /* Internal mixing frame size (matches frame_buf) */

typedef struct {
    HWAVEOUT hwo;
    WAVEHDR  hdrs[WAVEOUT_NUM_BUFS];
    int16_t  bufs[WAVEOUT_NUM_BUFS][WAVEOUT_BUF_SAMPLES][2];
    int      next_buf;
    bool     initialized;
    int      frames_written;
} WaveOutState;

static WaveOutState g_waveout = { 0 };

void mcpx_apu_monitor_init(MCPXAPUState *d, Error **errp)
{
    (void)errp;
    d->monitor.stream = NULL;
    d->monitor.queued_bytes_low = 1024;
    d->monitor.queued_bytes_high = 3072;

    /* Try XAudio2 first (lower latency) */
    if (xa2_init()) {
        fprintf(stderr, "[APU] host audio output active\n");
        return;
    }
    fprintf(stderr, "[APU] XAudio2 unavailable, falling back to waveOut\n");

    WAVEFORMATEX wfx = { 0 };
    wfx.wFormatTag      = WAVE_FORMAT_PCM;
    wfx.nChannels       = 2;
    wfx.nSamplesPerSec  = 48000;
    wfx.wBitsPerSample  = 16;
    wfx.nBlockAlign     = wfx.nChannels * wfx.wBitsPerSample / 8;
    wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;

    MMRESULT mr = waveOutOpen(&g_waveout.hwo, WAVE_MAPPER, &wfx,
                               0, 0, CALLBACK_NULL);
    if (mr != MMSYSERR_NOERROR) {
        fprintf(stderr, "[APU] waveOutOpen failed (error %u)\n", mr);
        g_waveout.initialized = false;
        return;
    }

    /* Prepare all headers */
    for (int i = 0; i < WAVEOUT_NUM_BUFS; i++) {
        memset(&g_waveout.hdrs[i], 0, sizeof(WAVEHDR));
        g_waveout.hdrs[i].lpData = (LPSTR)g_waveout.bufs[i];
        g_waveout.hdrs[i].dwBufferLength = WAVEOUT_BUF_SAMPLES * 2 * sizeof(int16_t);
        waveOutPrepareHeader(g_waveout.hwo, &g_waveout.hdrs[i], sizeof(WAVEHDR));
    }

    g_waveout.next_buf = 0;
    g_waveout.initialized = true;
    g_waveout.frames_written = 0;

    fprintf(stderr, "[APU] waveOut audio output initialized (48kHz stereo 16-bit, %d buffers)\n",
            WAVEOUT_NUM_BUFS);
}

void mcpx_apu_monitor_finalize(MCPXAPUState *d)
{
    (void)d;
    if (xa2_is_active()) {
        xa2_shutdown();
        return;
    }
    if (!g_waveout.initialized) return;

    waveOutReset(g_waveout.hwo);
    for (int i = 0; i < WAVEOUT_NUM_BUFS; i++) {
        waveOutUnprepareHeader(g_waveout.hwo, &g_waveout.hdrs[i], sizeof(WAVEHDR));
    }
    waveOutClose(g_waveout.hwo);
    g_waveout.initialized = false;
    fprintf(stderr, "[APU] waveOut audio output shut down (%d frames written)\n",
            g_waveout.frames_written);
}

void mcpx_apu_monitor_frame(MCPXAPUState *d)
{
    if ((d->ep_frame_div + 1) % 8) {
        return;
    }

    /* XAudio2 path: one block per 8 EP frames.
     *
     * The DSP stage has just written this block's 256 samples (8 frames x 32)
     * into frame_buf -- the guest's own audio. This used to clear frame_buf
     * first and then submit 1024 samples per call: every sample the game
     * produced was erased (the output was silent, peak -99 dB, in menus,
     * movies and levels alike), and four times real time was pushed, so the
     * queue dropped ~140 blocks a second. Now the block is kept, the software
     * mixer and test tone are added on top, exactly 256 samples go out, and
     * the buffer is cleared afterwards for the next block. */
    if (xa2_is_active()) {
        int n = MIXER_FRAME_SAMPLES;

        if (g_test_tone.active && !g_audio_muted) {
            for (int i = 0; i < n; i++) {
                int16_t s = (int16_t)(sin(g_test_tone.phase) * g_test_tone.amplitude);
                d->monitor.frame_buf[i][0] = s;
                d->monitor.frame_buf[i][1] = s;
                g_test_tone.phase += g_test_tone.phase_inc;
                if (g_test_tone.phase >= 2.0 * M_PI)
                    g_test_tone.phase -= 2.0 * M_PI;
            }
        }
        if (g_audio_muted)
            memset(d->monitor.frame_buf, 0, sizeof(d->monitor.frame_buf));
        else
            mixer_render(d->monitor.frame_buf, n);

        xa2_submit_samples((const int16_t *)d->monitor.frame_buf, n);
        memset(d->monitor.frame_buf, 0, sizeof(d->monitor.frame_buf));
        return;
    }

    if (!g_waveout.initialized) return;

    int idx = g_waveout.next_buf;
    WAVEHDR *hdr = &g_waveout.hdrs[idx];

    /* Wait if this buffer is still playing (with timeout) */
    int wait_loops = 0;
    while (!(hdr->dwFlags & WHDR_DONE) && (hdr->dwFlags & WHDR_INQUEUE)) {
        qemu_mutex_unlock(&d->lock);
        Sleep(1);
        qemu_mutex_lock(&d->lock);
        if (d->savestate_pause_requested)
            return;
        if (++wait_loops > 50) break;
    }

    /* Fill the large waveOut buffer by rendering multiple 256-sample frames */
    int16_t *out = (int16_t *)g_waveout.bufs[idx];
    int remaining = WAVEOUT_BUF_SAMPLES;
    int out_offset = 0;

    while (remaining > 0) {
        int chunk = (remaining < MIXER_FRAME_SAMPLES) ? remaining : MIXER_FRAME_SAMPLES;

        memset(d->monitor.frame_buf, 0, sizeof(d->monitor.frame_buf));

        /* Test tone (skip if muted) */
        if (g_test_tone.active && !g_audio_muted) {
            for (int i = 0; i < chunk; i++) {
                int16_t s = (int16_t)(sin(g_test_tone.phase) * g_test_tone.amplitude);
                d->monitor.frame_buf[i][0] = s;
                d->monitor.frame_buf[i][1] = s;
                g_test_tone.phase += g_test_tone.phase_inc;
                if (g_test_tone.phase >= 2.0 * M_PI)
                    g_test_tone.phase -= 2.0 * M_PI;
            }
        }

        /* Mix software voices (skip if muted) */
        if (!g_audio_muted)
            mixer_render(d->monitor.frame_buf, chunk);

        /* Copy to waveOut buffer */
        memcpy(out + out_offset * 2, d->monitor.frame_buf, chunk * 2 * sizeof(int16_t));
        out_offset += chunk;
        remaining -= chunk;
    }

    /* Volume + limiter, as on the XAudio2 path. */
    apu_output_safety(out, out, WAVEOUT_BUF_SAMPLES * 2);

    /* Submit to waveOut */
    hdr->dwFlags &= ~WHDR_DONE;
    waveOutWrite(g_waveout.hwo, hdr, sizeof(WAVEHDR));

    g_waveout.next_buf = (idx + 1) % WAVEOUT_NUM_BUFS;
    g_waveout.frames_written++;
}

/* ============================================================
 * Throttle (timing control for frame pacing)
 * ============================================================ */

/* Pacing statistics for the [perf] report (mcpx_apu_pacing_stats): timed
 * waits, requested vs slept, the SDL low-queue path, clock restarts after
 * EP_CATCHUP_US of lateness (time lost for good) and the worst lateness. */
static volatile int64_t s_thr_waits, s_thr_req_us, s_thr_slept_us, s_thr_lowq,
                        s_thr_resets, s_thr_lost_us, s_thr_late_max_us, s_thr_blocks,
                        s_thr_traps, s_thr_trap_us;

int mcpx_apu_pacing_stats(char *buf, int cap)
{
    int n = snprintf(buf, (size_t)cap,
                     "%lld blocks, %lld waits (asked %lld ms, slept %lld ms), %lld low-queue,"
                     " %lld clock restarts (%lld ms lost), worst lateness %lld ms,"
                     " %lld front-end traps (%lld ms)",
                     (long long)s_thr_blocks, (long long)s_thr_waits,
                     (long long)(s_thr_req_us / 1000), (long long)(s_thr_slept_us / 1000),
                     (long long)s_thr_lowq, (long long)s_thr_resets,
                     (long long)(s_thr_lost_us / 1000), (long long)(s_thr_late_max_us / 1000),
                     (long long)s_thr_traps, (long long)(s_thr_trap_us / 1000));
    s_thr_blocks = s_thr_waits = s_thr_req_us = s_thr_slept_us = s_thr_lowq = 0;
    s_thr_resets = s_thr_lost_us = s_thr_late_max_us = 0;
    s_thr_traps = s_thr_trap_us = 0;
    return n;
}

static void throttle(MCPXAPUState *d)
{
    if (d->ep_frame_div % 8) {
        return;
    }

    /* With XAudio2, pace by its queue: render the next 256-sample block once
     * fewer than 8 (43 ms) are waiting. 4 (21 ms) still ran dry: this thread
     * was measured going 20-44 ms between blocks while the game was busy. The sound card's clock drains the
     * queue, so output never drifts from it, and a stall is caught up with a
     * burst instead of being forgotten. The wall-clock pacing below reset
     * itself after any stall longer than one frame, so every stall was lost
     * for good and the queue ran dry: 4-10 underruns a second, heard as
     * crackle. */
#if defined(_WIN32)
    if (xa2_is_active()) {
        int64_t t0 = qemu_clock_get_us(QEMU_CLOCK_REALTIME);
        /* Let a guest register write in once per block even when behind and
         * not waiting: that writer can be holding the dispatch lock. */
        qemu_mutex_unlock(&d->lock);
        SwitchToThread();
        qemu_mutex_lock(&d->lock);
        while (!d->pause_requested && !d->savestate_pause_requested &&
               xa2_queued() >= xa2_queue_target())
            qemu_cond_timedwait(&d->cond, &d->lock, 1);
        d->sleep_acc_us += (int)(qemu_clock_get_us(QEMU_CLOCK_REALTIME) - t0);
        return;
    }
#endif

    int64_t now_us = qemu_clock_get_us(QEMU_CLOCK_REALTIME);
    int64_t wait_start_us = now_us;

    /* SDL output (Linux, Switch) never paces the APU: an audio server that
     * does not drain its queue (WSLg's PulseAudio) held every block for the
     * full wait, the APU clock crawled, and the title booted at a fraction
     * of its speed. The wall clock below stays the APU's clock; the device
     * is only fed. When its queue runs low (its clock is a little faster
     * than ours) one block is rendered without waiting and without moving
     * the deadline, so the APU follows the device instead of underrunning;
     * a full queue drops blocks in the backend. */
    s_thr_blocks++;
    if (d->next_frame_time_us && now_us - d->next_frame_time_us > s_thr_late_max_us)
        s_thr_late_max_us = now_us - d->next_frame_time_us;
    if (xa2_is_active() && xa2_queued() < xa2_queue_target() / 2) {
        s_thr_lowq++;
        qemu_mutex_unlock(&d->lock);
        SwitchToThread();
        qemu_mutex_lock(&d->lock);
        if (d->next_frame_time_us == 0 ||
            now_us - d->next_frame_time_us > EP_CATCHUP_US)
            d->next_frame_time_us = now_us;
        return;
    }

    /* Without a sound card to drain a queue, this deadline is the APU's
     * clock, and DirectSound's play cursor -- which paces the title's movies
     * -- follows it. Resetting it after any block that ran late threw the
     * lateness away for good. On Horizon, where busy cores time-slice in
     * 10 ms quanta, a 5 ms wait regularly overshoots, and movies played
     * visibly slow. A late block is now made up by running the next ones
     * back to back; only a stall long enough to be a pause (a debugger, a
     * suspended applet) restarts the clock. */
    if (d->next_frame_time_us == 0 ||
        now_us - d->next_frame_time_us > EP_CATCHUP_US) {
        if (d->next_frame_time_us) {
            s_thr_resets++;
            s_thr_lost_us += now_us - d->next_frame_time_us;
        }
        d->next_frame_time_us = now_us;
    }

    while (!d->pause_requested && !d->savestate_pause_requested) {
        now_us = qemu_clock_get_us(QEMU_CLOCK_REALTIME);
        int64_t remaining_ms = (d->next_frame_time_us - now_us) / 1000;
        if (remaining_ms > 0) {
            int64_t w0 = qemu_clock_get_us(QEMU_CLOCK_REALTIME);
            qemu_cond_timedwait(&d->cond, &d->lock, (int)remaining_ms);
            s_thr_waits++;
            s_thr_req_us += remaining_ms * 1000;
            s_thr_slept_us += qemu_clock_get_us(QEMU_CLOCK_REALTIME) - w0;
        } else {
            break;
        }
    }
    d->next_frame_time_us += EP_FRAME_US;

    d->sleep_acc_us += (int)(qemu_clock_get_us(QEMU_CLOCK_REALTIME) - wait_start_us);
}

/* ============================================================
 * se_frame - Process one audio frame (VP -> GP -> EP pipeline)
 * ============================================================ */

static void se_frame(MCPXAPUState *d)
{
    mcpx_apu_update_dsp_preference(d);
    mcpx_debug_begin_frame();
    g_dbg.gp_realtime = d->gp.realtime;
    g_dbg.ep_realtime = d->ep.realtime;

    int64_t now_ms = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    int64_t elapsed_ms = now_ms - d->frame_count_time_ms;
    if (elapsed_ms >= 1000) {
        g_dbg.utilization = 1.0f - d->sleep_acc_us / (elapsed_ms * 1000.0f);
        g_dbg.frames_processed = (int)(d->frame_count * 1000.0 / elapsed_ms + 0.5);
        d->frame_count_time_ms = now_ms;
        d->frame_count = 0;
        d->sleep_acc_us = 0;
    }
    d->frame_count++;

    /* Buffer for all mixbins for this frame */
    float mixbins[NUM_MIXBINS][NUM_SAMPLES_PER_FRAME];
    memset(mixbins, 0, sizeof(mixbins));

    mcpx_apu_vp_frame(d, mixbins);
    mcpx_apu_dsp_frame(d, mixbins);
    mcpx_apu_monitor_frame(d);

    d->ep_frame_div++;

    /* The front end trapped this frame (an idle voice the driver asked to
     * hear about): raise it now, or the trap halts the pipeline unseen. */
    if (d->set_irq) {
        d->set_irq = false;
        update_irq(d);
    }

    mcpx_debug_end_frame();
}

/* ============================================================
 * APU frame thread (background processing)
 * ============================================================ */

static void *mcpx_apu_frame_thread(void *arg)
{
    MCPXAPUState *d = MCPX_APU_DEVICE(arg);
    /* Audio misses are audible, a late game frame is not: stay ahead of the
     * game's busy threads for the few microseconds a block takes. */
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    {
        extern void xbox_nx_raise_host_thread(void);   /* win32_compat.c */
        xbox_nx_raise_host_thread();                    /* the above, on Horizon */
    }
    qemu_mutex_lock(&d->lock);

    while (!qatomic_read(&d->exiting)) {
        if (d->savestate_pause_requested) {
            d->is_idle = true;
            qemu_cond_signal(&d->idle_cond);
            while (d->savestate_pause_requested &&
                   !qatomic_read(&d->exiting))
                qemu_cond_wait(&d->cond, &d->lock);
            d->is_idle = false;
            continue;
        }

        if (d->pause_requested && !g_test_tone.active && !g_mixer_active_count) {
            d->is_idle = true;
            qemu_cond_signal(&d->idle_cond);
            qemu_cond_wait(&d->cond, &d->lock);
            d->is_idle = false;
            continue;
        }

        int xcntmode = GET_MASK(qatomic_read(&d->regs[NV_PAPU_SECTL]),
                                NV_PAPU_SECTL_XCNTMODE);
        uint32_t fectl = qatomic_read(&d->regs[NV_PAPU_FECTL]);

        /* Front end trapped (an idle voice the driver asked to hear about)
         * or halted: the pipeline stops until the driver's interrupt handler
         * clears it, and so does time, as in xemu. This used to run silent
         * monitor frames and let the clock run on: in races the handler is
         * late (the guest is busy), ~17% of frames went out as silence --
         * the audio stutter -- and the APU fell to 1250-1300 frames/s. Now
         * no frame is lost: throttle() catches up afterwards (up to
         * EP_CATCHUP_US). */
        if (xcntmode != NV_PAPU_SECTL_XCNTMODE_OFF && !g_test_tone.active &&
            (fectl & (NV_PAPU_FECTL_FEMETHMODE_TRAPPED | NV_PAPU_FECTL_FEMETHMODE_HALTED))) {
            int64_t t0 = qemu_clock_get_us(QEMU_CLOCK_REALTIME);
            if (!d->in_trap) {
                d->in_trap = true;
                s_thr_traps++;
            }
            qemu_cond_timedwait(&d->cond, &d->lock, 1);
            s_thr_trap_us += qemu_clock_get_us(QEMU_CLOCK_REALTIME) - t0;
            continue;
        }
        d->in_trap = false;

        /* Always run the audio output loop — the software mixer and test tone
         * need continuous frame delivery regardless of APU register state.
         * The VP/DSP pipeline (se_frame) only runs when registers allow it. */
        throttle(d);
        if (d->savestate_pause_requested)
            continue;

        bool apu_active = (xcntmode != NV_PAPU_SECTL_XCNTMODE_OFF) &&
                          !(fectl & NV_PAPU_FECTL_FEMETHMODE_TRAPPED) &&
                          !(fectl & NV_PAPU_FECTL_FEMETHMODE_HALTED);

        if (apu_active && !g_test_tone.active) {
            /* Full pipeline: VP voices → DSP → monitor → waveOut */
            se_frame(d);
        } else {
            /* Lightweight: just monitor frame (test tone + software mixer) */
            mcpx_apu_monitor_frame(d);
            if (d->savestate_pause_requested)
                continue;
            d->ep_frame_div++;
        }

        /* Hand the lock to a waiting guest thread (mcpx_apu_lock_guest). */
        while (d->lock_waiters && !qatomic_read(&d->exiting))
            qemu_cond_timedwait(&d->cond, &d->lock, 1);
    }

    qemu_mutex_unlock(&d->lock);
    return NULL;
}

/* ============================================================
 * Wait for idle / resume helpers
 * ============================================================ */

static void mcpx_apu_wait_for_idle(MCPXAPUState *d)
{
    d->pause_requested = true;
    qemu_cond_signal(&d->cond);
    while (!d->is_idle) {
        qemu_cond_wait(&d->idle_cond, &d->lock);
    }
}

static void mcpx_apu_resume(MCPXAPUState *d)
{
    d->pause_requested = false;
    qemu_cond_signal(&d->cond);
}

static int apu_pause_output_device(void)
{
    if (xa2_is_active())
        return xa2_pause_output();
    if (g_waveout.initialized) {
#if defined(_WIN32)
        MMRESULT mr = waveOutPause(g_waveout.hwo);
        if (mr != MMSYSERR_NOERROR) {
            fprintf(stderr, "[APU] waveOutPause failed for save state (error %u)\n", mr);
            return 0;
        }
#endif
    }
    return 1;
}

static void apu_resume_output_device(void)
{
    if (xa2_is_active()) {
        if (!xa2_resume_output())
            fprintf(stderr, "[APU] host audio output could not be resumed after save state\n");
    } else if (g_waveout.initialized) {
#if defined(_WIN32)
        MMRESULT mr = waveOutRestart(g_waveout.hwo);
        if (mr != MMSYSERR_NOERROR)
            fprintf(stderr, "[APU] waveOutRestart failed after save state (error %u)\n", mr);
#endif
    }
}

int mcpx_apu_pause_for_savestate(unsigned timeout_ms)
{
    MCPXAPUState *d = g_state;
    int64_t start_ms;

    if (!d)
        return 1;

    /* Stop playback before waiting for the worker so the audio clock cannot
     * run ahead while the guest is already stopped for the snapshot. */
    qemu_mutex_lock(&d->lock);
    if (!apu_pause_output_device()) {
        qemu_mutex_unlock(&d->lock);
        return 0;
    }
    d->savestate_pause_requested = true;
    qemu_cond_broadcast(&d->cond);
    start_ms = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    while (!d->is_idle && !qatomic_read(&d->exiting)) {
        if (qemu_clock_get_ms(QEMU_CLOCK_REALTIME) - start_ms >= (int64_t)timeout_ms) {
            d->savestate_pause_requested = false;
            qemu_cond_broadcast(&d->cond);
            apu_resume_output_device();
            qemu_mutex_unlock(&d->lock);
            fprintf(stderr, "[APU] timed out pausing for save state\n");
            return 0;
        }
        qemu_cond_timedwait(&d->idle_cond, &d->lock, 1);
    }
    int paused = !qatomic_read(&d->exiting);
    if (!paused) {
        d->savestate_pause_requested = false;
        qemu_cond_broadcast(&d->cond);
        apu_resume_output_device();
    }
    qemu_mutex_unlock(&d->lock);
    return paused;
}

size_t mcpx_apu_savestate_size(void)
{
    size_t size = sizeof(MCPXAPUSavestate);
    size_t output_queue_size;
    if (!g_mixer_initialized)
        return 0;

    EnterCriticalSection(&g_mixer_cs);
    for (int i = 0; i < APU_MIXER_MAX_VOICES; i++) {
        APUMixerVoice *voice = &g_mixer_voices[i];
        if ((voice->pcm_bytes && !voice->pcm_data) ||
            voice->pcm_bytes > MCPX_APU_SAVESTATE_MAX_PCM_BYTES ||
            size > SIZE_MAX - voice->pcm_bytes) {
            size = 0;
            break;
        }
        size += voice->pcm_bytes;
    }
    LeaveCriticalSection(&g_mixer_cs);

    output_queue_size = xa2_queue_snapshot_size();
    if (size == 0 || output_queue_size == 0 ||
        size > SIZE_MAX - output_queue_size)
        return 0;
    size += output_queue_size;
    if (size == 0 || size > UINT32_MAX ||
        size - sizeof(MCPXAPUSavestate) > MCPX_APU_SAVESTATE_MAX_PCM_BYTES)
        return 0;
    return size;
}

int mcpx_apu_savestate_save(void *buffer, size_t buffer_size)
{
    MCPXAPUState *d = g_state;
    MCPXAPUSavestate *snapshot = (MCPXAPUSavestate *)buffer;
    uint8_t *pcm_cursor;
    size_t output_queue_size = xa2_queue_snapshot_size();
    size_t required = mcpx_apu_savestate_size();
    int ok = 0;

    if (!d || !buffer || required == 0 || buffer_size != required ||
        output_queue_size == 0 || output_queue_size > buffer_size - sizeof(*snapshot))
        return 0;

    qemu_mutex_lock(&d->lock);
    EnterCriticalSection(&g_mixer_cs);
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->magic = MCPX_APU_SAVESTATE_MAGIC;
    snapshot->version = MCPX_APU_SAVESTATE_VERSION;
    snapshot->total_size = (uint32_t)buffer_size;
    snapshot->pcm_bytes = (uint32_t)(buffer_size - sizeof(*snapshot) - output_queue_size);
    snapshot->output_queue_bytes = (uint32_t)output_queue_size;
    memcpy(snapshot->regs, d->regs, sizeof(snapshot->regs));
    memcpy(snapshot->gp_regs, d->gp.regs, sizeof(snapshot->gp_regs));
    memcpy(snapshot->ep_regs, d->ep.regs, sizeof(snapshot->ep_regs));
    snapshot->set_irq = d->set_irq;
    snapshot->in_trap = d->in_trap;
    snapshot->pause_requested = d->pause_requested;
    snapshot->test_tone_active = g_test_tone.active;
    snapshot->ep_frame_div = d->ep_frame_div;
    snapshot->sleep_acc_us = d->sleep_acc_us;
    snapshot->frame_count = d->frame_count;
    snapshot->gp_realtime = d->gp.realtime;
    snapshot->ep_realtime = d->ep.realtime;
    snapshot->monitor_point = d->monitor.point;
    snapshot->monitor_queued_bytes_low = d->monitor.queued_bytes_low;
    snapshot->monitor_queued_bytes_high = d->monitor.queued_bytes_high;
    snapshot->test_tone_phase = g_test_tone.phase;
    snapshot->test_tone_phase_inc = g_test_tone.phase_inc;
    snapshot->test_tone_amplitude = g_test_tone.amplitude;
    memcpy(snapshot->monitor_frame_buf, d->monitor.frame_buf,
           sizeof(snapshot->monitor_frame_buf));
    if (!mcpx_apu_vp_save_state(d, &snapshot->vp))
        goto done;
    mcpx_apu_dsp_get_limiter_state(&snapshot->dsp_limiter);
    apu_output_get_state(&snapshot->output_limiter);

    pcm_cursor = (uint8_t *)buffer + sizeof(*snapshot);
    for (int i = 0; i < APU_MIXER_MAX_VOICES; i++) {
        APUMixerVoice *voice = &g_mixer_voices[i];
        MCPXAPUMixerVoiceSnapshot *saved = &snapshot->mixer_voices[i];
        saved->pcm_bytes = voice->pcm_bytes;
        saved->num_channels = voice->num_channels;
        saved->sample_rate = voice->sample_rate;
        saved->active = voice->active;
        saved->looping = voice->looping;
        saved->play_offset = voice->play_offset;
        saved->volume = voice->volume;
        if (voice->pcm_bytes) {
            memcpy(pcm_cursor, voice->pcm_data, voice->pcm_bytes);
            pcm_cursor += voice->pcm_bytes;
        }
    }
    if (!xa2_queue_snapshot_save(pcm_cursor, output_queue_size))
        goto done;
    ok = 1;
done:
    LeaveCriticalSection(&g_mixer_cs);
    qemu_mutex_unlock(&d->lock);
    if (!ok)
        fprintf(stderr, "[APU] could not capture a pointer-safe audio snapshot\n");
    return ok;
}

void *mcpx_apu_savestate_prepare(const void *buffer, size_t buffer_size)
{
    const MCPXAPUSavestate *snapshot = (const MCPXAPUSavestate *)buffer;
    const uint8_t *pcm_cursor;
    const uint8_t *queue_cursor;
    uint64_t pcm_bytes = 0;
    MCPXAPUSavestateRestorePlan *plan;

    if (!g_state || !g_mixer_initialized || !buffer ||
        buffer_size < sizeof(*snapshot) ||
        buffer_size > sizeof(*snapshot) + MCPX_APU_SAVESTATE_MAX_PCM_BYTES ||
        snapshot->magic != MCPX_APU_SAVESTATE_MAGIC ||
        snapshot->version != MCPX_APU_SAVESTATE_VERSION ||
        snapshot->total_size != buffer_size ||
        snapshot->pcm_bytes > buffer_size - sizeof(*snapshot) ||
        snapshot->output_queue_bytes !=
            buffer_size - sizeof(*snapshot) - snapshot->pcm_bytes ||
        snapshot->set_irq > 1 || snapshot->in_trap > 1 ||
        snapshot->pause_requested > 1 || snapshot->test_tone_active > 1 ||
        snapshot->ep_frame_div < 0 ||
        snapshot->monitor_point < MCPX_APU_DEBUG_MON_AC97 ||
        snapshot->monitor_point > MCPX_APU_DEBUG_MON_GP_OR_EP ||
        !isfinite(snapshot->test_tone_phase) ||
        !isfinite(snapshot->test_tone_phase_inc) ||
        !apu_limiter_state_valid(&snapshot->dsp_limiter) ||
        !apu_limiter_state_valid(&snapshot->output_limiter) ||
        !mcpx_apu_vp_validate_state(&snapshot->vp)) {
        fprintf(stderr, "[APU] invalid audio snapshot header or VP state\n");
        return NULL;
    }

    queue_cursor = (const uint8_t *)buffer + sizeof(*snapshot) + snapshot->pcm_bytes;
    if (!xa2_queue_snapshot_validate(queue_cursor, snapshot->output_queue_bytes)) {
        fprintf(stderr, "[APU] invalid host output queue in snapshot\n");
        return NULL;
    }

    for (int i = 0; i < APU_MIXER_MAX_VOICES; i++) {
        const MCPXAPUMixerVoiceSnapshot *voice = &snapshot->mixer_voices[i];
        uint32_t frame_bytes;
        if (voice->pcm_bytes == 0) {
            if (voice->active) {
                fprintf(stderr, "[APU] invalid active mixer voice %d in snapshot\n", i);
                return NULL;
            }
            continue;
        }
        if (voice->pcm_bytes > MCPX_APU_SAVESTATE_MAX_PCM_BYTES ||
            (voice->num_channels != 1 && voice->num_channels != 2) ||
            voice->sample_rate == 0 || voice->sample_rate > 192000 ||
            voice->active < 0 || voice->active > 1 ||
            voice->looping < 0 || voice->looping > 1 ||
            !isfinite(voice->volume)) {
            fprintf(stderr, "[APU] invalid mixer voice %d in snapshot\n", i);
            return NULL;
        }
        frame_bytes = voice->num_channels * sizeof(int16_t);
        if (voice->pcm_bytes < frame_bytes ||
            (voice->play_offset >> 16) >= voice->pcm_bytes / frame_bytes) {
            fprintf(stderr, "[APU] invalid mixer position for voice %d\n", i);
            return NULL;
        }
        pcm_bytes += voice->pcm_bytes;
    }
    if (pcm_bytes != snapshot->pcm_bytes) {
        fprintf(stderr, "[APU] mixer PCM size mismatch in snapshot\n");
        return NULL;
    }

    plan = (MCPXAPUSavestateRestorePlan *)calloc(1, sizeof(*plan));
    if (!plan)
        return NULL;
    memcpy(&plan->snapshot, snapshot, sizeof(*snapshot));
    pcm_cursor = (const uint8_t *)buffer + sizeof(*snapshot);
    for (int i = 0; i < APU_MIXER_MAX_VOICES; i++) {
        uint32_t bytes = snapshot->mixer_voices[i].pcm_bytes;
        if (!bytes)
            continue;
        plan->pcm_data[i] = (int16_t *)malloc(bytes);
        if (!plan->pcm_data[i]) {
            mcpx_apu_savestate_discard(plan);
            return NULL;
        }
        memcpy(plan->pcm_data[i], pcm_cursor, bytes);
        pcm_cursor += bytes;
    }
    plan->output_queue = (uint8_t *)malloc(snapshot->output_queue_bytes);
    if (!plan->output_queue) {
        mcpx_apu_savestate_discard(plan);
        return NULL;
    }
    memcpy(plan->output_queue, queue_cursor, snapshot->output_queue_bytes);
    return plan;
}

void mcpx_apu_savestate_commit(void *restore_plan)
{
    MCPXAPUSavestateRestorePlan *plan = (MCPXAPUSavestateRestorePlan *)restore_plan;
    MCPXAPUSavestate *snapshot;
    MCPXAPUState *d = g_state;
    int active_count = 0;

    if (!plan || !d)
        return;
    snapshot = &plan->snapshot;

    if (xa2_is_active() &&
        !xa2_queue_snapshot_restore(plan->output_queue, snapshot->output_queue_bytes)) {
        fprintf(stderr, "[APU] host output queue restore failed; clearing queued audio\n");
        if (!xa2_clear_queue())
            fprintf(stderr, "[APU] host audio queue could not be cleared after restore failure\n");
    }

    qemu_mutex_lock(&d->lock);
    EnterCriticalSection(&g_mixer_cs);
    memcpy(d->regs, snapshot->regs, sizeof(d->regs));
    memcpy(d->gp.regs, snapshot->gp_regs, sizeof(d->gp.regs));
    memcpy(d->ep.regs, snapshot->ep_regs, sizeof(d->ep.regs));
    mcpx_apu_vp_restore_state(d, &snapshot->vp);
    d->set_irq = snapshot->set_irq != 0;
    d->in_trap = snapshot->in_trap != 0;
    d->pause_requested = snapshot->pause_requested != 0;
    d->ep_frame_div = snapshot->ep_frame_div;
    d->sleep_acc_us = snapshot->sleep_acc_us;
    d->frame_count = snapshot->frame_count;
    d->gp.realtime = snapshot->gp_realtime != 0;
    d->ep.realtime = snapshot->ep_realtime != 0;
    d->monitor.point = (McpxApuDebugMonitorPoint)snapshot->monitor_point;
    d->monitor.queued_bytes_low = snapshot->monitor_queued_bytes_low;
    d->monitor.queued_bytes_high = snapshot->monitor_queued_bytes_high;
    memcpy(d->monitor.frame_buf, snapshot->monitor_frame_buf,
           sizeof(d->monitor.frame_buf));
    g_test_tone.active = snapshot->test_tone_active != 0;
    g_test_tone.phase = snapshot->test_tone_phase;
    g_test_tone.phase_inc = snapshot->test_tone_phase_inc;
    g_test_tone.amplitude = snapshot->test_tone_amplitude;
    apu_output_set_state(&snapshot->output_limiter);
    mcpx_apu_dsp_set_limiter_state(&snapshot->dsp_limiter);

    for (int i = 0; i < APU_MIXER_MAX_VOICES; i++) {
        APUMixerVoice *voice = &g_mixer_voices[i];
        const MCPXAPUMixerVoiceSnapshot *saved = &snapshot->mixer_voices[i];
        free(voice->savestate_pcm_data);
        voice->savestate_pcm_data = plan->pcm_data[i];
        plan->pcm_data[i] = NULL;
        voice->pcm_data = voice->savestate_pcm_data;
        voice->pcm_bytes = saved->pcm_bytes;
        voice->num_channels = saved->num_channels;
        voice->sample_rate = saved->sample_rate;
        voice->active = saved->active;
        voice->looping = saved->looping;
        voice->play_offset = saved->play_offset;
        voice->volume = saved->volume;
        active_count += voice->active != 0;
    }
    InterlockedExchange((volatile LONG *)&g_mixer_active_count, active_count);
    LeaveCriticalSection(&g_mixer_cs);
    qemu_mutex_unlock(&d->lock);
}

void mcpx_apu_savestate_discard(void *restore_plan)
{
    MCPXAPUSavestateRestorePlan *plan = (MCPXAPUSavestateRestorePlan *)restore_plan;
    if (!plan)
        return;
    for (int i = 0; i < APU_MIXER_MAX_VOICES; i++)
        free(plan->pcm_data[i]);
    free(plan->output_queue);
    free(plan);
}

void mcpx_apu_resume_after_savestate(int loaded, int audio_restored)
{
    MCPXAPUState *d = g_state;
    if (!d)
        return;

    qemu_mutex_lock(&d->lock);
    if (loaded) {
        if (xa2_is_active() && !audio_restored) {
            if (!xa2_clear_queue())
                fprintf(stderr, "[APU] host audio queue could not be fully cleared after load\n");
        } else if (g_waveout.initialized) {
#if defined(_WIN32)
            MMRESULT mr = waveOutReset(g_waveout.hwo);
            if (mr != MMSYSERR_NOERROR)
                fprintf(stderr, "[APU] waveOutReset failed after load (error %u)\n", mr);
            g_waveout.next_buf = 0;
#endif
        }
        if (!audio_restored) {
            apu_output_reset();
            memset(d->monitor.frame_buf, 0, sizeof(d->monitor.frame_buf));
            d->ep_frame_div = 0;
            d->pause_requested = false;
        }
        d->frame_count = 0;
        d->frame_count_time_ms = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    }
    apu_resume_output_device();
    d->next_frame_time_us = qemu_clock_get_us(QEMU_CLOCK_REALTIME);
    d->sleep_acc_us = 0;
    d->savestate_pause_requested = false;
    qemu_cond_broadcast(&d->cond);
    qemu_mutex_unlock(&d->lock);
}

/* ============================================================
 * Reset
 * ============================================================ */

static void mcpx_apu_reset_locked(MCPXAPUState *d)
{
    memset(d->regs, 0, sizeof(d->regs));
    mcpx_apu_vp_reset(d);

    if (d->gp.dsp) {
        memset((void *)d->gp.dsp->core.pram_opcache, 0,
               sizeof(d->gp.dsp->core.pram_opcache));
    }
    if (d->ep.dsp) {
        memset((void *)d->ep.dsp->core.pram_opcache, 0,
               sizeof(d->ep.dsp->core.pram_opcache));
    }
    d->set_irq = false;
}

/* ============================================================
 * Public API: Init / Shutdown
 * ============================================================ */

MCPXAPUState *mcpx_apu_init_standalone(uint8_t *ram_ptr)
{
    MCPXAPUState *d = (MCPXAPUState *)calloc(1, sizeof(MCPXAPUState));
    if (!d) {
        fprintf(stderr, "[APU] Failed to allocate MCPXAPUState\n");
        return NULL;
    }

    g_apu_ram_ptr = ram_ptr;
    g_state = d;
    d->ram_ptr = ram_ptr;

    d->set_irq = false;
    d->exiting = false;
    d->is_idle = false;
    d->pause_requested = true;

    qemu_mutex_init(&d->lock);
    qemu_mutex_lock(&d->lock);
    qemu_cond_init(&d->cond);
    qemu_cond_init(&d->idle_cond);

    /* Init VP (voice processor) */
    mcpx_apu_vp_init(d);

    /* Init DSP (GP/EP - stubbed) */
    mcpx_apu_dsp_init(d);

    /* Init software mixer for DirectSound bridge */
    mixer_init();

    /* Init monitor (waveOut output) */
    Error *local_err = NULL;
    mcpx_apu_monitor_init(d, &local_err);
    if (local_err) {
        warn_reportf_err(local_err, "mcpx_apu_monitor_init failed: ");
    }

    /* Start background frame thread */
    qemu_thread_create(&d->apu_thread, "mcpx.apu_thread",
                       mcpx_apu_frame_thread, d, QEMU_THREAD_JOINABLE);
    mcpx_apu_wait_for_idle(d);
    qemu_mutex_unlock(&d->lock);

    fprintf(stderr, "[APU] MCPX APU initialized (standalone)\n");
    fprintf(stderr, "[APU]   RAM pointer: %p\n", (void *)ram_ptr);
    fprintf(stderr, "[APU]   MMIO base: 0xFE800000 (512KB)\n");
    fprintf(stderr, "[APU]   VP: %d max voices, %d samples/frame\n",
            MCPX_HW_MAX_VOICES, NUM_SAMPLES_PER_FRAME);
    return d;
}

void mcpx_apu_shutdown(MCPXAPUState *d)
{
    if (!d) return;

    fprintf(stderr, "[APU] Shutting down MCPX APU...\n");

    qemu_mutex_lock(&d->lock);
    mcpx_apu_wait_for_idle(d);
    qatomic_set(&d->exiting, true);
    qemu_cond_signal(&d->cond);
    qemu_mutex_unlock(&d->lock);

    qemu_thread_join(&d->apu_thread);
    mcpx_apu_vp_finalize(d);
    mcpx_apu_monitor_finalize(d);

    free(d);
    g_state = NULL;
    fprintf(stderr, "[APU] Shutdown complete\n");
}

/* ============================================================
 * VP MMIO handlers (sub-region at +0x20000)
 *
 * These are called when the game writes to the VP PIO registers
 * to configure voices, SSL, etc.
 * ============================================================ */

uint64_t mcpx_apu_vp_read(void *opaque, hwaddr addr, unsigned int size);
void mcpx_apu_vp_write(void *opaque, hwaddr addr, uint64_t val, unsigned int size);

/* Dispatch a VP-region access (offset 0x20000-0x2FFFF from APU base) */
void mcpx_apu_dispatch_mmio(MCPXAPUState *d, hwaddr addr, uint64_t val,
                             unsigned int size, bool is_write)
{
    if (addr >= 0x20000 && addr < 0x30000) {
        /* VP region */
        hwaddr vp_addr = addr - 0x20000;
        if (is_write) {
            mcpx_apu_vp_write(d, vp_addr, val, size);
        }
        /* VP reads handled by caller if needed */
    } else if (addr < 0x20000) {
        /* Main APU registers */
        if (is_write) {
            mcpx_apu_write(d, addr, val, size);
        }
    }
    /* GP (0x30000) and EP (0x50000) regions ignored for now */
}

/* ============================================================
 * Public MMIO API (called from VEH or MMIO hook)
 * addr is offset from APU base (0xFE800000)
 * ============================================================ */

uint64_t mcpx_apu_mmio_read(MCPXAPUState *d, uint64_t addr, unsigned int size)
{
    if (!d) return 0;
    if (addr >= 0x20000 && addr < 0x30000) {
        return mcpx_apu_vp_read(d, addr - 0x20000, size);
    } else if (addr < 0x20000) {
        return mcpx_apu_read(d, (hwaddr)addr, size);
    }
    return 0;
}

void mcpx_apu_mmio_write(MCPXAPUState *d, uint64_t addr, uint64_t val, unsigned int size)
{
    if (!d) return;
    mcpx_apu_dispatch_mmio(d, (hwaddr)addr, val, size, true);
}

/* ============================================================
 * APU Test Tone - Direct waveOut sine generator
 *
 * Bypasses the VP pipeline entirely and writes a 440Hz sine wave
 * directly to the monitor frame_buf. This verifies that waveOut
 * output works correctly.
 * ============================================================ */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void mcpx_apu_play_test_tone(MCPXAPUState *d)
{
    if (!d) {
        fprintf(stderr, "[APU-TEST] No APU state\n");
        return;
    }

    if (g_test_tone.active) {
        /* Toggle off */
        g_test_tone.active = false;
        fprintf(stderr, "[APU-TEST] Test tone OFF\n");
        return;
    }

    /* 440Hz at 48kHz sample rate */
    g_test_tone.phase = 0.0;
    g_test_tone.phase_inc = 2.0 * M_PI * 440.0 / 48000.0;
    g_test_tone.amplitude = 6000;  /* ~18% of full scale */
    g_test_tone.active = true;

    /* Make sure waveOut is running - enable SECTL and resume APU thread */
    qemu_mutex_lock(&d->lock);
    d->regs[NV_PAPU_SECTL] = NV_PAPU_SECTL_XCNTMODE & ~NV_PAPU_SECTL_XCNTMODE_OFF;
    d->regs[NV_PAPU_FECTL] = NV_PAPU_FECTL_FEMETHMODE_FREE_RUNNING;
    /* Initialize empty voice lists so VP frame doesn't crash */
    d->regs[NV_PAPU_TVL2D] = 0xFFFF;
    d->regs[NV_PAPU_TVL3D] = 0xFFFF;
    d->regs[NV_PAPU_TVLMP] = 0xFFFF;
    mcpx_apu_resume(d);
    qemu_mutex_unlock(&d->lock);

    fprintf(stderr, "[APU-TEST] Test tone ON - 440Hz sine, amplitude=%d\n",
            g_test_tone.amplitude);
}

/* ============================================================
 * Software mixer - mixes DirectSound buffers to waveOut
 *
 * This bypasses the VP hardware voice pipeline entirely.
 * DirectSound buffers register PCM data here, and the APU
 * frame thread mixes them into the monitor frame_buf.
 * ============================================================ */

static void mixer_init(void)
{
    if (g_mixer_initialized) return;
    InitializeCriticalSection(&g_mixer_cs);
    memset(g_mixer_voices, 0, sizeof(g_mixer_voices));
    g_mixer_initialized = true;
}

int apu_mixer_alloc_voice(void)
{
    if (!g_mixer_initialized) mixer_init();
    EnterCriticalSection(&g_mixer_cs);
    for (int i = 0; i < APU_MIXER_MAX_VOICES; i++) {
        if (!g_mixer_voices[i].active && !g_mixer_voices[i].pcm_data) {
            g_mixer_voices[i].volume = 1.0f;
            g_mixer_voices[i].sample_rate = 44100;
            g_mixer_voices[i].num_channels = 2;
            LeaveCriticalSection(&g_mixer_cs);
            return i;
        }
    }
    LeaveCriticalSection(&g_mixer_cs);
    return -1;
}

void apu_mixer_free_voice(int slot)
{
    if (slot < 0 || slot >= APU_MIXER_MAX_VOICES) return;
    EnterCriticalSection(&g_mixer_cs);
    apu_mixer_stop(slot);
    free(g_mixer_voices[slot].savestate_pcm_data);
    g_mixer_voices[slot].savestate_pcm_data = NULL;
    g_mixer_voices[slot].pcm_data = NULL;
    g_mixer_voices[slot].pcm_bytes = 0;
    g_mixer_voices[slot].play_offset = 0;
    LeaveCriticalSection(&g_mixer_cs);
}

APUMixerVoice *apu_mixer_get_voice(int slot)
{
    if (slot < 0 || slot >= APU_MIXER_MAX_VOICES) return NULL;
    return &g_mixer_voices[slot];
}

void apu_mixer_set_buffer_data(int slot, const int16_t *pcm_data,
                               uint32_t pcm_bytes, uint32_t num_channels,
                               uint32_t sample_rate)
{
    if (slot < 0 || slot >= APU_MIXER_MAX_VOICES) return;
    EnterCriticalSection(&g_mixer_cs);
    APUMixerVoice *v = &g_mixer_voices[slot];
    free(v->savestate_pcm_data);
    v->savestate_pcm_data = NULL;
    v->pcm_data = pcm_data;
    v->pcm_bytes = pcm_bytes;
    v->num_channels = num_channels;
    v->sample_rate = sample_rate;
    v->play_offset = 0;
    LeaveCriticalSection(&g_mixer_cs);
}

int apu_mixer_set_position(int slot, uint32_t byte_offset)
{
    if (slot < 0 || slot >= APU_MIXER_MAX_VOICES) return 0;
    EnterCriticalSection(&g_mixer_cs);
    APUMixerVoice *v = &g_mixer_voices[slot];
    uint32_t frame_bytes = v->num_channels * sizeof(int16_t);
    int valid = (v->num_channels == 1 || v->num_channels == 2) &&
                byte_offset < v->pcm_bytes &&
                byte_offset / frame_bytes < v->pcm_bytes / frame_bytes;
    if (valid) v->play_offset = ((uint64_t)(byte_offset / frame_bytes)) << 16;
    LeaveCriticalSection(&g_mixer_cs);
    return valid;
}

void apu_mixer_get_state(int slot, uint32_t *byte_offset, int *active, int *looping)
{
    if (byte_offset) *byte_offset = 0;
    if (active) *active = 0;
    if (looping) *looping = 0;
    if (slot < 0 || slot >= APU_MIXER_MAX_VOICES) return;
    EnterCriticalSection(&g_mixer_cs);
    APUMixerVoice *v = &g_mixer_voices[slot];
    if (byte_offset) *byte_offset = (uint32_t)(v->play_offset >> 16) *
                                    v->num_channels * sizeof(int16_t);
    if (active) *active = v->active;
    if (looping) *looping = v->active && v->looping;
    LeaveCriticalSection(&g_mixer_cs);
}

void apu_mixer_play(int slot, int looping)
{
    if (g_audio_muted) return;
    if (slot < 0 || slot >= APU_MIXER_MAX_VOICES) return;
    EnterCriticalSection(&g_mixer_cs);
    APUMixerVoice *v = &g_mixer_voices[slot];
    if (!v->pcm_data || (v->num_channels != 1 && v->num_channels != 2) ||
        v->pcm_bytes < v->num_channels * sizeof(int16_t)) {
        LeaveCriticalSection(&g_mixer_cs);
        return;
    }
    v->looping = looping;
    if (!v->active) InterlockedIncrement((volatile LONG *)&g_mixer_active_count);
    v->active = 1;

    static int play_log_count = 0;
    if (play_log_count < 20) {
        fprintf(stderr, "[APU-MIX] Play voice %d: %u bytes, %u ch, %u Hz, vol=%.2f, loop=%d\n",
                slot, v->pcm_bytes, v->num_channels, v->sample_rate, v->volume, looping);
        play_log_count++;
    }
    LeaveCriticalSection(&g_mixer_cs);

    /* The frame thread takes the APU lock before the mixer lock. */
    extern MCPXAPUState *g_state;
    if (g_state) {
        mcpx_apu_lock_guest(g_state);
        g_state->pause_requested = false;
        qemu_cond_signal(&g_state->cond);
        qemu_mutex_unlock(&g_state->lock);
    }
}

void apu_mixer_stop(int slot)
{
    if (slot < 0 || slot >= APU_MIXER_MAX_VOICES) return;
    EnterCriticalSection(&g_mixer_cs);
    if (g_mixer_voices[slot].active) {
        g_mixer_voices[slot].active = 0;
        InterlockedDecrement((volatile LONG *)&g_mixer_active_count);
    }
    LeaveCriticalSection(&g_mixer_cs);
}

/* Mix all active voices into frame_buf. Called from mcpx_apu_monitor_frame.
 * Keep 16 fractional bits in a wide offset so buffers can exceed 65536 frames. */
static void mixer_render(int16_t frame_buf[][2], int num_samples)
{
    if (!g_mixer_initialized) return;
    EnterCriticalSection(&g_mixer_cs);

    for (int v = 0; v < APU_MIXER_MAX_VOICES; v++) {
        APUMixerVoice *voice = &g_mixer_voices[v];
        if (!voice->active || !voice->pcm_data || voice->pcm_bytes == 0)
            continue;

        uint32_t total_frames = voice->pcm_bytes / sizeof(int16_t);
        if (voice->num_channels == 2) total_frames /= 2;
        if (total_frames == 0) continue;

        /* Fixed-point 16.16 increment per output sample */
        uint64_t inc = ((uint64_t)voice->sample_rate << 16) / 48000;
        uint64_t pos = voice->play_offset;
        uint64_t end = (uint64_t)total_frames << 16;
        float vol = voice->volume;

        for (int i = 0; i < num_samples; i++) {
            uint32_t src_frame = pos >> 16;

            if (src_frame >= total_frames) {
                if (voice->looping) {
                    pos %= end;
                    src_frame = (uint32_t)(pos >> 16);
                } else {
                    pos = 0;
                    voice->active = 0;
                    InterlockedDecrement((volatile LONG *)&g_mixer_active_count);
                    break;
                }
            }

            int32_t left, right;
            if (voice->num_channels >= 2) {
                left  = (int32_t)(voice->pcm_data[src_frame * 2] * vol);
                right = (int32_t)(voice->pcm_data[src_frame * 2 + 1] * vol);
            } else {
                left = right = (int32_t)(voice->pcm_data[src_frame] * vol);
            }

            /* Accumulate (mix) into frame_buf with clamping */
            int32_t mixed_l = frame_buf[i][0] + left;
            int32_t mixed_r = frame_buf[i][1] + right;
            if (mixed_l > 32767) mixed_l = 32767;
            if (mixed_l < -32768) mixed_l = -32768;
            if (mixed_r > 32767) mixed_r = 32767;
            if (mixed_r < -32768) mixed_r = -32768;
            frame_buf[i][0] = (int16_t)mixed_l;
            frame_buf[i][1] = (int16_t)mixed_r;

            pos += inc;
        }

        voice->play_offset = pos;
        uint32_t end_frame = pos >> 16;
        if (end_frame >= total_frames) {
            if (voice->looping) {
                voice->play_offset = pos % end;
            } else if (voice->active) {
                voice->play_offset = 0;
                voice->active = 0;
                InterlockedDecrement((volatile LONG *)&g_mixer_active_count);
            }
        }
    }
    LeaveCriticalSection(&g_mixer_cs);
}
