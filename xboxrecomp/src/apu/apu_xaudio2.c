/**
 * XAudio2 Audio Output Backend
 *
 * Provides low-latency audio output via XAudio2 (Win7+).
 * Called from the APU monitor frame to submit mixed samples.
 * Falls back gracefully if XAudio2 is unavailable.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "apu_xaudio2.h"

/* A stereo-linked peak limiter: it turns the gain down, never the waveform.
 * Each sample asks for the gain that keeps it under the ceiling; the lowest
 * ask is held while that sample crosses a 1.3 ms delay line and the applied
 * gain glides down to it in that time, so the gain is already there when the
 * peak comes out; then it recovers over ~120 ms. A final clamp catches the
 * residue of the glide (under 1%).
 *
 * Both stages used to be tanh per sample. That is a waveshaper: fine for a
 * stray spike, but a race sums to 1.27 x full scale for whole crashes, and
 * two of them in series (mixdown, then volume 100 driving this one 2x into
 * its ceiling) flattened FlatOut 2's crash sounds to a 4.6 dB crest factor --
 * heard as overdrive. */
void apu_limiter_run(ApuLimiter *lim, float *lr, int frames, float ceiling)
{
    const float att = 1.0f - expf(-5.0f / APU_LIMITER_LOOKAHEAD);
    const float rel = 1.0f - expf(-1.0f / (0.120f * 48000.0f));
    int i, c;

    if (!lim->init) {
        memset(lim, 0, sizeof *lim);
        lim->gain = lim->target = 1.0f;
        lim->init = 1;
    }
    for (i = 0; i < frames; i++) {
        float *s = lr + i * 2, *d = lim->delay[lim->pos];
        float pk = fabsf(s[0]) > fabsf(s[1]) ? fabsf(s[0]) : fabsf(s[1]);
        float want = pk > ceiling ? ceiling / pk : 1.0f;
        float out[2];

        if (want <= lim->target) {
            lim->target = want;
            lim->hold = APU_LIMITER_LOOKAHEAD;
        } else if (lim->hold > 0) {
            lim->hold--;
        } else {
            lim->target += (1.0f - lim->target) * rel;
        }
        if (lim->target < lim->gain)
            lim->gain += (lim->target - lim->gain) * att;
        else
            lim->gain = lim->target;

        for (c = 0; c < 2; c++) {
            float v = d[c] * lim->gain;
            out[c] = v > ceiling ? ceiling : v < -ceiling ? -ceiling : v;
            d[c] = s[c];
            s[c] = out[c];
        }
        lim->pos = (lim->pos + 1) % APU_LIMITER_LOOKAHEAD;
    }
}

/* Hearing safety, for every output path: master volume, then the limiter
 * with its ceiling at -6 dBFS (16384), so nothing the guest or the mixers
 * produce reaches full scale in a headset. Only one output path runs, so
 * one limiter state serves them all. */
static float g_xa2_volume = 0.5f;
static ApuLimiter g_output_limiter;

void apu_output_safety(const int16_t *in, int16_t *out, int n)
{
    static int peak_log = -1, in_peak, out_peak;
    static long peak_samples;
    float buf[512 * 2];
    int done, k, i;

    if (peak_log < 0) {                       /* RECOMP_AUDIO_PEAK=1: log the levels */
        const char *e = getenv("RECOMP_AUDIO_PEAK");
        peak_log = e && *e == '1';
    }
    for (done = 0; done < n; done += k) {
        k = n - done < (int)(sizeof buf / sizeof buf[0])
          ? n - done : (int)(sizeof buf / sizeof buf[0]);
        for (i = 0; i < k; i++)
            buf[i] = (float)in[done + i] * g_xa2_volume;
        apu_limiter_run(&g_output_limiter, buf, k / 2, 16384.0f);
        for (i = 0; i < k; i++)
            out[done + i] = (int16_t)lrintf(buf[i]);
    }
    if (peak_log) {
        for (i = 0; i < n; i++) {
            int a = in[i] < 0 ? -in[i] : in[i], b = out[i] < 0 ? -out[i] : out[i];
            if (a > in_peak) in_peak = a;
            if (b > out_peak) out_peak = b;
        }
        peak_samples += n;
        if (peak_samples >= 2 * 48000 * 2) {      /* about every 2 s of stereo 48 kHz */
            fprintf(stderr, "[audio] peak in %d, out %d (volume %.2f)\n",
                    in_peak, out_peak, g_xa2_volume);
            in_peak = out_peak = 0;
            peak_samples = 0;
        }
    }
}

void apu_output_reset(void)
{
    memset(&g_output_limiter, 0, sizeof(g_output_limiter));
}

void apu_output_get_state(ApuLimiter *out)
{
    *out = g_output_limiter;
}

void apu_output_set_state(const ApuLimiter *state)
{
    g_output_limiter = *state;
}

void xa2_set_master_volume(float v)
{
    g_xa2_volume = v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
}

#define XA2_QUEUE_SNAPSHOT_MAGIC 0x51554150u
#define XA2_QUEUE_SNAPSHOT_MAX_FRAMES 262144u

typedef struct Xa2QueueSnapshotHeader {
    uint32_t magic;
    uint32_t frames;
} Xa2QueueSnapshotHeader;

int xa2_queue_snapshot_validate(const void *buffer, size_t buffer_size)
{
    const Xa2QueueSnapshotHeader *header = (const Xa2QueueSnapshotHeader *)buffer;
    uint64_t expected;

    if (!buffer || buffer_size < sizeof(*header) ||
        header->magic != XA2_QUEUE_SNAPSHOT_MAGIC ||
        header->frames > XA2_QUEUE_SNAPSHOT_MAX_FRAMES)
        return 0;
    expected = sizeof(*header) +
               (uint64_t)header->frames * 2u * sizeof(int16_t);
    return expected == buffer_size;
}

/* The XAudio2 backend is Windows-only; on other hosts the same xa2_* calls
 * are served by SDL2 audio (below). */
#if defined(_WIN32)

#define COBJMACROS
#include <windows.h>
#include <xaudio2.h>

#pragma comment(lib, "xaudio2.lib")
#pragma comment(lib, "ole32.lib")

#define XA2_SAMPLE_RATE   48000
#define XA2_CHANNELS      2
#define XA2_BUF_SAMPLES   1024   /* ~21ms per submission */
#define XA2_NUM_BUFS      68     /* Covers the SDL backend's maximum queued PCM. */

static IXAudio2               *g_xa2 = NULL;
static IXAudio2MasteringVoice *g_xa2_master = NULL;
static IXAudio2SourceVoice    *g_xa2_source = NULL;
static int16_t                 g_xa2_bufs[XA2_NUM_BUFS][XA2_BUF_SAMPLES][2];
static int                     g_xa2_next_buf = 0;
static uint32_t                g_xa2_buf_frames[XA2_NUM_BUFS];
static int                     g_xa2_queue_head = 0;
static int                     g_xa2_queue_count = 0;
static uint32_t                g_xa2_head_offset = 0;
static uint64_t                g_xa2_last_samples_played = 0;
static int                     g_xa2_initialized = 0;
static int                     g_xa2_output_paused = 0;
static int                     g_xa2_frames_written = 0;
static Xa2Stats                g_xa2_stats;

static uint64_t xa2_pending_frames(void)
{
    uint64_t frames = 0;
    for (int i = 0; i < g_xa2_queue_count; i++) {
        int idx = (g_xa2_queue_head + i) % XA2_NUM_BUFS;
        uint32_t offset = i == 0 ? g_xa2_head_offset : 0;
        if (g_xa2_buf_frames[idx] > offset)
            frames += g_xa2_buf_frames[idx] - offset;
    }
    return frames;
}

static void xa2_consume_frames(uint64_t frames)
{
    while (frames && g_xa2_queue_count > 0) {
        uint32_t available = g_xa2_buf_frames[g_xa2_queue_head] - g_xa2_head_offset;
        if (frames < available) {
            g_xa2_head_offset += (uint32_t)frames;
            return;
        }
        frames -= available;
        g_xa2_buf_frames[g_xa2_queue_head] = 0;
        g_xa2_queue_head = (g_xa2_queue_head + 1) % XA2_NUM_BUFS;
        g_xa2_queue_count--;
        g_xa2_head_offset = 0;
    }
}

static void xa2_reconcile_queue(const XAUDIO2_VOICE_STATE *state)
{
    uint64_t delta = state->SamplesPlayed >= g_xa2_last_samples_played
        ? state->SamplesPlayed - g_xa2_last_samples_played
        : state->SamplesPlayed;
    uint64_t pending = xa2_pending_frames();

    g_xa2_last_samples_played = state->SamplesPlayed;
    if (delta > pending)
        delta = pending;
    xa2_consume_frames(delta);

    /* Keep the shadow ring consistent if a buffer completion happened between
     * the backend's polling calls. */
    while (g_xa2_queue_count > (int)state->BuffersQueued) {
        g_xa2_buf_frames[g_xa2_queue_head] = 0;
        g_xa2_queue_head = (g_xa2_queue_head + 1) % XA2_NUM_BUFS;
        g_xa2_queue_count--;
        g_xa2_head_offset = 0;
    }
}

int xa2_init(void)
{
    HRESULT hr;
    int com_initialized;
    WAVEFORMATEX wfx = { 0 };

    if (g_xa2_initialized) return 1;

    {   /* RECOMP_AUDIO_VOLUME = 0..100, as on the SDL path below */
        const char *vol = getenv("RECOMP_AUDIO_VOLUME");
        if (vol) xa2_set_master_volume((float)atoi(vol) / 100.0f);
    }

    hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if (FAILED(hr) && hr != RPC_E_CHANGED_MODE) {
        fprintf(stderr, "[XA2] CoInitializeEx failed: 0x%08lX\n", hr);
        return 0;
    }
    com_initialized = SUCCEEDED(hr);

    hr = XAudio2Create(&g_xa2, 0, XAUDIO2_DEFAULT_PROCESSOR);
    if (FAILED(hr) || !g_xa2) {
        fprintf(stderr, "[XA2] XAudio2Create failed: 0x%08lX\n", hr);
        goto fail;
    }

    hr = IXAudio2_CreateMasteringVoice(g_xa2, &g_xa2_master,
        XA2_CHANNELS, XA2_SAMPLE_RATE, 0, NULL, NULL, 0);
    if (FAILED(hr)) {
        fprintf(stderr, "[XA2] CreateMasteringVoice failed: 0x%08lX\n", hr);
        goto fail;
    }

    wfx.wFormatTag      = WAVE_FORMAT_PCM;
    wfx.nChannels       = XA2_CHANNELS;
    wfx.nSamplesPerSec  = XA2_SAMPLE_RATE;
    wfx.wBitsPerSample  = 16;
    wfx.nBlockAlign     = XA2_CHANNELS * 2;
    wfx.nAvgBytesPerSec = XA2_SAMPLE_RATE * wfx.nBlockAlign;

    hr = IXAudio2_CreateSourceVoice(g_xa2, &g_xa2_source,
        &wfx, 0, XAUDIO2_DEFAULT_FREQ_RATIO, NULL, NULL, NULL);
    if (FAILED(hr)) {
        fprintf(stderr, "[XA2] CreateSourceVoice failed: 0x%08lX\n", hr);
        goto fail;
    }

    hr = IXAudio2SourceVoice_Start(g_xa2_source, 0, XAUDIO2_COMMIT_NOW);
    if (FAILED(hr)) {
        fprintf(stderr, "[XA2] Start failed: 0x%08lX\n", hr);
        goto fail;
    }

    g_xa2_next_buf = 0;
    memset(g_xa2_buf_frames, 0, sizeof(g_xa2_buf_frames));
    g_xa2_queue_head = 0;
    g_xa2_queue_count = 0;
    g_xa2_head_offset = 0;
    {
        XAUDIO2_VOICE_STATE state;
        IXAudio2SourceVoice_GetState(g_xa2_source, &state, 0);
        g_xa2_last_samples_played = state.SamplesPlayed;
    }
    g_xa2_initialized = 1;
    g_xa2_output_paused = 0;
    g_xa2_frames_written = 0;

    fprintf(stderr, "[XA2] XAudio2 initialized (%d Hz stereo 16-bit, %d x %d-sample buffers)\n",
            XA2_SAMPLE_RATE, XA2_NUM_BUFS, XA2_BUF_SAMPLES);
    return 1;

fail:
    xa2_shutdown();
    /* Failed initialization still runs on the COM-initializing thread. */
    if (com_initialized) CoUninitialize();
    return 0;
}

void xa2_shutdown(void)
{
    if (g_xa2_source) {
        IXAudio2SourceVoice_Stop(g_xa2_source, 0, XAUDIO2_COMMIT_NOW);
        IXAudio2SourceVoice_FlushSourceBuffers(g_xa2_source);
        g_xa2_source->lpVtbl->DestroyVoice(g_xa2_source);
        g_xa2_source = NULL;
    }
    if (g_xa2_master) {
        g_xa2_master->lpVtbl->DestroyVoice(g_xa2_master);
        g_xa2_master = NULL;
    }
    if (g_xa2) {
        IXAudio2_Release(g_xa2);
        g_xa2 = NULL;
    }

    if (g_xa2_initialized)
        fprintf(stderr, "[XA2] Shut down (%d frames written)\n", g_xa2_frames_written);
    g_xa2_initialized = 0;
    g_xa2_output_paused = 0;
    g_xa2_queue_head = g_xa2_queue_count = 0;
    g_xa2_head_offset = 0;
}

int xa2_pause_output(void)
{
    HRESULT hr;
    if (!g_xa2_initialized || !g_xa2_source || g_xa2_output_paused)
        return 1;

    XAUDIO2_VOICE_STATE state;
    IXAudio2SourceVoice_GetState(g_xa2_source, &state, 0);
    xa2_reconcile_queue(&state);
    hr = IXAudio2SourceVoice_Stop(g_xa2_source, 0, XAUDIO2_COMMIT_NOW);
    if (FAILED(hr)) {
        fprintf(stderr, "[XA2] pausing output failed (0x%08lX)\n", hr);
        return 0;
    }
    g_xa2_output_paused = 1;
    return 1;
}

int xa2_resume_output(void)
{
    HRESULT hr;
    XAUDIO2_VOICE_STATE state;
    if (!g_xa2_initialized || !g_xa2_source || !g_xa2_output_paused)
        return 1;

    IXAudio2SourceVoice_GetState(g_xa2_source, &state, 0);
    g_xa2_last_samples_played = state.SamplesPlayed;
    hr = IXAudio2SourceVoice_Start(g_xa2_source, 0, XAUDIO2_COMMIT_NOW);
    if (FAILED(hr)) {
        fprintf(stderr, "[XA2] resuming output failed (0x%08lX)\n", hr);
        return 0;
    }
    g_xa2_output_paused = 0;
    return 1;
}

int xa2_clear_queue(void)
{
    HRESULT stop_hr, flush_hr;
    if (!g_xa2_initialized || !g_xa2_source)
        return 1;

    stop_hr = IXAudio2SourceVoice_Stop(g_xa2_source, 0, XAUDIO2_COMMIT_NOW);
    flush_hr = IXAudio2SourceVoice_FlushSourceBuffers(g_xa2_source);
    g_xa2_next_buf = 0;
    memset(g_xa2_buf_frames, 0, sizeof(g_xa2_buf_frames));
    g_xa2_queue_head = 0;
    g_xa2_queue_count = 0;
    g_xa2_head_offset = 0;
    if (SUCCEEDED(stop_hr))
        g_xa2_output_paused = 1;
    if (FAILED(stop_hr) || FAILED(flush_hr)) {
        fprintf(stderr, "[XA2] clearing audio queue failed (stop 0x%08lX, flush 0x%08lX)\n",
                stop_hr, flush_hr);
        return 0;
    }
    return 1;
}

int xa2_is_active(void)
{
    return g_xa2_initialized;
}

/* Submit a buffer of mixed samples to XAudio2.
 * Called from APU frame thread. Returns 1 if buffer was submitted. */
int xa2_submit_samples(const int16_t *samples, int num_samples)
{
    XAUDIO2_VOICE_STATE state;
    XAUDIO2_BUFFER xbuf;
    int idx;
    int copy_samples;
    HRESULT hr;

    if (!g_xa2_initialized || !g_xa2_source) return 0;

    IXAudio2SourceVoice_GetState(g_xa2_source, &state, 0);
    xa2_reconcile_queue(&state);
    if ((int)state.BuffersQueued >= XA2_NUM_BUFS) {
        g_xa2_stats.dropped++;
        return 0;
    }
    {
        static LARGE_INTEGER f, prev;
        LARGE_INTEGER now;
        if (!f.QuadPart) QueryPerformanceFrequency(&f);
        QueryPerformanceCounter(&now);
        if (state.BuffersQueued == 0 && g_xa2_stats.submitted) {
            g_xa2_stats.underruns++;
            if (getenv("RECOMP_APU_TRACE") && g_xa2_stats.underruns < 20)
                fprintf(stderr, "[XA2] underrun: %.1f ms since the previous block\n",
                        (now.QuadPart - prev.QuadPart) * 1000.0 / f.QuadPart);
        }
        prev = now;
    }

    idx = g_xa2_next_buf;
    copy_samples = (num_samples > XA2_BUF_SAMPLES) ? XA2_BUF_SAMPLES : num_samples;
    {
        int i, n = copy_samples * XA2_CHANNELS;
        for (i = 0; i < n; i++) {
            int a = samples[i] < 0 ? -samples[i] : samples[i];
            if (a >= 32767) g_xa2_stats.clipped++;
            if (a > g_xa2_stats.peak) g_xa2_stats.peak = a;
        }
        g_xa2_stats.samples += (uint64_t)copy_samples;
    }
    apu_output_safety(samples, &g_xa2_bufs[idx][0][0], copy_samples * XA2_CHANNELS);

    memset(&xbuf, 0, sizeof(xbuf));
    xbuf.AudioBytes = copy_samples * XA2_CHANNELS * sizeof(int16_t);
    xbuf.pAudioData = (const BYTE *)g_xa2_bufs[idx];

    hr = IXAudio2SourceVoice_SubmitSourceBuffer(g_xa2_source, &xbuf, NULL);
    if (FAILED(hr)) return 0;

    g_xa2_next_buf = (idx + 1) % XA2_NUM_BUFS;
    g_xa2_buf_frames[idx] = (uint32_t)copy_samples;
    if (g_xa2_queue_count == 0)
        g_xa2_queue_head = idx;
    g_xa2_queue_count++;
    g_xa2_frames_written++;
    g_xa2_stats.submitted++;
    return 1;
}

int xa2_get_buffer_size(void)
{
    return XA2_BUF_SAMPLES;
}

int xa2_queued(void)
{
    XAUDIO2_VOICE_STATE state;
    if (!g_xa2_initialized || !g_xa2_source) return 0;
    IXAudio2SourceVoice_GetState(g_xa2_source, &state, 0);
    xa2_reconcile_queue(&state);
    return (int)((xa2_pending_frames() + 255u) / 256u);
}

size_t xa2_queue_snapshot_size(void)
{
    XAUDIO2_VOICE_STATE state;
    uint64_t frames;
    if (!g_xa2_initialized || !g_xa2_source)
        return sizeof(Xa2QueueSnapshotHeader);
    IXAudio2SourceVoice_GetState(g_xa2_source, &state, 0);
    xa2_reconcile_queue(&state);
    frames = xa2_pending_frames();
    if (frames > XA2_QUEUE_SNAPSHOT_MAX_FRAMES)
        return 0;
    return sizeof(Xa2QueueSnapshotHeader) + (size_t)frames * 2u * sizeof(int16_t);
}

int xa2_queue_snapshot_save(void *buffer, size_t buffer_size)
{
    Xa2QueueSnapshotHeader *header = (Xa2QueueSnapshotHeader *)buffer;
    int16_t *pcm;
    uint64_t frames = 0;
    if (!buffer || (g_xa2_initialized && !g_xa2_output_paused) ||
        buffer_size != xa2_queue_snapshot_size())
        return 0;
    if (g_xa2_initialized && g_xa2_source) {
        XAUDIO2_VOICE_STATE state;
        IXAudio2SourceVoice_GetState(g_xa2_source, &state, 0);
        xa2_reconcile_queue(&state);
        frames = xa2_pending_frames();
    }
    header->magic = XA2_QUEUE_SNAPSHOT_MAGIC;
    header->frames = (uint32_t)frames;
    pcm = (int16_t *)(header + 1);
    for (int i = 0; i < g_xa2_queue_count; i++) {
        int idx = (g_xa2_queue_head + i) % XA2_NUM_BUFS;
        uint32_t offset = i == 0 ? g_xa2_head_offset : 0;
        uint32_t count = g_xa2_buf_frames[idx] - offset;
        memcpy(pcm, &g_xa2_bufs[idx][offset][0],
               (size_t)count * XA2_CHANNELS * sizeof(int16_t));
        pcm += count * XA2_CHANNELS;
    }
    return 1;
}

int xa2_queue_snapshot_restore(const void *buffer, size_t buffer_size)
{
    const Xa2QueueSnapshotHeader *header = (const Xa2QueueSnapshotHeader *)buffer;
    const int16_t *pcm;
    uint32_t left;
    if (!xa2_queue_snapshot_validate(buffer, buffer_size))
        return 0;
    if (!g_xa2_initialized || !g_xa2_source)
        return header->frames == 0;
    if (!g_xa2_output_paused)
        return 0;
    if (header->frames > XA2_NUM_BUFS * XA2_BUF_SAMPLES)
        return 0;
    if (!xa2_clear_queue())
        return 0;

    pcm = (const int16_t *)(header + 1);
    left = header->frames;
    while (left) {
        XAUDIO2_BUFFER xbuf;
        uint32_t count = left > XA2_BUF_SAMPLES ? XA2_BUF_SAMPLES : left;
        int idx = g_xa2_next_buf;
        memcpy(g_xa2_bufs[idx], pcm, (size_t)count * XA2_CHANNELS * sizeof(int16_t));
        memset(&xbuf, 0, sizeof(xbuf));
        xbuf.AudioBytes = count * XA2_CHANNELS * sizeof(int16_t);
        xbuf.pAudioData = (const BYTE *)g_xa2_bufs[idx];
        if (FAILED(IXAudio2SourceVoice_SubmitSourceBuffer(g_xa2_source, &xbuf, NULL))) {
            xa2_clear_queue();
            return 0;
        }
        g_xa2_buf_frames[idx] = count;
        if (g_xa2_queue_count == 0)
            g_xa2_queue_head = idx;
        g_xa2_queue_count++;
        g_xa2_next_buf = (idx + 1) % XA2_NUM_BUFS;
        pcm += count * XA2_CHANNELS;
        left -= count;
    }
    return 1;
}

int xa2_queue_target(void)
{
    return 8;
}

void xa2_get_stats(Xa2Stats *out)
{
    *out = g_xa2_stats;          /* ponytail: unlocked; counters only grow */
    g_xa2_stats.peak = 0;
}

#else /* !_WIN32 -- SDL2 audio (Linux, Nintendo Switch) */

/* The same contract as the XAudio2 path: the APU frame thread submits one
 * 256-sample block per 8 EP frames and paces itself by xa2_queued(), so the
 * output device's clock drains the queue and drives the APU clock (and with
 * it DirectSound's play cursor, which paces the title's movies).
 *
 * SDL_QueueAudio, not a callback: the frame thread already produces blocks
 * at the device's rate, and a callback would need a second ring and lock.
 * SDL copies each block into its own queue, so the samples never go to the
 * sound service from guest memory (on Horizon the audren driver copies from
 * SDL's mix buffer into its own pool).
 *
 * RECOMP_AUDIO=0 keeps the host device closed; the APU then paces by wall
 * clock as before. RECOMP_AUDIO_BLOCKS sets the queue depth the frame thread
 * keeps (256-sample blocks, default 8 = 43 ms; Switch 12 = 64 ms, since its
 * threads time-slice in 10 ms quanta). RECOMP_AUDIO_VOLUME = 0..100. */

#include <stdlib.h>
#include <SDL.h>

#define SDLA_SAMPLE_RATE   48000
#define SDLA_CHANNELS      2
#define SDLA_BLOCK_SAMPLES 256
#define SDLA_BLOCK_BYTES   (SDLA_BLOCK_SAMPLES * SDLA_CHANNELS * (int)sizeof(int16_t))
#define SDLA_MAX_SAMPLES   1024
#ifdef __SWITCH__
#define SDLA_DEFAULT_BLOCKS 12
#else
#define SDLA_DEFAULT_BLOCKS 8
#endif

static SDL_AudioDeviceID g_sdla_dev = 0;
static int               g_sdla_paused = 0;
static int               g_sdla_target = SDLA_DEFAULT_BLOCKS;
static int               g_sdla_frames_written = 0;
static int16_t           g_sdla_queue_shadow[XA2_QUEUE_SNAPSHOT_MAX_FRAMES * 2u];
static uint32_t          g_sdla_queue_shadow_frames;
static Xa2Stats          g_sdla_stats;

static void dump_init(void);

static int sdla_reconcile_queue(void)
{
    uint32_t queued_frames =
        SDL_GetQueuedAudioSize(g_sdla_dev) / (SDLA_CHANNELS * sizeof(int16_t));
    if (queued_frames > g_sdla_queue_shadow_frames) {
        fprintf(stderr, "[AUDIO] host queue exceeds the tracked PCM shadow\n");
        return 0;
    }
    if (queued_frames < g_sdla_queue_shadow_frames) {
        uint32_t consumed = g_sdla_queue_shadow_frames - queued_frames;
        memmove(g_sdla_queue_shadow,
                g_sdla_queue_shadow + consumed * SDLA_CHANNELS,
                (size_t)queued_frames * SDLA_CHANNELS * sizeof(int16_t));
        g_sdla_queue_shadow_frames = queued_frames;
    }
    return 1;
}

int xa2_init(void)
{
    SDL_AudioSpec want, have;
    const char *e;

    if (g_sdla_dev) return 1;

    e = getenv("RECOMP_AUDIO");
    if (e && strcmp(e, "0") == 0) {
        fprintf(stderr, "[AUDIO] host output off (RECOMP_AUDIO=0)\n");
        return 0;
    }
    e = getenv("RECOMP_AUDIO_BLOCKS");
    if (e && atoi(e) >= 2 && atoi(e) <= 64) g_sdla_target = atoi(e);
    e = getenv("RECOMP_AUDIO_VOLUME");
    if (e) xa2_set_master_volume((float)atoi(e) / 100.0f);

    if (!SDL_WasInit(SDL_INIT_AUDIO) && SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        fprintf(stderr, "[AUDIO] SDL audio init failed: %s\n", SDL_GetError());
        return 0;
    }

    SDL_zero(want);
    want.freq     = SDLA_SAMPLE_RATE;
    want.format   = AUDIO_S16SYS;
    want.channels = SDLA_CHANNELS;
    want.samples  = 1024;
    want.callback = NULL;       /* SDL_QueueAudio */

    /* No allowed changes: SDL converts if the device wants another format. */
    g_sdla_dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (!g_sdla_dev) {
        fprintf(stderr, "[AUDIO] SDL_OpenAudioDevice failed: %s\n", SDL_GetError());
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        return 0;
    }
    SDL_PauseAudioDevice(g_sdla_dev, 0);
    g_sdla_paused = 0;
    dump_init();

    g_sdla_frames_written = 0;
    g_sdla_queue_shadow_frames = 0;
    fprintf(stderr, "[AUDIO] SDL %s output: %d Hz, %d ch, %d-sample device buffer,"
                    " queue %d x %d samples\n",
            SDL_GetCurrentAudioDriver() ? SDL_GetCurrentAudioDriver() : "?",
            have.freq, have.channels, have.samples,
            g_sdla_target, SDLA_BLOCK_SAMPLES);
    return 1;
}

void xa2_shutdown(void)
{
    if (!g_sdla_dev) return;
    SDL_CloseAudioDevice(g_sdla_dev);
    g_sdla_dev = 0;
    g_sdla_paused = 0;
    g_sdla_queue_shadow_frames = 0;
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
    fprintf(stderr, "[AUDIO] shut down (%d blocks written, %llu underruns,"
                    " %llu dropped)\n", g_sdla_frames_written,
            (unsigned long long)g_sdla_stats.underruns,
            (unsigned long long)g_sdla_stats.dropped);
}

int xa2_clear_queue(void)
{
    if (g_sdla_dev) {
        SDL_ClearQueuedAudio(g_sdla_dev);
        g_sdla_queue_shadow_frames = 0;
    }
    return 1;
}

int xa2_pause_output(void)
{
    if (g_sdla_dev && !g_sdla_paused) {
        SDL_PauseAudioDevice(g_sdla_dev, 1);
        g_sdla_paused = 1;
    }
    return 1;
}

int xa2_resume_output(void)
{
    if (g_sdla_dev && g_sdla_paused) {
        SDL_PauseAudioDevice(g_sdla_dev, 0);
        g_sdla_paused = 0;
    }
    return 1;
}

int xa2_is_active(void)
{
    return g_sdla_dev != 0;
}

int xa2_queued(void)
{
    if (!g_sdla_dev) return 0;
    sdla_reconcile_queue();
    return (int)(SDL_GetQueuedAudioSize(g_sdla_dev) / SDLA_BLOCK_BYTES);
}

size_t xa2_queue_snapshot_size(void)
{
    if (g_sdla_dev && (!g_sdla_paused || !sdla_reconcile_queue()))
        return 0;
    if (g_sdla_queue_shadow_frames > XA2_QUEUE_SNAPSHOT_MAX_FRAMES)
        return 0;
    return sizeof(Xa2QueueSnapshotHeader) +
           (size_t)g_sdla_queue_shadow_frames * SDLA_CHANNELS * sizeof(int16_t);
}

int xa2_queue_snapshot_save(void *buffer, size_t buffer_size)
{
    Xa2QueueSnapshotHeader *header = (Xa2QueueSnapshotHeader *)buffer;
    uint32_t frames;

    if (!buffer || (g_sdla_dev &&
        (!g_sdla_paused || !sdla_reconcile_queue())) ||
        buffer_size != xa2_queue_snapshot_size())
        return 0;
    frames = g_sdla_queue_shadow_frames;
    header->magic = XA2_QUEUE_SNAPSHOT_MAGIC;
    header->frames = frames;
    memcpy(header + 1, g_sdla_queue_shadow,
           (size_t)frames * SDLA_CHANNELS * sizeof(int16_t));
    return 1;
}

int xa2_queue_snapshot_restore(const void *buffer, size_t buffer_size)
{
    const Xa2QueueSnapshotHeader *header = (const Xa2QueueSnapshotHeader *)buffer;
    Uint32 pcm_bytes;
    if (!xa2_queue_snapshot_validate(buffer, buffer_size))
        return 0;
    if (!g_sdla_dev)
        return header->frames == 0;
    if (!g_sdla_paused)
        return 0;

    pcm_bytes = header->frames * SDLA_CHANNELS * sizeof(int16_t);
    SDL_ClearQueuedAudio(g_sdla_dev);
    if (pcm_bytes && SDL_QueueAudio(g_sdla_dev, header + 1, pcm_bytes) != 0) {
        SDL_ClearQueuedAudio(g_sdla_dev);
        g_sdla_queue_shadow_frames = 0;
        fprintf(stderr, "[AUDIO] restoring queued PCM failed: %s\n", SDL_GetError());
        return 0;
    }
    memcpy(g_sdla_queue_shadow, header + 1, pcm_bytes);
    g_sdla_queue_shadow_frames = header->frames;
    return 1;
}

int xa2_queue_target(void)
{
    return g_sdla_target;
}

/* RECOMP_AUDIO_DUMP=<path>,<start s>,<seconds>: the exact PCM handed to the
 * device (48 kHz s16 stereo, after volume and limiter), from <start> seconds
 * after the device opened, kept in RAM and written in one go when full -- a
 * console capture to compare with a Linux one (SD writes while playing would
 * disturb what is being measured). */
static struct {
    int      on;
    FILE    *f;
    char     path[256];
    uint64_t skip, left;                /* samples */
    int16_t *buf;
    size_t   n, cap;                    /* int16 values */
} s_dump;

static void dump_init(void)
{
    const char *e = getenv("RECOMP_AUDIO_DUMP");
    char *c1, *c2;
    if (!e || !*e) return;
    snprintf(s_dump.path, sizeof s_dump.path, "%s", e);
    c1 = strchr(s_dump.path, ',');
    c2 = c1 ? strchr(c1 + 1, ',') : NULL;
    if (!c1 || !c2) return;
    *c1 = 0;
    s_dump.skip = (uint64_t)atoi(c1 + 1) * SDLA_SAMPLE_RATE;
    s_dump.left = (uint64_t)atoi(c2 + 1) * SDLA_SAMPLE_RATE;
    s_dump.cap = (size_t)s_dump.left * SDLA_CHANNELS;
    s_dump.buf = s_dump.cap ? (int16_t *)malloc(s_dump.cap * sizeof(int16_t)) : NULL;
    if (!s_dump.buf) return;
    s_dump.on = 1;
    fprintf(stderr, "[AUDIO] dump: %llu s from %llu s to %s\n",
            (unsigned long long)(s_dump.left / SDLA_SAMPLE_RATE),
            (unsigned long long)(s_dump.skip / SDLA_SAMPLE_RATE), s_dump.path);
}

static void dump_block(const int16_t *pcm, int num_samples)
{
    size_t take;
    if (!s_dump.on) return;
    if (s_dump.skip >= (uint64_t)num_samples) { s_dump.skip -= num_samples; return; }
    pcm += s_dump.skip * SDLA_CHANNELS;
    num_samples -= (int)s_dump.skip;
    s_dump.skip = 0;
    take = (size_t)num_samples * SDLA_CHANNELS;
    if (take > s_dump.cap - s_dump.n) take = s_dump.cap - s_dump.n;
    memcpy(s_dump.buf + s_dump.n, pcm, take * sizeof(int16_t));
    s_dump.n += take;
    if (s_dump.n < s_dump.cap) return;
    s_dump.on = 0;
    s_dump.f = fopen(s_dump.path, "wb");
    if (s_dump.f) {
        fwrite(s_dump.buf, sizeof(int16_t), s_dump.n, s_dump.f);
        fclose(s_dump.f);
    }
    fprintf(stderr, "[AUDIO] dump written: %s (%zu bytes)%s\n", s_dump.path,
            s_dump.n * sizeof(int16_t), s_dump.f ? "" : " -- open failed");
    free(s_dump.buf);
    s_dump.buf = NULL;
}

int xa2_submit_samples(const int16_t *samples, int num_samples)
{
    int16_t buf[SDLA_MAX_SAMPLES * SDLA_CHANNELS];
    int i, n, queued;

    if (!g_sdla_dev) return 0;
    if (num_samples > SDLA_MAX_SAMPLES) num_samples = SDLA_MAX_SAMPLES;

    /* A device that stopped draining (a suspended applet) must not grow the
     * queue without bound; the frame thread stops waiting on it too. */
    queued = xa2_queued();
    if (queued >= g_sdla_target * 4) {
        g_sdla_stats.dropped++;
        return 0;
    }
    if (queued == 0 && g_sdla_stats.submitted) {
        g_sdla_stats.underruns++;
        if (getenv("RECOMP_APU_TRACE") && g_sdla_stats.underruns < 20)
            fprintf(stderr, "[AUDIO] underrun\n");
    }

    n = num_samples * SDLA_CHANNELS;
    for (i = 0; i < n; i++) {
        int a = samples[i] < 0 ? -samples[i] : samples[i];
        if (a >= 32767) g_sdla_stats.clipped++;
        if (a > g_sdla_stats.peak) g_sdla_stats.peak = a;
    }
    g_sdla_stats.samples += (uint64_t)num_samples;
    apu_output_safety(samples, buf, n);
    dump_block(buf, num_samples);

    if (g_sdla_queue_shadow_frames + (uint32_t)num_samples >
        XA2_QUEUE_SNAPSHOT_MAX_FRAMES) {
        fprintf(stderr, "[AUDIO] tracked host queue exceeded its snapshot limit\n");
        return 0;
    }
    if (SDL_QueueAudio(g_sdla_dev, buf, (Uint32)(n * sizeof(int16_t))) != 0)
        return 0;
    memcpy(g_sdla_queue_shadow + g_sdla_queue_shadow_frames * SDLA_CHANNELS,
           buf, (size_t)n * sizeof(int16_t));
    g_sdla_queue_shadow_frames += (uint32_t)num_samples;
    g_sdla_frames_written++;
    g_sdla_stats.submitted++;
    return 1;
}

int xa2_get_buffer_size(void)
{
    return SDLA_BLOCK_SAMPLES;
}

void xa2_get_stats(Xa2Stats *out)
{
    *out = g_sdla_stats;         /* unlocked; counters only grow */
    g_sdla_stats.peak = 0;
}

#endif /* _WIN32 */
