/*
 * MCPX APU Audio Emulation - Public API
 *
 * Extracted from xemu (LGPL v2+), adapted for standalone use.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MCPXAPUState MCPXAPUState;

/* Initialize the APU emulation.
 * ram_ptr: pointer to the base of Xbox physical RAM (64MB).
 * Returns the APU state, or NULL on failure. */
MCPXAPUState *mcpx_apu_init_standalone(uint8_t *ram_ptr);

/* Shut down and free the APU state. */
void mcpx_apu_shutdown(MCPXAPUState *d);

/* Pause audio-frame processing and host playback around a guest snapshot/restore. */
int mcpx_apu_pause_for_savestate(unsigned timeout_ms);
void mcpx_apu_resume_after_savestate(int loaded, int audio_restored);
size_t mcpx_apu_savestate_size(void);
int mcpx_apu_savestate_save(void *buffer, size_t buffer_size);
void *mcpx_apu_savestate_prepare(const void *buffer, size_t buffer_size);
void mcpx_apu_savestate_commit(void *restore_plan);
void mcpx_apu_savestate_discard(void *restore_plan);

/* MMIO read from APU register space (addr is offset from 0xFE800000). */
uint64_t mcpx_apu_mmio_read(MCPXAPUState *d, uint64_t addr, unsigned int size);

/* MMIO write to APU register space (addr is offset from 0xFE800000). */
void mcpx_apu_mmio_write(MCPXAPUState *d, uint64_t addr, uint64_t val, unsigned int size);

/* Play a 440Hz test tone through the APU pipeline to verify audio output.
 * Directly programs a voice without going through DirectSound. */
void mcpx_apu_play_test_tone(MCPXAPUState *d);

/* ============================================================
 * Software mixer - DirectSound buffer bridge
 *
 * DirectSound buffers register here. The APU frame thread
 * mixes active buffers directly into the waveOut output,
 * bypassing the VP hardware voice pipeline.
 * ============================================================ */

#define APU_MIXER_MAX_VOICES 64

typedef struct APUMixerVoice {
    volatile int     active;       /* 1 = playing, 0 = stopped */
    volatile int     looping;      /* 1 = loop, 0 = one-shot */
    const int16_t   *pcm_data;     /* Pointer to 16-bit PCM samples */
    uint32_t         pcm_bytes;    /* Total buffer size in bytes */
    uint32_t         num_channels;  /* 1=mono, 2=stereo */
    uint32_t         sample_rate;   /* e.g. 22050, 44100, 48000 */
    uint64_t         play_offset; /* Source frame position, 16 fractional bits */
    float            volume;       /* 0.0 to 1.0 */
    int16_t         *savestate_pcm_data; /* Owned copy restored from a state */
} APUMixerVoice;

/* APU frames processed in the last second (1500 = real time) and the frame
 * thread's busy share of it. */
int   mcpx_apu_frames_per_second(void);
float mcpx_apu_utilization(void);

/* Allocate a mixer voice slot. Returns slot index or -1 if full. */
int apu_mixer_alloc_voice(void);

/* Free a mixer voice slot. */
void apu_mixer_free_voice(int slot);

/* Get pointer to a mixer voice for configuration. */
APUMixerVoice *apu_mixer_get_voice(int slot);

/* Replace a voice's PCM source and release any state-owned copy. */
void apu_mixer_set_buffer_data(int slot, const int16_t *pcm_data,
                               uint32_t pcm_bytes, uint32_t num_channels,
                               uint32_t sample_rate);

/* Seek in PCM bytes (rounded down to a whole frame). Returns 0 if invalid. */
int apu_mixer_set_position(int slot, uint32_t byte_offset);

/* Snapshot playback state under the mixer lock; output pointers may be NULL. */
void apu_mixer_get_state(int slot, uint32_t *byte_offset, int *active, int *looping);

/* Start/resume or stop playback without changing the current position. */
void apu_mixer_play(int slot, int looping);
void apu_mixer_stop(int slot);

#ifdef __cplusplus
}
#endif
