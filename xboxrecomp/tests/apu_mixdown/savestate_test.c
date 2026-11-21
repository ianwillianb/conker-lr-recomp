#include "apu_state.h"
#include "apu.h"
#include "apu_xaudio2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int xbox_gil_suspend(void) { return 0; }
void xbox_gil_resume(int depth) { (void)depth; }
void mcpx_apu_dsp_get_limiter_state(ApuLimiter *out);
void mcpx_apu_dsp_set_limiter_state(const ApuLimiter *state);

int main(void)
{
    const size_t ram_size = 64u * 1024u * 1024u;
    uint8_t *ram = (uint8_t *)calloc(1, ram_size);
    MCPXAPUState *apu;
    size_t snapshot_size;
    size_t queue_snapshot_size;
    void *snapshot, *restore_plan;
    void *queue_snapshot, *restored_queue;
    uint32_t saved_queue_frames = 0, restored_queue_frames = 0;
    int16_t pcm[] = { 100, 200, 300, 400 };
    int slot;
    int saved_queue;
    APUMixerVoice *voice;
    ApuLimiter limiter = { 0 }, restored_limiter;
    ApuLimiter dsp_limiter = { 0 }, restored_dsp_limiter;

    if (!ram) {
        fprintf(stderr, "out of memory allocating guest RAM\n");
        return 2;
    }

    apu = mcpx_apu_init_standalone(ram);
    if (!apu) {
        free(ram);
        return 2;
    }
    if (xa2_is_active()) {
        mcpx_apu_play_test_tone(apu);
        int waits = 0;
        while (xa2_queued() == 0 && waits++ < 500)
            Sleep(1);
        if (xa2_queued() == 0) {
            fprintf(stderr, "audio output did not queue a block before pause test\n");
            mcpx_apu_shutdown(apu);
            free(ram);
            return 1;
        }
    }
    if (!mcpx_apu_pause_for_savestate(2000)) {
        mcpx_apu_shutdown(apu);
        free(ram);
        return 1;
    }
    int paused_queue = xa2_queued();
    int paused_ep_frame = apu->ep_frame_div;
    Sleep(50);
    if (apu->ep_frame_div != paused_ep_frame ||
        (xa2_is_active() && xa2_queued() != paused_queue)) {
        fprintf(stderr, "APU state or host audio queue advanced while paused\n");
        mcpx_apu_resume_after_savestate(0, 0);
        mcpx_apu_shutdown(apu);
        free(ram);
        return 1;
    }
    if (xa2_is_active())
        mcpx_apu_play_test_tone(apu);
    mcpx_apu_resume_after_savestate(0, 0);
    if (!mcpx_apu_pause_for_savestate(2000)) {
        fprintf(stderr, "could not re-pause APU after save-pause test\n");
        mcpx_apu_shutdown(apu);
        free(ram);
        return 1;
    }

    apu->regs[0x1234] = 0xA55A1234;
    apu->gp.regs[0x234] = 0x12345678;
    apu->ep.regs[0x345] = 0x87654321;
    apu->vp.filters[7].svf[0].f = 0.25f;
    apu->vp.filters[7].svf[0].l = 0.75f;
    apu->vp.filters[7].svf[0].op = &apu->vp.filters[7].svf[0].l;
    apu->vp.filters[7].hrtf.ch[0].buf[3] = 0.5f;

    slot = apu_mixer_alloc_voice();
    if (slot < 0) {
        fprintf(stderr, "could not allocate mixer voice\n");
        mcpx_apu_resume_after_savestate(0, 0);
        mcpx_apu_shutdown(apu);
        free(ram);
        return 1;
    }
    apu_mixer_set_buffer_data(slot, pcm, sizeof(pcm), 1, 48000);
    voice = apu_mixer_get_voice(slot);
    voice->active = 1;
    voice->looping = 1;
    voice->play_offset = (uint64_t)2 << 16;
    voice->volume = 0.375f;

    limiter.gain = 0.625f;
    limiter.target = 0.75f;
    limiter.pos = 7;
    limiter.delay[7][0] = 0.125f;
    apu_output_set_state(&limiter);
    dsp_limiter.gain = 0.5f;
    dsp_limiter.target = 0.875f;
    dsp_limiter.pos = 11;
    dsp_limiter.delay[11][1] = -0.25f;
    mcpx_apu_dsp_set_limiter_state(&dsp_limiter);

    saved_queue = xa2_queued();
    queue_snapshot_size = xa2_queue_snapshot_size();
    queue_snapshot = malloc(queue_snapshot_size);
    if (!queue_snapshot ||
        !xa2_queue_snapshot_save(queue_snapshot, queue_snapshot_size)) {
        fprintf(stderr, "could not capture queued host PCM\n");
        free(queue_snapshot);
        mcpx_apu_resume_after_savestate(0, 0);
        mcpx_apu_shutdown(apu);
        free(ram);
        return 1;
    }
    memcpy(&saved_queue_frames, (uint8_t *)queue_snapshot + sizeof(uint32_t),
           sizeof(saved_queue_frames));
    snapshot_size = mcpx_apu_savestate_size();
    snapshot = malloc(snapshot_size);
    if (!snapshot || !mcpx_apu_savestate_save(snapshot, snapshot_size)) {
        fprintf(stderr, "could not capture APU state\n");
        free(snapshot);
        free(queue_snapshot);
        apu_mixer_stop(slot);
        apu_mixer_free_voice(slot);
        mcpx_apu_resume_after_savestate(0, 0);
        mcpx_apu_shutdown(apu);
        free(ram);
        return 1;
    }

    apu->regs[0x1234] = 0;
    apu->gp.regs[0x234] = 0;
    apu->ep.regs[0x345] = 0;
    apu->vp.filters[7].svf[0].f = 0.0f;
    apu->vp.filters[7].svf[0].op = NULL;
    apu->vp.filters[7].hrtf.ch[0].buf[3] = 0.0f;
    apu_mixer_set_buffer_data(slot, NULL, 0, 1, 48000);
    apu_output_reset();
    memset(&restored_dsp_limiter, 0, sizeof(restored_dsp_limiter));
    mcpx_apu_dsp_set_limiter_state(&restored_dsp_limiter);

    voice = apu_mixer_get_voice(slot);
    voice->active = 0;
    apu_mixer_free_voice(slot);
    mcpx_apu_resume_after_savestate(0, 0);
    mcpx_apu_shutdown(apu);

    apu = mcpx_apu_init_standalone(ram);
    if (!apu || !mcpx_apu_pause_for_savestate(2000)) {
        fprintf(stderr, "could not initialize a fresh APU instance\n");
        free(snapshot);
        free(queue_snapshot);
        if (apu)
            mcpx_apu_shutdown(apu);
        free(ram);
        return 1;
    }

    restore_plan = mcpx_apu_savestate_prepare(snapshot, snapshot_size);
    if (!restore_plan) {
        fprintf(stderr, "could not validate APU state\n");
        free(snapshot);
        free(queue_snapshot);
        mcpx_apu_resume_after_savestate(0, 0);
        mcpx_apu_shutdown(apu);
        free(ram);
        return 1;
    }
    mcpx_apu_savestate_commit(restore_plan);
    mcpx_apu_savestate_discard(restore_plan);
    free(snapshot);
    restored_queue = malloc(queue_snapshot_size);
    int queue_matches = restored_queue &&
        xa2_queue_snapshot_save(restored_queue, queue_snapshot_size);
    if (queue_matches) {
        memcpy(&restored_queue_frames,
               (uint8_t *)restored_queue + sizeof(uint32_t),
               sizeof(restored_queue_frames));
        queue_matches = memcmp(queue_snapshot, restored_queue, queue_snapshot_size) == 0;
    }
    printf("Queued PCM: %.2f ms before load, %.2f ms restored; %.2f ms skipped "
           "(clearing instead would skip %.2f ms).\n",
           saved_queue_frames * 1000.0 / 48000.0,
           restored_queue_frames * 1000.0 / 48000.0,
           (saved_queue_frames > restored_queue_frames
                ? saved_queue_frames - restored_queue_frames : 0) *
               1000.0 / 48000.0,
           saved_queue_frames * 1000.0 / 48000.0);
    free(restored_queue);
    free(queue_snapshot);

    voice = apu_mixer_get_voice(slot);
    apu_output_get_state(&restored_limiter);
    mcpx_apu_dsp_get_limiter_state(&restored_dsp_limiter);
    if (apu->regs[0x1234] != 0xA55A1234 ||
        apu->gp.regs[0x234] != 0x12345678 ||
        apu->ep.regs[0x345] != 0x87654321 ||
        apu->vp.filters[7].svf[0].f != 0.25f ||
        apu->vp.filters[7].svf[0].op != &apu->vp.filters[7].svf[0].l ||
        apu->vp.filters[7].hrtf.ch[0].buf[3] != 0.5f ||
        !voice->active || !voice->looping ||
        voice->play_offset != ((uint64_t)2 << 16) ||
        voice->volume != 0.375f || !voice->savestate_pcm_data ||
        memcmp(voice->pcm_data, pcm, sizeof(pcm)) != 0 ||
        restored_limiter.gain != limiter.gain ||
        restored_limiter.target != limiter.target ||
        restored_limiter.pos != limiter.pos ||
        restored_limiter.delay[7][0] != limiter.delay[7][0] ||
        restored_dsp_limiter.gain != dsp_limiter.gain ||
        restored_dsp_limiter.target != dsp_limiter.target ||
        restored_dsp_limiter.pos != dsp_limiter.pos ||
        restored_dsp_limiter.delay[11][1] != dsp_limiter.delay[11][1] ||
        (xa2_is_active() && (xa2_queued() != saved_queue || !queue_matches))) {
        fprintf(stderr, "APU save-state round trip did not restore audio state\n");
        apu_mixer_stop(slot);
        apu_mixer_free_voice(slot);
        mcpx_apu_resume_after_savestate(0, 0);
        mcpx_apu_shutdown(apu);
        free(ram);
        return 1;
    }

    apu_mixer_stop(slot);
    apu_mixer_free_voice(slot);
    mcpx_apu_resume_after_savestate(0, 0);
    mcpx_apu_shutdown(apu);
    free(ram);
    puts("APU save-state round trip: passed");
    return 0;
}
