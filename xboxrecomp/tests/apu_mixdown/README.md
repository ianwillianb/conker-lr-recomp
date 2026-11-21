# apu_mixdown

`mcpx_apu_dsp_frame` used to read mixbins 0 and 1 and discard 2..31. On
hardware the GP and EP mix the submixes down; here they are stubs, so thirty
of thirty-two bins were computed correctly and thrown away every frame, with
no counter anywhere to say so.

Titles route their 3D positional voices — their sound effects — to bins above
1. Measured on Jet Set Radio Future over one 200 s gameplay run:

    [APU-BIN] 2D heard=557466 lost=0
              3D heard=0      lost=377768
              lost by bin: 6,7,8,9,10

Music is on 2D voices and lands in bins 0 and 1, which is why it was always
audible while no effect ever was.

## Running

    cmake -S tests/apu_mixdown -B build/apu-mixdown
    cmake --build build/apu-mixdown
    ctest --test-dir build/apu-mixdown

The two mixdown tests both matter. `apu_mixdown_all` puts a signal only in bins
6..10 and requires it to reach the frame buffer. `apu_mixdown_two_bins` requires the
same signal to be *lost* with the switch off — without that arm, a mixdown
that ignored its own switch would pass.

`apu_savestate_roundtrip` also verifies that a paused APU snapshot restores the
exact PCM already queued to the host output device. That queue is part of the
APU timeline: dropping it on load would skip samples produced before the saved
APU state. The new queue data is covered by save-state format version 6; older
state files are intentionally rejected. The test reports the queued-PCM window
before and after restore in milliseconds; this measures pending host-queue audio,
not the full audible A/V offset or samples already in the device pipeline.
