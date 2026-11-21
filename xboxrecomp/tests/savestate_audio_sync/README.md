# Executable save-state audio test

This opt-in Linux integration test launches the built game executable with an isolated
configuration, save directory, cache, X11 display, and PulseAudio null sink. It saves to
slot 5, captures the actual SDL audio output, loads that state once in the same process,
then compares the saved-state playback with the replay after aligning for host output
latency. A second consecutive capture serves as a control for natural audio variation in
the current game scene. If that scene is too variable to support a waveform comparison,
the test reports inconclusive rather than mislabeling it as a desync.

The test needs a Linux build, the user's unpacked game data, `Xephyr`, `pactl`, `parec`,
`libX11`, `libXtst`, and Python with NumPy. It does not modify the supplied game folder or
existing save-state files.

```sh
python3 xboxrecomp/tests/savestate_audio_sync/test_executable.py \
  --executable out/work/build-linux/clx_recomp \
  --game-dir /path/to/unpacked/conker-disc
```

The default correlation threshold detects substantial waveform divergence, not every
perceptible audio artifact. A passing result is a regression signal for this captured
scene, not proof that all levels, host audio devices, or restart-load behavior are correct.
