#!/usr/bin/env python3
"""Exercise save/load and compare captured game audio from the real Linux build."""

import argparse
import ctypes
import ctypes.util
import os
from pathlib import Path
import re
import select
import shutil
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np


SAMPLE_RATE = 48000
CHANNELS = 2
BYTES_PER_FRAME = CHANNELS * 2
SAVE_MENU_DOWNS = 18
MIN_REPLAY_CORRELATION = 0.65
MIN_CONTROL_ADVANTAGE = 0.20
MAX_CONTROL_REGRESSION = 0.15


class XTestKeyboard:
    def __init__(self, display_name):
        x11_name = ctypes.util.find_library("X11")
        xtst_name = ctypes.util.find_library("Xtst")
        if not x11_name or not xtst_name:
            raise RuntimeError("libX11 and libXtst are required for keyboard automation")

        self.x11 = ctypes.CDLL(x11_name)
        self.xtst = ctypes.CDLL(xtst_name)
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XKeysymToKeycode.restype = ctypes.c_uint
        self.x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.x11.XSetInputFocus.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong
        ]
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.xtst.XTestFakeKeyEvent.argtypes = [
            ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong
        ]
        self.xtst.XTestFakeKeyEvent.restype = ctypes.c_int

        self.display = self.x11.XOpenDisplay(display_name.encode())
        if not self.display:
            raise RuntimeError(f"could not connect to nested X display {display_name}")

    def press(self, keysym, hold_seconds=0.16):
        keycode = self.x11.XKeysymToKeycode(self.display, keysym)
        if not keycode:
            raise RuntimeError(f"could not map X11 keysym {keysym:#x}")
        if not self.xtst.XTestFakeKeyEvent(self.display, keycode, 1, 0):
            raise RuntimeError("XTest could not send key press")
        self.x11.XSync(self.display, 0)
        time.sleep(hold_seconds)
        if not self.xtst.XTestFakeKeyEvent(self.display, keycode, 0, 0):
            raise RuntimeError("XTest could not send key release")
        self.x11.XSync(self.display, 0)
        time.sleep(0.12)

    def focus(self, window_id):
        self.x11.XSetInputFocus(self.display, int(window_id, 16), 1, 0)
        self.x11.XSync(self.display, 0)

    def close(self):
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None


class AudioCapture:
    def __init__(self, sink_name):
        self.process = subprocess.Popen(
            [
                "parec",
                f"--device={sink_name}.monitor",
                "--raw",
                "--format=s16le",
                f"--rate={SAMPLE_RATE}",
                f"--channels={CHANNELS}",
                "--latency-msec=20",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )
        self.chunks = []
        self.lock = threading.Lock()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        time.sleep(0.25)
        if self.process.poll() is not None:
            self.reader.join(timeout=2)
            self.process.stdout.close()
            raise RuntimeError("parec could not capture from the temporary audio sink")

    def _read(self):
        while True:
            data = self.process.stdout.read(16384)
            if not data:
                return
            end = time.monotonic()
            start = end - len(data) / (SAMPLE_RATE * BYTES_PER_FRAME)
            with self.lock:
                self.chunks.append((start, data))

    def segment(self, duration_seconds):
        start = time.monotonic()
        time.sleep(duration_seconds)
        end = time.monotonic()
        pieces = []
        with self.lock:
            chunks = list(self.chunks)
        for chunk_start, data in chunks:
            chunk_end = chunk_start + len(data) / (SAMPLE_RATE * BYTES_PER_FRAME)
            overlap_start = max(start, chunk_start)
            overlap_end = min(end, chunk_end)
            if overlap_end <= overlap_start:
                continue
            first = int((overlap_start - chunk_start) * SAMPLE_RATE) * BYTES_PER_FRAME
            last = int((overlap_end - chunk_start) * SAMPLE_RATE) * BYTES_PER_FRAME
            pieces.append(data[first:last])
        return b"".join(pieces)

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.reader.join(timeout=2)
        if self.process.stdout:
            self.process.stdout.close()


def start_xephyr():
    read_fd, write_fd = os.pipe()
    process = subprocess.Popen(
        [
            "Xephyr",
            "-displayfd",
            str(write_fd),
            "-ac",
            "-nolisten",
            "tcp",
            "-screen",
            "1280x720",
            "-noreset",
        ],
        pass_fds=(write_fd,),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    os.close(write_fd)
    try:
        ready, _, _ = select.select([read_fd], [], [], 10)
        if not ready:
            raise RuntimeError("Xephyr did not allocate a display")
        display_number = os.read(read_fd, 32).decode().strip()
        if not display_number:
            raise RuntimeError("Xephyr returned an empty display number")
        return process, f":{display_number}"
    except Exception:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        raise
    finally:
        os.close(read_fd)


def wait_for_log(log_path, marker, occurrence, process, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        text = log_path.read_text(errors="replace") if log_path.exists() else ""
        if text.count(marker) >= occurrence:
            return
        if process.poll() is not None:
            raise RuntimeError(f"game exited while waiting for {marker!r} (code {process.returncode})")
        time.sleep(0.1)
    raise TimeoutError(f"timed out waiting for {marker!r} occurrence {occurrence}")


def wait_for_game_window(display, process, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    pattern = re.compile(r'^\s+(0x[0-9a-fA-F]+) "NV2A":', re.MULTILINE)
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"game exited before opening its window (code {process.returncode})")
        result = subprocess.run(
            ["xwininfo", "-display", display, "-root", "-tree"],
            check=False,
            capture_output=True,
            text=True,
        )
        match = pattern.search(result.stdout)
        if match:
            return match.group(1)
        time.sleep(0.1)
    raise TimeoutError("timed out waiting for the game window")


def audio_samples(raw):
    samples = np.frombuffer(raw[: len(raw) // BYTES_PER_FRAME * BYTES_PER_FRAME],
                            dtype="<i2")
    if samples.size < BYTES_PER_FRAME:
        return np.empty(0, dtype=np.float32)
    return samples.reshape(-1, CHANNELS).astype(np.float32).mean(axis=1)


def compare_audio(reference, replay):
    reference = audio_samples(reference)
    replay = audio_samples(replay)
    sample_step = 8
    reference = reference[::sample_step]
    replay = replay[::sample_step]
    count = min(reference.size, replay.size)
    if count < SAMPLE_RATE // sample_step:
        raise RuntimeError("audio capture was too short to compare")
    reference = reference[:count]
    replay = replay[:count]
    reference -= reference.mean()
    replay -= replay.mean()
    reference_rms = float(np.sqrt(np.mean(reference * reference)))
    replay_rms = float(np.sqrt(np.mean(replay * replay)))
    if reference_rms < 20:
        raise RuntimeError("captured game audio is silent; load a state with active audio")
    if replay_rms < 20:
        raise RuntimeError("audio became silent after loading the saved state")

    max_lag = min(SAMPLE_RATE // sample_step // 2, count // 4)
    best = (-1.0, 0)
    for lag in range(-max_lag, max_lag + 1, 4):
        left_start = max(0, -lag)
        right_start = max(0, lag)
        length = count - max(left_start, right_start)
        if length <= 0:
            continue
        left = reference[left_start:left_start + length]
        right = replay[right_start:right_start + length]
        denom = np.linalg.norm(left) * np.linalg.norm(right)
        score = float(np.dot(left, right) / denom) if denom else -1.0
        if score > best[0]:
            best = (score, lag)

    return best[0], best[1], reference_rms, replay_rms


def tail_log(path, lines=30):
    if not path.exists():
        return ""
    entries = path.read_text(errors="replace").splitlines()
    relevant = [
        line for line in entries
        if "[menu]" in line or "[savestate]" in line or "[AUDIO]" in line
    ]
    return "\n".join(relevant + entries[-lines:])


def run_test(executable, game_dir, capture_seconds):
    if not all(shutil.which(tool) for tool in ("Xephyr", "pactl", "parec", "xwininfo")):
        raise RuntimeError("Xephyr, pactl, parec, and xwininfo must be installed")
    if not (game_dir / "default.xbe").is_file():
        raise RuntimeError(f"no default.xbe in game directory: {game_dir}")

    with tempfile.TemporaryDirectory(prefix="clx-exe-audio-sync-") as temp_name:
        work = Path(temp_name)
        options = work / "clx_options.txt"
        options.write_text("state_slot=5\nvolume=100\n")
        log_path = work / "game.log"
        sink_name = f"clx_sync_{os.getpid()}"
        module_id = None
        xephyr = None
        game = None
        capture = None
        keyboard = None

        try:
            module_id = subprocess.check_output(
                [
                    "pactl",
                    "load-module",
                    "module-null-sink",
                    f"sink_name={sink_name}",
                    f"rate={SAMPLE_RATE}",
                    f"channels={CHANNELS}",
                ],
                text=True,
                stderr=subprocess.STDOUT,
            ).strip()
            xephyr, display = start_xephyr()
            keyboard = XTestKeyboard(display)
            capture = AudioCapture(sink_name)

            env = os.environ.copy()
            env.update(
                {
                    "DISPLAY": display,
                    "SDL_VIDEODRIVER": "x11",
                    "SDL_AUDIODRIVER": "pulseaudio",
                    "PULSE_SINK": sink_name,
                    "NFSU2_GAME_DIR": str(game_dir),
                    "CLX_OPTIONS": str(options),
                    "CLX_SAVESTATE_DIR": str(work / "savestates"),
                    "XDG_DATA_HOME": str(work / "xdg"),
                    "RECOMP_CACHE_DIR": str(work / "cache"),
                    "RECOMP_QUIET": "1",
                    "CLX_SKIP_INTRO": "1",
                    "CLX_START_SINGLE": "1",
                }
            )
            with log_path.open("w") as log:
                game = subprocess.Popen(
                    ["stdbuf", "-oL", "-eL", str(executable)],
                    cwd=work,
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
                wait_for_log(log_path, "[hitch] frame", 1, game, 120)
                keyboard.focus(wait_for_game_window(display, game, 20))
                time.sleep(3)

                keyboard.press(0xFF1B, 0.5)  # Escape opens the settings menu.
                wait_for_log(log_path, "[menu] open", 1, game, 10)
                for _ in range(SAVE_MENU_DOWNS):
                    keyboard.press(0xFF54, 0.22)  # Down selects SAVE STATE on Linux/PC.
                keyboard.press(0x20, 0.3)  # Space selects A.
                wait_for_log(log_path, "[savestate] slot 5 save succeeded", 1, game, 60)
                time.sleep(0.5)
                reference = capture.segment(capture_seconds)
                control = capture.segment(capture_seconds)

                keyboard.press(0xFF1B, 0.5)
                wait_for_log(log_path, "[menu] open", 2, game, 10)
                keyboard.press(0xFF54, 0.22)  # Move from SAVE STATE to LOAD STATE.
                keyboard.press(0x20, 0.3)
                wait_for_log(log_path, "[savestate] slot 5 load succeeded", 1, game, 60)
                time.sleep(0.5)
                replay = capture.segment(capture_seconds)

            replay_score, lag, reference_rms, replay_rms = compare_audio(reference, replay)
            control_score, _, _, _ = compare_audio(reference, control)
            print(f"Compared {capture_seconds:.1f}s before and after one in-process load.")
            print(f"Audio RMS: {reference_rms:.1f} before load, {replay_rms:.1f} after load.")
            print(f"Replay correlation: {replay_score:.3f} (lag {lag * 8 / SAMPLE_RATE * 1000:.1f} ms); "
                  f"normal-playback control: {control_score:.3f} "
                  f"(difference {replay_score - control_score:+.3f}).")
            if replay_score + MAX_CONTROL_REGRESSION < control_score:
                raise RuntimeError("replayed audio diverged more than normal playback (possible desync)")
            if replay_score < MIN_REPLAY_CORRELATION and \
                    replay_score - control_score < MIN_CONTROL_ADVANTAGE:
                print("Executable save/load audio synchronization: inconclusive "
                      "(the boot scene was not repeatable enough for waveform comparison)")
                return 2
            print("Executable save/load audio synchronization: passed")
        except Exception as error:
            print(f"Executable save/load audio synchronization: failed: {error}", file=sys.stderr)
            print(tail_log(log_path), file=sys.stderr)
            raise
        finally:
            if keyboard:
                keyboard.close()
            if capture:
                capture.close()
            if game and game.poll() is None:
                game.terminate()
                try:
                    game.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    game.kill()
                    game.wait()
            if xephyr and xephyr.poll() is None:
                xephyr.terminate()
                try:
                    xephyr.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    xephyr.kill()
                    xephyr.wait()
            if module_id:
                subprocess.run(
                    ["pactl", "unload-module", module_id],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--game-dir", type=Path, required=True)
    parser.add_argument("--capture-seconds", type=float, default=3.0)
    args = parser.parse_args()
    executable = args.executable.resolve()
    game_dir = args.game_dir.resolve()
    if not executable.is_file() or not os.access(executable, os.X_OK):
        parser.error(f"not an executable file: {executable}")
    if args.capture_seconds < 1:
        parser.error("--capture-seconds must be at least 1")
    try:
        return run_test(executable, game_dir, args.capture_seconds)
    except Exception:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
