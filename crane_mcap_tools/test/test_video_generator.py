# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""VideoGenerator が組み立てる ffmpeg コマンドを固定する特性テスト（ffmpeg は起動しない）."""

import io
import subprocess

import pytest
from crane_mcap_tools.svg_video import video_generator
from crane_mcap_tools.svg_video.video_generator import InputFormat, VideoGenerator

OUTPUT_ARGS = [
    "-c:v",
    "libx265",
    "-crf",
    "18",
    "-pix_fmt",
    "yuv444p",
    "-preset",
    "fast",
]


class FakePopen:
    """argv と stdin への書き込みを記録する Popen の代役（instances と exit_code は fixture が設定する）."""

    def __init__(self, cmd, stdin=None, stdout=None, stderr=None):
        self.cmd = cmd
        self.stdin = io.BytesIO()
        self.written = b""
        self.returncode = None
        self.killed = False
        original_close = self.stdin.close

        def close():
            self.written = self.stdin.getvalue()
            original_close()

        self.stdin.close = close
        FakePopen.instances.append(self)

    def wait(self):
        self.returncode = FakePopen.exit_code

    def kill(self):
        self.killed = True


@pytest.fixture
def ffmpeg(monkeypatch):
    run_calls = []
    FakePopen.instances = []
    FakePopen.exit_code = 0

    def fake_run(cmd, **kwargs):
        run_calls.append(cmd)
        return subprocess.CompletedProcess(cmd, 0)

    monkeypatch.setattr(video_generator.subprocess, "run", fake_run)
    monkeypatch.setattr(video_generator.subprocess, "Popen", FakePopen)
    return run_calls


def make_generator(input_format):
    return VideoGenerator(
        fps=60,
        codec="libx265",
        crf=18,
        pixel_format="yuv444p",
        preset="fast",
        input_format=input_format,
        width=640,
        height=480,
    )


def test_constructor_probes_ffmpeg(ffmpeg):
    make_generator(InputFormat.PNG)

    assert ffmpeg == [["ffmpeg", "-version"]]


def test_missing_ffmpeg_raises_runtime_error(monkeypatch):
    def missing(cmd, **kwargs):
        raise FileNotFoundError(cmd[0])

    monkeypatch.setattr(video_generator.subprocess, "run", missing)

    with pytest.raises(RuntimeError, match="ffmpeg is not installed"):
        VideoGenerator()


def test_png_stream_command_and_frames(ffmpeg, tmp_path):
    out = tmp_path / "sub" / "out.mp4"

    make_generator(InputFormat.PNG).generate(iter([b"a", b"bc"]), out)

    (process,) = FakePopen.instances
    assert process.cmd == [
        "ffmpeg",
        "-y",
        "-f",
        "image2pipe",
        "-vcodec",
        "png",
        "-r",
        "60",
        "-i",
        "-",
        *OUTPUT_ARGS,
        str(out),
    ]
    assert process.written == b"abc"
    assert out.parent.is_dir()


def test_raw_rgba_stream_command(ffmpeg, tmp_path):
    out = tmp_path / "out.mp4"

    make_generator(InputFormat.RAW_RGBA).generate(iter([b"x"]), out)

    (process,) = FakePopen.instances
    assert process.cmd == [
        "ffmpeg",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgba",
        "-s",
        "640x480",
        "-r",
        "60",
        "-i",
        "-",
        *OUTPUT_ARGS,
        str(out),
    ]


def test_ffmpeg_failure_kills_process_and_raises(ffmpeg, tmp_path):
    FakePopen.exit_code = 1

    with pytest.raises(RuntimeError, match="return code 1"):
        make_generator(InputFormat.PNG).generate(iter([b"a"]), tmp_path / "o.mp4")

    assert FakePopen.instances[0].killed


def test_directory_command(ffmpeg, tmp_path):
    frames_dir = tmp_path / "frames"
    out = tmp_path / "out.mp4"

    make_generator(InputFormat.PNG).generate_from_directory(frames_dir, out)

    assert ffmpeg[1] == [
        "ffmpeg",
        "-y",
        "-framerate",
        "60",
        "-i",
        str(frames_dir / "frame_%06d.png"),
        *OUTPUT_ARGS,
        str(out),
    ]
