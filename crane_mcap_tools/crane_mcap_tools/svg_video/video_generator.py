"""Video generator using ffmpeg.

このモジュールは、PNGフレームストリームまたはRAWフレームストリームをffmpegでMP4動画に変換します。
"""

import logging
import subprocess
from collections.abc import Iterator
from enum import Enum
from pathlib import Path

logger = logging.getLogger(__name__)


class InputFormat(Enum):
    """ffmpeg入力フォーマット."""

    PNG = "png"
    RAW_RGBA = "raw_rgba"


class VideoGenerator:
    """ffmpegを使用してPNGフレームまたはRAWフレームから動画を生成."""

    def __init__(
        self,
        fps: int = 30,
        codec: str = "libx264",
        crf: int = 23,
        pixel_format: str = "yuv420p",
        preset: str = "medium",
        input_format: InputFormat = InputFormat.PNG,
        width: int = 1920,
        height: int = 1080,
    ):
        """
        初期化.

        Args:
            fps: フレームレート
            codec: 動画コーデック
            crf: 品質設定（0-51、低いほど高品質）
            pixel_format: ピクセルフォーマット
            preset: エンコーディングプリセット（ultrafast, fast, medium, slow, veryslow）
            input_format: 入力フォーマット（PNG or RAW_RGBA）
            width: RAW入力の場合の画像幅（PNG入力では無視される）
            height: RAW入力の場合の画像高さ（PNG入力では無視される）
        """
        self.fps = fps
        self.codec = codec
        self.crf = crf
        self.pixel_format = pixel_format
        self.preset = preset
        self.input_format = input_format
        self.width = width
        self.height = height

        try:
            subprocess.run(
                ["ffmpeg", "-version"],
                capture_output=True,
                check=True,
            )
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            raise RuntimeError("ffmpeg is not installed or not in PATH") from e

    def generate(
        self,
        frames: Iterator[bytes],
        output_path: str | Path,
        verbose: bool = False,
    ) -> None:
        """
        フレームストリームから動画を生成.

        Args:
            frames: フレームバイト列のイテレータ（PNG or RAW RGBA）
            output_path: 出力動画ファイルパス
            verbose: 詳細ログを表示
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        if self.input_format == InputFormat.RAW_RGBA:
            cmd = [
                "ffmpeg",
                "-y",  # 出力ファイルを上書き
                "-f",
                "rawvideo",  # RAW入力
                "-pix_fmt",
                "rgba",  # 入力ピクセルフォーマット
                "-s",
                f"{self.width}x{self.height}",  # 入力サイズ
                "-r",
                str(self.fps),  # フレームレート
                "-i",
                "-",  # 標準入力から読み込み
                "-c:v",
                self.codec,  # 出力コーデック
                "-crf",
                str(self.crf),  # 品質
                "-pix_fmt",
                self.pixel_format,  # 出力ピクセルフォーマット
                "-preset",
                self.preset,  # プリセット
                str(output_path),
            ]
        else:
            cmd = [
                "ffmpeg",
                "-y",  # 出力ファイルを上書き
                "-f",
                "image2pipe",  # パイプ入力
                "-vcodec",
                "png",  # 入力コーデック
                "-r",
                str(self.fps),  # フレームレート
                "-i",
                "-",  # 標準入力から読み込み
                "-c:v",
                self.codec,  # 出力コーデック
                "-crf",
                str(self.crf),  # 品質
                "-pix_fmt",
                self.pixel_format,  # ピクセルフォーマット
                "-preset",
                self.preset,  # プリセット
                str(output_path),
            ]

        if verbose:
            logger.info(f"Running ffmpeg: {' '.join(cmd)}")

        # DEVNULL を使用してデッドロックを防ぐ
        process = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL if not verbose else None,
            stderr=subprocess.DEVNULL if not verbose else None,
        )

        try:
            frame_count = 0
            for frame_bytes in frames:
                if process.stdin:
                    process.stdin.write(frame_bytes)
                    frame_count += 1

                    if frame_count % 100 == 0:
                        logger.info(f"Processed {frame_count} frames...")

            if process.stdin:
                process.stdin.close()

            process.wait()

            if process.returncode != 0:
                raise RuntimeError(
                    f"ffmpeg failed with return code {process.returncode}. "
                    "Run with --verbose for details."
                )

            logger.info(
                f"Video generation completed: {output_path} ({frame_count} frames)"
            )

        except Exception:
            process.kill()
            raise

    def generate_from_directory(
        self,
        frames_dir: str | Path,
        output_path: str | Path,
        frame_pattern: str = "frame_%06d.png",
        verbose: bool = False,
    ) -> None:
        """
        ディレクトリ内のPNGファイルから動画を生成.

        Args:
            frames_dir: フレームディレクトリ
            output_path: 出力動画ファイルパス
            frame_pattern: フレームファイル名パターン
            verbose: 詳細ログを表示
        """
        frames_dir = Path(frames_dir)
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        cmd = [
            "ffmpeg",
            "-y",
            "-framerate",
            str(self.fps),
            "-i",
            str(frames_dir / frame_pattern),
            "-c:v",
            self.codec,
            "-crf",
            str(self.crf),
            "-pix_fmt",
            self.pixel_format,
            "-preset",
            self.preset,
            str(output_path),
        ]

        if verbose:
            logger.info(f"Running ffmpeg: {' '.join(cmd)}")

        try:
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL if not verbose else None,
                stderr=subprocess.DEVNULL if not verbose else None,
                check=True,
            )
            logger.info(f"Video generation completed: {output_path}")

        except subprocess.CalledProcessError as e:
            raise RuntimeError(
                f"ffmpeg failed with return code {e.returncode}. "
                "Run with --verbose for details."
            ) from e
