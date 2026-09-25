#!/usr/bin/env python3
"""SVG video generator from MCAP rosbag files.

このスクリプトは、MCAPファイルからSVGメッセージを読み込み、動画を生成します。
"""

import argparse
import logging
import multiprocessing
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from crane_mcap_tools.svg_video import (
    SvgAssembler,
    SvgExtractor,
    VideoGenerator,
    create_renderer,
    list_available_backends,
)
from crane_mcap_tools.svg_video.renderers import OutputFormat
from crane_mcap_tools.svg_video.video_generator import InputFormat


def setup_logging(verbose: bool) -> None:
    """ロギングを設定."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def parse_layer_list(layer_str: str | None) -> set[str] | None:
    """レイヤーリスト文字列をパース."""
    if layer_str is None:
        return None
    return {layer.strip() for layer in layer_str.split(",") if layer.strip()}


_renderer_backend = None
_renderer_output_format = None


def init_worker(backend: str | None, output_format: OutputFormat):
    """ワーカープロセスの初期化関数."""
    global _renderer_backend, _renderer_output_format
    _renderer_backend = backend
    _renderer_output_format = output_format


def render_frame_worker(args: tuple[str, int, int, int]) -> bytes:
    """
    ワーカープロセスで実行されるフレームレンダリング関数.

    Args:
        args: (svg_string, width, height, dpi)のタプル

    Returns:
        画像バイト列
    """
    svg_string, width, height, dpi = args

    renderer = create_renderer(
        backend=_renderer_backend,
        width=width,
        height=height,
        dpi=dpi,
        output_format=_renderer_output_format,
    )

    return renderer.render(svg_string)


def main() -> int:
    """メイン処理."""
    QUALITY_PRESETS = {
        "low": {"width": 1280, "height": 720},
        "medium": {"width": 1920, "height": 1080},
        "high": {"width": 1920, "height": 1080},
    }

    parser = argparse.ArgumentParser(
        description="Generate MP4 video from MCAP rosbag SVG topics",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # 基本的な使い方
  %(prog)s /path/to/rosbag_dir -o output.mp4

  # フレームレート60fps、2倍速再生
  %(prog)s /path/to/rosbag_dir -o output.mp4 --fps 60 --speed 2.0

  # 特定のレイヤーのみ表示
  %(prog)s /path/to/rosbag_dir -o output.mp4 --layers "layer1,layer2,layer3"

  # フレームをディレクトリに保存
  %(prog)s /path/to/rosbag_dir -o output.mp4 --save-frames frames_dir

  # 低解像度（720p）で高速生成
  %(prog)s /path/to/rosbag_dir -o output.mp4 --quality low

  # 並列処理のワーカー数を指定
  %(prog)s /path/to/rosbag_dir -o output.mp4 --workers 8
        """,
    )

    parser.add_argument("mcap_path", help="Path to MCAP file or rosbag2 directory")

    parser.add_argument(
        "-o",
        "--output",
        default="output.mp4",
        help="Output video file path (default: output.mp4)",
    )

    parser.add_argument(
        "--fps", type=int, default=30, help="Output frame rate (default: 30)"
    )
    parser.add_argument(
        "--quality",
        choices=["low", "medium", "high"],
        default="high",
        help="Video quality preset: low=720p, medium=1080p, high=1080p (default: high)",
    )
    parser.add_argument(
        "--width", type=int, help="Output width in pixels (overrides --quality)"
    )
    parser.add_argument(
        "--height",
        type=int,
        help="Output height in pixels (overrides --quality)",
    )
    parser.add_argument(
        "--crf",
        type=int,
        default=23,
        help="Video quality (0-51, lower is better, default: 23)",
    )
    parser.add_argument(
        "--preset",
        default="medium",
        choices=["ultrafast", "fast", "medium", "slow", "veryslow"],
        help="Encoding preset (default: medium)",
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (default: 1.0)",
    )
    parser.add_argument(
        "--start-time", type=float, help="Start time in seconds (from bag start)"
    )
    parser.add_argument(
        "--end-time", type=float, help="End time in seconds (from bag start)"
    )

    parser.add_argument(
        "--layers", help="Comma-separated list of layers to display (default: all)"
    )
    parser.add_argument(
        "--exclude-layers",
        help="Comma-separated list of layers to exclude (default: none)",
    )

    parser.add_argument(
        "--snapshot-topic",
        default="/aggregated_svgs",
        help="Snapshot topic name (default: /aggregated_svgs)",
    )
    parser.add_argument(
        "--update-topic",
        default="/visualizer_svgs",
        help="Update topic name (default: /visualizer_svgs)",
    )

    parser.add_argument(
        "--workers",
        type=int,
        default=0,
        help="Number of parallel workers (0=auto, 1=disable parallelization, default: 0)",
    )
    parser.add_argument(
        "--backend",
        choices=["auto", "resvg", "cairosvg"],
        default="auto",
        help="SVG rendering backend (default: auto - use fastest available)",
    )
    parser.add_argument(
        "--list-backends",
        action="store_true",
        help="List available rendering backends and exit",
    )
    parser.add_argument(
        "--raw-frames",
        action="store_true",
        help="Use RAW RGBA frames instead of PNG (faster but uses more memory)",
    )

    parser.add_argument(
        "--save-frames", help="Save PNG frames to directory (for debugging)"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose logging"
    )

    # バックエンド一覧表示（早期チェック）
    if "--list-backends" in sys.argv:
        args = parser.parse_args(["dummy_path", "--list-backends"])
    else:
        args = parser.parse_args()

    if args.list_backends:
        print("Available SVG rendering backends:")
        print()
        for name, available, description in list_available_backends():
            status = "✓ Available" if available else "✗ Not installed"
            print(f"  {name:12} {status:15} - {description}")
        print()
        return 0

    setup_logging(args.verbose)
    logger = logging.getLogger(__name__)

    try:
        mcap_path = Path(args.mcap_path)
        if not mcap_path.exists():
            logger.error(f"MCAP path does not exist: {mcap_path}")
            return 1

        if args.width is not None and args.height is not None:
            width = args.width
            height = args.height
        elif args.width is not None or args.height is not None:
            logger.error("Both --width and --height must be specified together")
            return 1
        else:
            preset = QUALITY_PRESETS[args.quality]
            width = preset["width"]
            height = preset["height"]

        logger.info(f"Output resolution: {width}x{height}")

        if args.workers == 0:
            num_workers = multiprocessing.cpu_count()
        else:
            num_workers = args.workers

        logger.info(
            f"Parallel workers: {num_workers} ({'disabled' if num_workers == 1 else 'enabled'})"
        )

        visible_layers = parse_layer_list(args.layers)
        exclude_layers = parse_layer_list(args.exclude_layers)

        if args.raw_frames:
            output_format = OutputFormat.RAW_RGBA
            input_format = InputFormat.RAW_RGBA
            logger.info("Using RAW RGBA frames (higher memory usage, faster)")
        else:
            output_format = OutputFormat.PNG
            input_format = InputFormat.PNG
            logger.info("Using PNG frames (lower memory usage)")

        logger.info("Initializing components...")
        extractor = SvgExtractor(
            snapshot_topic=args.snapshot_topic, update_topic=args.update_topic
        )
        assembler = SvgAssembler()

        backend = args.backend if args.backend != "auto" else None
        renderer = create_renderer(
            backend=backend, width=width, height=height, output_format=output_format
        )

        video_gen = VideoGenerator(
            fps=args.fps,
            crf=args.crf,
            preset=args.preset,
            input_format=input_format,
            width=width,
            height=height,
        )

        logger.info("Extracting SVG frames from MCAP...")

        def generate_png_frames():
            """PNGフレームを生成（固定フレームレート、並列処理対応）."""
            logger.info("Loading all SVG states from MCAP...")
            all_frames = list(
                extractor.extract_from_mcap(
                    mcap_path,
                    start_time_sec=args.start_time,
                    end_time_sec=args.end_time,
                )
            )

            if not all_frames:
                logger.warning("No frames found in MCAP")
                return

            start_time_ns = all_frames[0].timestamp_ns
            end_time_ns = all_frames[-1].timestamp_ns
            duration_sec = (end_time_ns - start_time_ns) / 1e9

            target_frame_count = int(duration_sec * args.fps / args.speed)

            if target_frame_count == 0:
                logger.warning("Duration too short or invalid fps/speed")
                return

            frame_interval_ns = int((end_time_ns - start_time_ns) / target_frame_count)

            logger.info(
                f"Duration: {duration_sec:.2f}s, Target frames: {target_frame_count} "
                f"(from {len(all_frames)} SVG states)"
            )

            if num_workers == 1:
                yield from _generate_frames_sequential(
                    all_frames,
                    target_frame_count,
                    start_time_ns,
                    frame_interval_ns,
                    assembler,
                    renderer,
                    visible_layers,
                    exclude_layers,
                    args.save_frames,
                    logger,
                )
            else:
                yield from _generate_frames_parallel(
                    all_frames,
                    target_frame_count,
                    start_time_ns,
                    frame_interval_ns,
                    assembler,
                    width,
                    height,
                    num_workers,
                    visible_layers,
                    exclude_layers,
                    args.save_frames,
                    logger,
                )

        def _generate_frames_sequential(
            all_frames,
            target_frame_count,
            start_time_ns,
            frame_interval_ns,
            assembler,
            renderer,
            visible_layers,
            exclude_layers,
            save_frames_dir,
            logger,
        ):
            """シーケンシャルなフレーム生成."""
            frame_idx = 0
            last_frame_idx = -1
            cached_png = None

            for i in range(target_frame_count):
                target_time_ns = start_time_ns + i * frame_interval_ns

                # target_time_ns以下で最も近いフレームを見つける
                while (
                    frame_idx < len(all_frames) - 1
                    and all_frames[frame_idx + 1].timestamp_ns <= target_time_ns
                ):
                    frame_idx += 1

                # 同じフレームならキャッシュを再利用
                if frame_idx == last_frame_idx and cached_png is not None:
                    if save_frames_dir:
                        frames_dir = Path(save_frames_dir)
                        frames_dir.mkdir(parents=True, exist_ok=True)
                        frame_path = frames_dir / f"frame_{i:06d}.png"
                        frame_path.write_bytes(cached_png)

                    yield cached_png

                    if (i + 1) % 100 == 0:
                        progress = (i + 1) / target_frame_count * 100
                        logger.info(
                            f"Generated {i + 1}/{target_frame_count} frames ({progress:.1f}%)"
                        )
                    continue

                svg_frame = all_frames[frame_idx]

                layers_to_render = svg_frame.layers
                if visible_layers is not None:
                    effective_visible = visible_layers
                elif exclude_layers is not None:
                    effective_visible = set(layers_to_render.keys()) - exclude_layers
                else:
                    effective_visible = None

                svg_string = assembler.assemble(
                    layers_to_render, visible_layers=effective_visible
                )

                cached_png = renderer.render(svg_string)
                last_frame_idx = frame_idx

                if save_frames_dir:
                    frames_dir = Path(save_frames_dir)
                    frames_dir.mkdir(parents=True, exist_ok=True)
                    frame_path = frames_dir / f"frame_{i:06d}.png"
                    frame_path.write_bytes(cached_png)

                yield cached_png

                if (i + 1) % 100 == 0:
                    progress = (i + 1) / target_frame_count * 100
                    logger.info(
                        f"Generated {i + 1}/{target_frame_count} frames ({progress:.1f}%)"
                    )

            logger.info(f"Total frames generated: {target_frame_count}")

        def _generate_frames_parallel(
            all_frames,
            target_frame_count,
            start_time_ns,
            frame_interval_ns,
            assembler,
            width,
            height,
            num_workers,
            visible_layers,
            exclude_layers,
            save_frames_dir,
            logger,
        ):
            """並列処理でのフレーム生成."""
            batch_size = num_workers * 2

            # SVG文字列の事前準備（メインプロセスで実行）
            svg_strings = []
            frame_idx = 0
            last_frame_idx = -1
            last_svg_idx = -1

            logger.info("Preparing SVG strings for parallel rendering...")

            for i in range(target_frame_count):
                target_time_ns = start_time_ns + i * frame_interval_ns

                # target_time_ns以下で最も近いフレームを見つける
                while (
                    frame_idx < len(all_frames) - 1
                    and all_frames[frame_idx + 1].timestamp_ns <= target_time_ns
                ):
                    frame_idx += 1

                # 同じフレームなら再利用（インデックス参照）
                if frame_idx == last_frame_idx:
                    svg_strings.append(last_svg_idx)
                else:
                    svg_frame = all_frames[frame_idx]

                    layers_to_render = svg_frame.layers
                    if visible_layers is not None:
                        effective_visible = visible_layers
                    elif exclude_layers is not None:
                        effective_visible = (
                            set(layers_to_render.keys()) - exclude_layers
                        )
                    else:
                        effective_visible = None

                    svg_string = assembler.assemble(
                        layers_to_render, visible_layers=effective_visible
                    )
                    svg_strings.append(svg_string)
                    last_frame_idx = frame_idx
                    last_svg_idx = len(svg_strings) - 1

            logger.info("SVG strings prepared. Starting parallel rendering...")

            with ProcessPoolExecutor(
                max_workers=num_workers,
                initializer=init_worker,
                initargs=(backend, output_format),
            ) as executor:
                batch_args = []
                batch_indices = []
                result_index = 0

                for i, svg_ref in enumerate(svg_strings):
                    # インデックス参照の場合は実際のSVG文字列を取得
                    if isinstance(svg_ref, int):
                        svg_string = svg_strings[svg_ref]
                    else:
                        svg_string = svg_ref

                    batch_args.append((svg_string, width, height, 96))
                    batch_indices.append(i)

                    if len(batch_args) >= batch_size:
                        results = list(executor.map(render_frame_worker, batch_args))

                        for png_bytes, idx in zip(results, batch_indices):
                            if save_frames_dir:
                                frames_dir = Path(save_frames_dir)
                                frames_dir.mkdir(parents=True, exist_ok=True)
                                frame_path = frames_dir / f"frame_{idx:06d}.png"
                                frame_path.write_bytes(png_bytes)

                            yield png_bytes
                            result_index += 1

                            if result_index % 100 == 0:
                                progress = result_index / target_frame_count * 100
                                logger.info(
                                    f"Generated {result_index}/{target_frame_count} frames ({progress:.1f}%)"
                                )

                        batch_args = []
                        batch_indices = []

                # 残りのフレームを処理
                if batch_args:
                    results = list(executor.map(render_frame_worker, batch_args))

                    for png_bytes, idx in zip(results, batch_indices):
                        if save_frames_dir:
                            frames_dir = Path(save_frames_dir)
                            frames_dir.mkdir(parents=True, exist_ok=True)
                            frame_path = frames_dir / f"frame_{idx:06d}.png"
                            frame_path.write_bytes(png_bytes)

                        yield png_bytes
                        result_index += 1

            logger.info(f"Total frames generated: {target_frame_count}")

        if args.save_frames:
            if args.raw_frames:
                logger.error(
                    "--save-frames is not compatible with --raw-frames. "
                    "Use PNG mode to save frames."
                )
                return 1

            logger.info(f"Saving frames to: {args.save_frames}")
            list(generate_png_frames())  # ジェネレータを実行

            logger.info(f"Generating video from saved frames: {args.output}")
            video_gen.generate_from_directory(
                frames_dir=args.save_frames,
                output_path=args.output,
                verbose=args.verbose,
            )
        else:
            # ストリーミングモード: メモリ効率重視
            logger.info(f"Generating video: {args.output}")
            video_gen.generate(
                frames=generate_png_frames(),
                output_path=args.output,
                verbose=args.verbose,
            )

        logger.info("Done!")
        return 0

    except KeyboardInterrupt:
        logger.warning("Interrupted by user")
        return 130
    except Exception:
        logger.exception("Error")
        return 1


if __name__ == "__main__":
    sys.exit(main())
