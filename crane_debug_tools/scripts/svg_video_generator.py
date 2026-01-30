#!/usr/bin/env python3
"""SVG video generator from MCAP rosbag files.

このスクリプトは、MCAPファイルからSVGメッセージを読み込み、動画を生成します。
"""

import argparse
import logging
import sys
from pathlib import Path

from crane_debug_tools.svg_video import (
    SvgAssembler,
    SvgExtractor,
    SvgRenderer,
    VideoGenerator,
)


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
    return set(layer.strip() for layer in layer_str.split(",") if layer.strip())


def main() -> int:
    """メイン処理."""
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
        """,
    )

    # 必須引数
    parser.add_argument("mcap_path", help="Path to MCAP file or rosbag2 directory")

    # 出力設定
    parser.add_argument(
        "-o",
        "--output",
        default="output.mp4",
        help="Output video file path (default: output.mp4)",
    )

    # 動画設定
    parser.add_argument(
        "--fps", type=int, default=30, help="Output frame rate (default: 30)"
    )
    parser.add_argument(
        "--width", type=int, default=1920, help="Output width in pixels (default: 1920)"
    )
    parser.add_argument(
        "--height",
        type=int,
        default=1080,
        help="Output height in pixels (default: 1080)",
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

    # 時刻範囲設定
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

    # レイヤー設定
    parser.add_argument(
        "--layers", help="Comma-separated list of layers to display (default: all)"
    )
    parser.add_argument(
        "--exclude-layers",
        help="Comma-separated list of layers to exclude (default: none)",
    )

    # トピック設定
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

    # デバッグ設定
    parser.add_argument(
        "--save-frames", help="Save PNG frames to directory (for debugging)"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose logging"
    )

    args = parser.parse_args()

    # ロギング設定
    setup_logging(args.verbose)
    logger = logging.getLogger(__name__)

    try:
        # 入力ファイルの確認
        mcap_path = Path(args.mcap_path)
        if not mcap_path.exists():
            logger.error(f"MCAP path does not exist: {mcap_path}")
            return 1

        # レイヤーフィルタリング設定
        visible_layers = parse_layer_list(args.layers)
        exclude_layers = parse_layer_list(args.exclude_layers)

        # コンポーネント初期化
        logger.info("Initializing components...")
        extractor = SvgExtractor(
            snapshot_topic=args.snapshot_topic, update_topic=args.update_topic
        )
        assembler = SvgAssembler()
        renderer = SvgRenderer(width=args.width, height=args.height)
        video_gen = VideoGenerator(fps=args.fps, crf=args.crf, preset=args.preset)

        # フレーム生成
        logger.info("Extracting SVG frames from MCAP...")

        def generate_png_frames():
            """PNGフレームを生成（固定フレームレート）."""
            # 全フレーム状態を収集
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

            # 時刻範囲を取得
            start_time_ns = all_frames[0].timestamp_ns
            end_time_ns = all_frames[-1].timestamp_ns
            duration_sec = (end_time_ns - start_time_ns) / 1e9

            # 目標フレーム数を計算（固定フレームレート）
            target_frame_count = int(duration_sec * args.fps / args.speed)

            if target_frame_count == 0:
                logger.warning("Duration too short or invalid fps/speed")
                return

            frame_interval_ns = int((end_time_ns - start_time_ns) / target_frame_count)

            logger.info(
                f"Duration: {duration_sec:.2f}s, Target frames: {target_frame_count} "
                f"(from {len(all_frames)} SVG states)"
            )

            # 固定フレームレートでサンプリング
            # キャッシュ変数（同一フレームの重複レンダリングを防止）
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
                    # フレーム保存（デバッグ用）
                    if args.save_frames:
                        frames_dir = Path(args.save_frames)
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

                # 新しいフレームの処理
                svg_frame = all_frames[frame_idx]

                # レイヤーフィルタリング
                layers_to_render = svg_frame.layers
                if visible_layers is not None:
                    effective_visible = visible_layers
                elif exclude_layers is not None:
                    effective_visible = set(layers_to_render.keys()) - exclude_layers
                else:
                    effective_visible = None

                # SVG組み立て
                svg_string = assembler.assemble(
                    layers_to_render, visible_layers=effective_visible
                )

                # PNG変換
                cached_png = renderer.render(svg_string)
                last_frame_idx = frame_idx

                # フレーム保存（デバッグ用）
                if args.save_frames:
                    frames_dir = Path(args.save_frames)
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

        # 動画生成
        if args.save_frames:
            # フレーム保存モード: ジェネレータを実行してフレームを保存
            logger.info(f"Saving frames to: {args.save_frames}")
            list(generate_png_frames())  # ジェネレータを実行

            # 保存したフレームから動画生成
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
                png_frames=generate_png_frames(),
                output_path=args.output,
                verbose=args.verbose,
            )

        logger.info("Done!")
        return 0

    except KeyboardInterrupt:
        logger.warning("Interrupted by user")
        return 130
    except Exception as e:
        logger.exception(f"Error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
