#!/usr/bin/env python3
# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""MCAP Annotation Analyzer - RoboCup SSL試合解析CLIツール.

MCAPファイルから人間によるアノテーションを抽出し、Gemini APIで解析、
Markdownレポートとして出力します。
"""

import argparse
import logging
import sys
from pathlib import Path

from crane_mcap_tools.mcap_analysis import (
    MCAPAnnotationExtractor,
    GeminiAnalysisClient,
    ReportGenerator,
)
from crane_mcap_tools.mcap_analysis.prompts import (
    SYSTEM_INSTRUCTION,
    create_annotation_analysis_prompt,
    format_position_info,
    format_robot_context,
)


def setup_logging(verbose: bool) -> None:
    """ロギング設定."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )


def main() -> int:
    """メイン処理."""
    parser = argparse.ArgumentParser(
        description="MCAP Annotation Analyzer - RoboCup SSL試合解析ツール",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  # 基本使用法
  %(prog)s /path/to/rosbag2_xxx/

  # オプション指定
  %(prog)s /path/to/rosbag2_xxx/ \\
      --output report.md \\
      --context-before 5.0 \\
      --context-after 3.0 \\
      --model gemini-2.0-flash-exp

  # Dry-run（API呼び出しなし）
  %(prog)s /path/to/rosbag2_xxx/ --dry-run

環境変数:
  GEMINI_API_KEY    Gemini APIキー（必須、--dry-run時は不要）
        """,
    )

    parser.add_argument(
        "mcap_path",
        type=str,
        help="MCAPファイルまたはrosbag2ディレクトリのパス",
    )

    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="mcap_analysis_report.md",
        help="出力レポートファイルパス（デフォルト: mcap_analysis_report.md）",
    )

    parser.add_argument(
        "--context-before",
        type=float,
        default=3.0,
        help="アノテーション前のコンテキスト時間（秒、デフォルト: 3.0）",
    )

    parser.add_argument(
        "--context-after",
        type=float,
        default=2.0,
        help="アノテーション後のコンテキスト時間（秒、デフォルト: 2.0）",
    )

    parser.add_argument(
        "--sampling-interval",
        type=int,
        default=100,
        help="WorldModelサンプリング間隔（ミリ秒、デフォルト: 100）",
    )

    parser.add_argument(
        "--model",
        type=str,
        default="gemini-2.0-flash-exp",
        help="使用するGeminiモデル（デフォルト: gemini-2.0-flash-exp）",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="API呼び出しなしでMCAP抽出のみ実行",
    )

    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="詳細ログを表示",
    )

    args = parser.parse_args()

    setup_logging(args.verbose)
    logger = logging.getLogger(__name__)

    # MCAPパスの検証
    mcap_path = Path(args.mcap_path)
    if not mcap_path.exists():
        logger.error(f"Path not found: {mcap_path}")
        return 1

    try:
        # Step 1: MCAP抽出
        logger.info("=" * 60)
        logger.info("Step 1: MCAPファイルからアノテーションを抽出")
        logger.info("=" * 60)

        extractor = MCAPAnnotationExtractor(
            context_before_sec=args.context_before,
            context_after_sec=args.context_after,
            world_model_sampling_ms=args.sampling_interval,
        )

        annotations = extractor.extract_from_mcap(mcap_path)

        if not annotations:
            logger.warning("アノテーションが見つかりませんでした。")
            return 0

        logger.info(f"✓ {len(annotations)}件のアノテーションを抽出しました")

        # Step 2: Gemini解析（dry-runでない場合）
        analysis_results = None

        if not args.dry_run:
            logger.info("")
            logger.info("=" * 60)
            logger.info("Step 2: Gemini APIで解析")
            logger.info("=" * 60)

            try:
                gemini_client = GeminiAnalysisClient(model=args.model)
            except ValueError as e:
                logger.error(f"Gemini APIキーが設定されていません: {e}")
                logger.error(
                    "環境変数GEMINI_API_KEYを設定するか、--dry-runを使用してください"
                )
                return 1

            # プロンプト生成
            prompts = []
            for ann in annotations:
                position_info = (
                    format_position_info(ann.position)
                    if ann.has_position and ann.position
                    else ""
                )
                robot_context = (
                    format_robot_context(ann.related_robot_ids, ann.robot_is_ours)
                    if ann.has_robot_context
                    else ""
                )

                prompt = create_annotation_analysis_prompt(
                    label=ann.label,
                    description=ann.description,
                    category=ann.get_category_name(),
                    priority=ann.get_priority_name(),
                    event_timestamp_ns=ann.event_timestamp_ns,
                    position_info=position_info,
                    robot_context=robot_context,
                )
                prompts.append((prompt, SYSTEM_INSTRUCTION))

            # バッチ解析（Function Calling有効）
            logger.info("Function Calling有効でGemini解析を実行します...")
            analysis_results = gemini_client.analyze_batch_with_tools(
                annotations, prompts, max_tool_calls=10
            )

            # エラーチェック
            error_count = sum(1 for r in analysis_results if r.error)
            if error_count > 0:
                logger.warning(f"⚠️ {error_count}件の解析でエラーが発生しました")

            logger.info(
                f"✓ {len(analysis_results) - error_count}件の解析が完了しました"
            )

        # Step 3: レポート生成
        logger.info("")
        logger.info("=" * 60)
        logger.info("Step 3: レポート生成")
        logger.info("=" * 60)

        report_gen = ReportGenerator(mcap_path)
        report = report_gen.generate_report(annotations, analysis_results)

        # レポート保存
        output_path = Path(args.output)
        report_gen.save_report(report, output_path)

        logger.info(f"✓ レポートを保存しました: {output_path.absolute()}")

        logger.info("")
        logger.info("=" * 60)
        logger.info("完了")
        logger.info("=" * 60)

        return 0

    except FileNotFoundError as e:
        logger.error(f"ファイルが見つかりません: {e}")
        return 1

    except ImportError as e:
        logger.error(f"必要なパッケージがインストールされていません: {e}")
        return 1

    except Exception as e:
        logger.exception(f"予期しないエラーが発生しました: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
