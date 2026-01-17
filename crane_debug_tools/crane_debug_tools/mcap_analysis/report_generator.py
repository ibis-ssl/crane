"""Markdown report generator for annotation analysis.

このモジュールは、解析結果をMarkdownレポートとして出力します。
"""

import logging
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any

from .extractor import AnnotationContext
from .gemini_client import AnalysisResult

logger = logging.getLogger(__name__)


class ReportGenerator:
    """Markdownレポート生成クラス."""

    def __init__(self, mcap_path: str | Path):
        """
        初期化.

        Args:
            mcap_path: 解析対象のMCAPファイルパス
        """
        self.mcap_path = Path(mcap_path)

    def generate_report(
        self,
        annotations: list[AnnotationContext],
        analysis_results: list[AnalysisResult] | None = None,
    ) -> str:
        """
        Markdownレポートを生成.

        Args:
            annotations: アノテーションコンテキストのリスト
            analysis_results: Gemini解析結果のリスト（Noneの場合はdry-run）

        Returns:
            Markdownレポート文字列
        """
        is_dry_run = analysis_results is None

        # ヘッダー
        lines = [
            "# MCAPアノテーション解析レポート",
            "",
            "## 概要",
            "",
            f"- **ファイル**: `{self.mcap_path.name}`",
            f"- **解析日時**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"- **アノテーション数**: {len(annotations)}",
        ]

        if is_dry_run:
            lines.append("- **モード**: Dry-run（Gemini API呼び出しなし）")
        else:
            lines.append(f"- **解析済み**: {len(analysis_results)}件")

        lines.append("")

        # 統計情報
        lines.extend(self._generate_statistics(annotations))

        if not is_dry_run and analysis_results:
            # サマリー（重要な発見）
            lines.extend(self._generate_summary(annotations, analysis_results))

        # カテゴリ別分析
        lines.extend(
            self._generate_category_sections(annotations, analysis_results)
        )

        return "\n".join(lines)

    def _generate_statistics(
        self, annotations: list[AnnotationContext]
    ) -> list[str]:
        """統計情報セクションを生成."""
        lines = ["## 統計情報", ""]

        # カテゴリ別集計
        category_counts = defaultdict(int)
        for ann in annotations:
            category_counts[ann.get_category_name()] += 1

        lines.append("### カテゴリ別")
        for category, count in sorted(
            category_counts.items(), key=lambda x: x[1], reverse=True
        ):
            lines.append(f"- **{category}**: {count}件")

        lines.append("")

        # 重要度別集計
        priority_counts = defaultdict(int)
        for ann in annotations:
            priority_counts[ann.get_priority_name()] += 1

        lines.append("### 重要度別")
        for priority, count in sorted(
            priority_counts.items(),
            key=lambda x: ["CRITICAL", "HIGH", "MEDIUM", "LOW"].index(x[0])
            if x[0] in ["CRITICAL", "HIGH", "MEDIUM", "LOW"]
            else 999,
        ):
            lines.append(f"- **{priority}**: {count}件")

        lines.append("")

        return lines

    def _generate_summary(
        self,
        annotations: list[AnnotationContext],
        analysis_results: list[AnalysisResult],
    ) -> list[str]:
        """サマリーセクションを生成（重要な発見と推奨アクション）."""
        lines = ["## サマリー", ""]

        # CRITICAL/HIGH優先度のアノテーションを抽出
        high_priority_items = []
        for ann, result in zip(annotations, analysis_results):
            if ann.priority >= 2:  # HIGH or CRITICAL
                high_priority_items.append((ann, result))

        if high_priority_items:
            lines.append("### 重要な発見")
            for ann, result in high_priority_items[:5]:  # 最大5件
                lines.append(
                    f"- **{ann.label}** ({ann.get_priority_name()}): {result.root_cause}"
                )
            lines.append("")

        # 改善提案を優先度順にソート
        all_improvements = []
        for ann, result in zip(annotations, analysis_results):
            if result.error:
                continue
            for improvement in result.improvements:
                all_improvements.append((ann, improvement))

        # 優先度でソート
        priority_order = {"HIGH": 0, "MEDIUM": 1, "LOW": 2}
        all_improvements.sort(
            key=lambda x: priority_order.get(x[1].get("priority", "LOW"), 3)
        )

        if all_improvements:
            lines.append("### 推奨アクション（優先度順）")
            for ann, improvement in all_improvements[:10]:  # 最大10件
                priority = improvement.get("priority", "UNKNOWN")
                desc = improvement.get("description", "")
                lines.append(f"- **[{priority}]** {desc} (関連: {ann.label})")
            lines.append("")

        return lines

    def _generate_category_sections(
        self,
        annotations: list[AnnotationContext],
        analysis_results: list[AnalysisResult] | None,
    ) -> list[str]:
        """カテゴリ別詳細セクションを生成."""
        lines = ["## カテゴリ別分析", ""]

        # カテゴリごとにグループ化
        by_category = defaultdict(list)
        for i, ann in enumerate(annotations):
            result = analysis_results[i] if analysis_results else None
            by_category[ann.get_category_name()].append((ann, result))

        # カテゴリ順でソート
        category_order = [
            "ISSUE",
            "QUESTION",
            "OBSERVATION",
            "POSITIVE",
            "STRATEGY",
            "TIMING",
            "CUSTOM",
        ]
        sorted_categories = sorted(
            by_category.keys(),
            key=lambda c: category_order.index(c)
            if c in category_order
            else 999,
        )

        for category in sorted_categories:
            items = by_category[category]
            lines.append(f"### {category} ({len(items)}件)")
            lines.append("")

            for i, (ann, result) in enumerate(items, 1):
                lines.extend(
                    self._generate_annotation_detail(i, ann, result)
                )

        return lines

    def _generate_annotation_detail(
        self,
        index: int,
        annotation: AnnotationContext,
        analysis_result: AnalysisResult | None,
    ) -> list[str]:
        """個別アノテーションの詳細を生成."""
        lines = [f"#### {index}. {annotation.label}", ""]

        # 基本情報
        time_sec = annotation.event_timestamp_ns / 1e9
        lines.append(f"- **時刻**: {time_sec:.3f}秒")
        lines.append(f"- **重要度**: {annotation.get_priority_name()}")

        if annotation.description:
            lines.append(f"- **詳細**: {annotation.description}")

        if annotation.has_position and annotation.position:
            x, y, z = annotation.position
            lines.append(f"- **位置**: ({x:.2f}, {y:.2f}, {z:.2f})m")

        if annotation.has_robot_context and annotation.related_robot_ids:
            robot_info = []
            for robot_id, is_ours in zip(
                annotation.related_robot_ids, annotation.robot_is_ours
            ):
                team = "自チーム" if is_ours else "相手"
                robot_info.append(f"{team} ID {robot_id}")
            lines.append(f"- **関連ロボット**: {', '.join(robot_info)}")

        lines.append("")

        # WorldModelコンテキストサマリー
        if annotation.world_model_context:
            num_snapshots = len(annotation.world_model_context)
            lines.append(
                f"**WorldModelコンテキスト**: {num_snapshots}サンプル"
            )
            lines.append("")

        # Gemini解析結果
        if analysis_result:
            if analysis_result.error:
                lines.append("**⚠️ 解析エラー**")
                lines.append(f"```\n{analysis_result.error}\n```")
            else:
                lines.append("**🤖 Gemini解析結果**")
                lines.append("")
                lines.append(f"- **根本原因**: {analysis_result.root_cause}")
                lines.append(
                    f"- **戦術的評価**: {analysis_result.tactical_analysis}"
                )
                lines.append(
                    f"- **信頼度**: {analysis_result.confidence}"
                )

                if analysis_result.improvements:
                    lines.append("")
                    lines.append("**改善提案**:")
                    for imp in analysis_result.improvements:
                        priority = imp.get("priority", "UNKNOWN")
                        desc = imp.get("description", "")
                        impl = imp.get("implementation", "")
                        lines.append(f"- **[{priority}]** {desc}")
                        if impl:
                            lines.append(f"  - 実装: {impl}")

        lines.append("")
        lines.append("---")
        lines.append("")

        return lines

    def save_report(
        self,
        report: str,
        output_path: str | Path,
    ) -> None:
        """
        レポートをファイルに保存.

        Args:
            report: レポート文字列
            output_path: 出力ファイルパス
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, "w", encoding="utf-8") as f:
            f.write(report)

        logger.info(f"Report saved to: {output_path}")
