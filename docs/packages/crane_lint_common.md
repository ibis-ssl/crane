# crane_lint_common

## 概要

Craneプロジェクト全体の**共通リント設定とコード品質管理**を担うパッケージです。C++、Python、ROS 2固有のコーディング規約を統一し、継続的インテグレーション（CI）での自動品質チェックを実現します。

## 主要機能

- **統一コーディング規約**: C++・Python・ROSの標準化されたスタイル
- **自動フォーマット**: clang-format、black、ruffによる自動整形
- **静的解析**: cpplint、cppcheck、静的解析ツールの統合
- **CI/CD統合**: pre-commitフック、GitHub Actions連携

## コード品質ツール

- **C++**: clang-format、cpplint、cppcheck
- **Python**: black、ruff、flake8
- **ROS 2**: ament_lint系ツール
- **Git**: pre-commit hooks

## アーキテクチャ上の役割

Craneプロジェクトの**開発品質保証基盤**として、全パッケージで共通のコード品質基準を維持し、チーム開発の効率性を向上させます。

## 使用方法

```bash
# リント実行
ament_lint_auto

# フォーマット適用
clang-format -i *.cpp *.hpp
black *.py
```

## 最近の開発状況

🟢 **安定**: コード品質管理システムとして基本機能が確立しており、新しいリントルールの追加やツールの更新が継続的に行われています。

---

**関連パッケージ**: 全パッケージ（共通品質基盤）
