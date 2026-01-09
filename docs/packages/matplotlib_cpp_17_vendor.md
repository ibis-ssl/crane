# matplotlib_cpp_17_vendor

## 概要

**C++17対応matplotlib**のベンダーパッケージです。C++アプリケーションからPythonのmatplotlibライブラリを呼び出し、グラフ描画・データ可視化機能を提供します。戦略分析・デバッグ・パフォーマンス評価での可視化に使用されます。

## 主要機能

- **C++からのグラフ描画**: matplotlib-cppラッパーによるシームレスな描画
- **リアルタイム可視化**: ライブデータのグラフ表示
- **多様なプロット**: 線グラフ・散布図・ヒートマップ・3Dプロット
- **ファイル出力**: PNG・PDF・SVG形式での保存

## 技術的特徴

### C++17互換性

- **Modern C++**: C++17標準への完全対応
- **型安全**: STLコンテナとの統合
- **RAII**: リソース管理の自動化

### Python統合

- **Python C API**: matplotlibの直接呼び出し
- **NumPy配列**: 効率的なデータ転送
- **スクリプトレス**: Pythonスクリプト不要

## アーキテクチャ上の役割

Craneシステムの**データ可視化支援**として、戦略プランナーでの分析結果表示、デバッグ情報の可視化、パフォーマンス評価グラフの生成を担います。

## 使用例

```cpp
#include "matplotlib_cpp.hpp"
namespace plt = matplotlibcpp;

// 戦略分析グラフ
std::vector<double> time_data = {0, 1, 2, 3, 4};
std::vector<double> score_data = {0.2, 0.7, 0.5, 0.9, 0.8};

plt::plot(time_data, score_data);
plt::xlabel("Time (s)");
plt::ylabel("Strategy Score");
plt::title("Strategy Performance");
plt::save("strategy_analysis.png");
```

## 用途

- **戦略分析**: プランナー性能の可視化
- **軌道表示**: ロボット・ボール軌道の描画  
- **統計グラフ**: 試合統計・パフォーマンス分析
- **デバッグ**: 内部状態の可視化

## 最近の開発状況

🟢 **安定**: 可視化ライブラリとして基本機能が確立しており、新しい描画機能の追加や描画性能の向上が継続的に行われています。

---

**関連パッケージ**: [crane_tactics](./crane_tactics.md) | [crane_game_analyzer](./crane_game_analyzer.md)
