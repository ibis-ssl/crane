# crane_play_switcher

## 概要

試合状況に応じた**プレイ自動選択システム**として、Vision・Referee データから状況を抽出し、最適な戦術プレイを自動選択・切り替えを行うパッケージです。ゲーム状況の変化に応じて動的に戦術を調整します。

## 主要機能

- **状況認識**: Vision/Referee データからのゲーム状況抽出
- **プレイ選択**: 状況に最適な戦術プレイの自動選択
- **動的切り替え**: リアルタイムでの戦術変更
- **状況履歴管理**: 過去の状況変化パターンの記録

## アーキテクチャ上の役割

Craneシステムの**戦術切り替え制御層**として、ゲーム状況の変化を監視し、session_controllerとplanner_pluginsに対して最適な戦術プレイを指示します。

## 使用方法

```cpp
#include "crane_play_switcher/play_switcher.hpp"

auto play_switcher = std::make_shared<PlaySwitcher>();
PlayType current_play = play_switcher->selectPlay(world_model, referee_info);
```

## 最近の開発状況

🟡 **中活動**: 戦術切り替えロジックの改良と新しい状況判定機能の追加が継続的に行われています。

---

**関連パッケージ**: [crane_game_analyzer](./crane_game_analyzer.md) | [crane_tactic_coordinator](./crane_tactic_coordinator.md)
