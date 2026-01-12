# crane_commentary

## 概要

**crane_commentary**は、Gemini Multimodal Live APIを活用したRoboCup SSL向けの**リアルタイムAI実況システム**です。試合中に発生するイベント（パス、シュート、ゴールなど）を検知し、AI実況者による動的な音声実況を生成します。

## 主要機能

- **リアルタイム音声実況**: Gemini Multimodal Live API (Websocket) による低遅延な音声生成
- **イベント駆動型実況**: シュート、パス、ゴールなどのイベント（RONARイベント）に即座に反応（Reflex Mode）
- **戦術分析実況**: 試合の静止時間や展開が落ち着いた際に、現在の戦術状況を分析して実況（Analyst Mode）
- **多言語対応**: Geminiの能力により、日本語を含む多言語での自然な実況が可能
- **チーム情報統合**: チームの特性や選手名（背番号）を考慮した実況のパーソナライズ
- **関数呼び出し (Function Calling)**: AIがリアルタイムに世界モデル（ロボット位置、スコア等）を問い合わせ、正確な情報に基づいた実況を実現

## アーキテクチャ上の役割

システムの「実況・エンターテインメント層」として、`crane_world_model_publisher`や`crane_play_switcher`（イベント生成元）からの情報を受け取り、外部のAIサービス（Google Gemini）と連携して音声を生成・出力します。

## コンポーネント構成

### メインノード

- **commentary_node**: 実況システムのメインノード（Python）。
  - 各種イベントの購読とフィルタリング
  - Gemini APIとのWebsocket通信管理
  - 音声データの再生（sounddevice）

### 内部モジュール

- **Statler (WorldModelWriter/Reader)**: 実況に必要な世界モデル情報の管理・整形
- **Gemini Live Client**: Gemini Multimodal Live APIとの低レイテンシ通信
- **Function Handler**: Geminiからの情報問い合わせ要求（Function Call）を処理
- **Audio Output**: 受信したPCM音声データの再生管理

## 依存関係

### システム依存

- **Google Gemini API Key**: APIへのアクセスに必要
- **Pythonライブラリ**: `websockets`, `sounddevice`, `numpy`, `PyYAML`

### コア依存

- **crane_msgs**: `RonarEvent`, `PlaySituation`, `WorldModel` の購読

## 使用方法

### 基本起動

```bash
# APIキーを環境変数に設定
export GEMINI_API_KEY="your_api_key_here"

# ノードの起動
ros2 run crane_commentary commentary_node
```

### 主要パラメータ

| パラメータ名 | 型 | 説明 | デフォルト値 |
|-------------|----|------|-------------|
| `gemini_api_key` | string | Gemini APIキー（環境変数優先） | "" |
| `gemini_model` | string | 使用するGeminiモデル名 | "gemini-2.0-flash-exp" |
| `audio_sample_rate` | int | 音声サンプリングレート(Hz) | 24000 |
| `analyst_silence_threshold` | float | 解析モードに移行する無音時間(s) | 5.0 |
| `mode` | string | 動作モード ("reflex_analyst", "self_commentary") | "reflex_analyst" |

## 動作モード

### Reflex Analyst Mode (`reflex_analyst`)

デフォルトのモードです。第三者視点の実況解説者として振る舞います。

- **Reflex**: シュートやゴールなどのイベントに即座に反応
- **Analyst**: 試合が落ち着いている時に戦術分析を行う

### Self Commentary Mode (`self_commentary`)

システム（またはロボット）自身の視点として、意思決定や意図を語るモードです。

- **意図追跡**: `IntentTracker`により、ロボットの役割変更や目標位置の変更を追跡
- **内部状態の実況**: 「攻撃に切り替えます」「ロボット1にパスコースを作らせます」といった、システムの思考プロセスを言語化します。
- **用途**: デバッグやデモンストレーション、システムの透明性向上

## 設定ファイル

- **system_instruction.md**: AI実況者の性格、役割、実況スタイルを定義
- **system_instruction_self.md**: Self Commentaryモード用のシステム指示書
- **function_declarations.json**: AIが使用可能なツール（世界モデル取得など）の定義
- **team_profiles.yaml**: チーム名の読み方や特徴の設定
- **ssl_rules.yaml**: SSLのルール情報のサマリー

---

**関連パッケージ**: [crane_world_model_publisher](./crane_world_model_publisher.md) | [crane_msgs](./crane_msgs.md) | [crane_play_switcher](./crane_play_switcher.md)
