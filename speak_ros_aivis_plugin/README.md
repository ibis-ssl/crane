# speak_ros_aivis_plugin

[Aivis Speech Engine](https://github.com/Aivis-Project/AivisSpeech-Engine)用のspeak_rosプラグイン

## 概要

Aivis Speech EngineはVOICEVOX互換のAPIを持つオープンソース音声合成エンジンです。このプラグインにより、speak_ros経由でAivis Speech Engineを使用できます。

## 特徴

- VOICEVOX API互換（`/audio_query` → `/synthesis`）
- デフォルトポート: **10101**
- Aivis固有パラメータのサポート:
  - `intonationScale`: 感情の強さ（0.0-2.0）
  - `tempoDynamicsScale`: 話速変動（0.0-2.0）

## 依存関係

- speak_ros
- rclcpp
- pluginlib
- libcpprest-dev

## ビルド

```bash
colcon build --packages-select speak_ros_aivis_plugin
```

## 使用方法

### 1. Aivis Speech Engineの起動

```bash
# Dockerで起動（推奨）
docker compose -f docker/sim/docker-compose.yaml up aivis-speech -d

# または手動で起動
# https://github.com/Aivis-Project/AivisSpeech-Engine
```

### 2. speak_rosノードの起動

```bash
ros2 run speak_ros speak_ros_node --ros-args \
  -p plugin_name:=aivis_plugin::AivisPlugin
```

### 3. パラメータのカスタマイズ

```bash
ros2 run speak_ros speak_ros_node --ros-args \
  -p plugin_name:=aivis_plugin::AivisPlugin \
  -p aivis_plugin/speaker:=888753760 \
  -p aivis_plugin/intonationScale:=1.5 \
  -p aivis_plugin/tempoDynamicsScale:=1.2
```

### 4. 音声合成の実行

```bash
ros2 action send_goal /speak speak_ros_interfaces/action/Speak \
  "{text: 'こんにちは、Aivisです。', speed_rate: 1.0}"
```

## パラメータ一覧

| パラメータ名 | 型 | デフォルト値 | 説明 |
|------------|-----|------------|------|
| speaker | int | 888753760 | AivisスピーカーID |
| host_name | string | localhost | Aivisエンジンのホスト |
| port | int | 10101 | Aivisエンジンのポート |
| speedScale | double | 1.0 | 話速（1.0が標準） |
| pitchScale | double | 0.0 | ピッチ調整 |
| intonationScale | double | 1.0 | 感情の強さ（0.0-2.0） |
| volumeScale | double | 1.0 | 音量スケール |
| tempoDynamicsScale | double | 1.0 | 話速変動（0.0-2.0） |
| prePhonemeLength | double | 0.1 | 前音素長 [秒] |
| postPhonemeLength | double | 0.1 | 後音素長 [秒] |
| outputSamplingRate | int | 44100 | 出力サンプリングレート [Hz] |
| outputStereo | string | false | ステレオ出力 |

## スピーカーID一覧

デフォルトモデル（まお v1.2.0）:

| ID | スタイル |
|----|---------|
| 888753760 | ノーマル |
| 888753761 | ふつー |
| 888753762 | あまあま |
| 888753763 | おちつき |
| 888753764 | からかい |
| 888753765 | せつなめ |

スピーカーIDの確認:

```bash
curl http://localhost:10101/speakers
```

## VOICEVOXとの違い

| 項目 | VOICEVOX | Aivis |
|------|----------|-------|
| ポート | 50021（固定） | 10101（設定可能） |
| `speedScale` デフォルト | 2.0 | 1.0 |
| `intonationScale` | なし | あり（感情の強さ） |
| `tempoDynamicsScale` | なし | あり（話速変動） |
| `outputSamplingRate` | 24000 | 44100 |
| `outputStereo` | true | false |

## トラブルシューティング

### エンジンに接続できない

```bash
# エンジンの起動を確認
curl http://localhost:10101/speakers

# Dockerコンテナの状態を確認
docker compose -f docker/sim/docker-compose.yaml ps aivis-speech
```

### スピーカーIDが見つからない

利用可能なスピーカーIDを確認:

```bash
curl http://localhost:10101/speakers | python3 -m json.tool
```

## ライセンス

MIT License

## 参考リンク

- [Aivis Speech Engine](https://github.com/Aivis-Project/AivisSpeech-Engine)
- [speak_ros](https://github.com/HansRobo/speak_ros)
- [speak_ros_voicevox_plugin](https://github.com/HansRobo/speak_ros_voicevox_plugin)
