# crane_ball_calibration_ui

## 概要

`crane_ball_calibration_ui`は、ボール物理モデルとキッカーモデルのパラメータをHuman-in-the-Loopでインタラクティブに最適化するためのWebUIツールです。FastAPIバックエンドとブラウザUIにより、rosbagデータから最適なパラメータを探索・エクスポートできます。

## 主要機能

- **軌跡データ読み込み**: rosbagからボール軌跡データを抽出
- **パラメータ最適化**: 減速係数・キックパワーのNumPy最適化
- **インタラクティブUI**: ブラウザ上でパラメータをリアルタイム調整・可視化
- **YAMLエクスポート**: 最適化したパラメータをROS 2設定ファイルとして出力

## アーキテクチャ上の役割

**依存レイヤ**: 統合層（Layer 5）- キャリブレーションツール

ROS 2とは疎結合（rclpy依存）。主にオフラインでrosbagデータを使用してパラメータ最適化を行う。

## コンポーネント

### Pythonモジュール

- **app.py**: FastAPI Webアプリケーション（REST API + 静的ファイル配信）
- **data_loader.py**: rosbagからボール軌跡を読み込む
- **optimizer.py**: NumPy最適化（減速係数・キックパワーモデル）
- **models.py**: Pydanticデータモデル
- **yaml_exporter.py**: キャリブレーション結果のYAMLエクスポート

### REST API エンドポイント

- `GET /`: メインUIページ
- `POST /load`: rosbagデータの読み込み
- `POST /optimize`: パラメータ最適化の実行
- `POST /predict`: 予測軌跡の計算
- `GET /export`: キャリブレーションYAMLのエクスポート

## 依存関係

- `rclpy`
- `python3-fastapi`, `python3-numpy`, `python3-pydantic`, `python3-yaml`

## 使用方法

```bash
# launchファイルで起動
ros2 launch crane_ball_calibration_ui calibration_ui.launch.py

# ブラウザでアクセス
# http://localhost:8000/
```

### ワークフロー

1. UIでrosbagディレクトリを指定してデータを読み込む
2. 軌跡データを確認し、最適化を実行
3. パラメータをブラウザ上でインタラクティブに調整
4. 最適なパラメータをYAMLとしてエクスポート

## 最近の開発状況

- **2026年1月（PR #1213）**: 初回実装 - ボール物理モデルパラメータのHuman-in-the-Loop最適化WebUI

## 関連パッケージ

- [crane_physics](./crane_physics.md) - 最適化対象の物理モデル
- [crane_bag](./crane_bag.md) - rosbagデータ読み込み
