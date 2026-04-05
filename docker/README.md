# Dockerディレクトリ構成

このディレクトリには、craneプロジェクトの各種Docker環境が整理されています。

## ディレクトリ一覧

### 開発・テスト環境

- **`dev/`** - 統合開発環境
  - シミュレーション(sim)と実機(real)を統合
  - 簡単なコマンドで環境を切り替え可能
  - 詳細: [dev/README.md](dev/README.md)

### 自動テスト環境

- **`scenario/`** - シナリオテスト環境
  - CIで実行される自動テスト環境
  - 特定シナリオの動作検証
  - 詳細: [scenario/README.md](scenario/README.md)

- **`match-vs-tigers/`** - 対戦テスト環境
  - TIGERs Sumatra AIとの自動対戦
  - 定期的なベンチマーク評価
  - CI実行: `.github/workflows/match-vs-tigers.yaml`

### ビルド支援

- **`base/`** - ベースイメージのDockerfile
- **`prebuilt/`** - プリビルド依存関係
- **`ccache/`** - ccacheボリューム設定

### 共通設定

- **`config/`** - 環境間で共有される設定ファイル
  - `engine.yaml` - Game Controllerエンジン設定
  - `state-store.json.stream` - GC初期状態(自動生成、gitignore対象)

## ディレクトリ命名規則

- すべてのディレクトリ名は **ケバブケース(kebab-case)** で統一されています
  - 例: `match-vs-tigers`, `ssl-game-controller`

## 使い方

各環境の詳細な使用方法は、それぞれのREADME.mdを参照してください。

### 開発環境の起動

```bash
# シミュレーション環境(デフォルト)
./scripts/docker-dev.sh

# 実機環境
./scripts/docker-dev.sh real

# バックグラウンド起動
./scripts/docker-dev.sh -d
./scripts/docker-dev.sh real -d

# robot-managerなしで起動
./scripts/docker-dev.sh --no-debug

# 停止
./scripts/docker-dev.sh down
```

詳細は [dev/README.md](dev/README.md) を参照してください。

### テスト実行

```bash
# シナリオテスト(CI互換)
cd docker/scenario
docker compose up

# TIGERs対戦テスト(ローカル実行)
./scripts/match-vs-tigers/run_local.sh
```

## 関連ドキュメント

- プロジェクト全体の概要: [../AGENTS.md](../AGENTS.md)
- Dockerビルド: `docker/base/Dockerfile`
- CI設定: `.github/workflows/`
