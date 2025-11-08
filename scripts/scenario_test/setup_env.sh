#!/bin/bash
# シナリオテスト用Python環境のセットアップスクリプト

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
VENV_DIR="${REPO_ROOT}/scenario_test_env"

echo "=== シナリオテスト環境のセットアップを開始 ==="

# Python仮想環境の作成
if [ ! -d "${VENV_DIR}" ]; then
    echo "Python仮想環境を作成中: ${VENV_DIR}"
    python3 -m venv "${VENV_DIR}"
else
    echo "Python仮想環境は既に存在します: ${VENV_DIR}"
fi

# 仮想環境の有効化
# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

# pipのアップグレード
echo "pipをアップグレード中..."
python -m pip install --upgrade pip --quiet

# protobuf-compilerのインストール確認
if ! command -v protoc &>/dev/null; then
    echo "protobuf-compilerをインストールしています..."
    sudo apt update
    sudo apt install -y protobuf-compiler
else
    echo "protobuf-compilerは既にインストールされています"
fi

# 必要な依存関係のインストール
echo "必要なPythonライブラリをインストール中..."
pip install pyyaml setuptools jinja2 typeguard lark

# robocup_scenario_testライブラリのインストール
echo "robocup_scenario_testライブラリをインストール中..."
pip install -v git+https://github.com/SSL-Roots/robocup_scenario_test

# pytestのインストール
echo "pytestをインストール中..."
pip install pytest

echo ""
echo "=== セットアップが完了しました ==="
echo "仮想環境: ${VENV_DIR}"
echo ""
echo "仮想環境を有効化するには："
echo "  source ${VENV_DIR}/bin/activate"
echo ""
