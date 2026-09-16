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
#
# 本家 SSL-Roots/robocup_scenario_test は更新が止まっているため、ibis-ssl の
# フォークを正本として使う。フォークには VisionWorld が検出の途切れたロボットを
# 削除するようにした修正が入っている。本家では一度でも見えたロボットが最後の位置に
# 残り続け、send_empty_world() が効く前の初期配置が亡霊としてフィールドに居座るため、
# ロボット同士の距離を見るテストが存在しないロボットとの衝突を報告していた。
#
# cm4-sim イメージと同じくcommit SHAで固定する。追従にするとフォーク側のpushだけで
# craneのCIが壊れ、原因がcrane側のリグレッションに見えてしまう。
echo "robocup_scenario_testライブラリをインストール中..."
pip install -v git+https://github.com/ibis-ssl/robocup_scenario_test@aa3a6ab6b8608224e107f51003be0503fc47d41f

# pytestのインストール（ROS 2 Jazzy互換性のため7.4.4を指定）
echo "pytestをインストール中..."
pip install "pytest==7.4.4"

echo ""
echo "=== セットアップが完了しました ==="
echo "仮想環境: ${VENV_DIR}"
echo ""
echo "仮想環境を有効化するには："
echo "  source ${VENV_DIR}/bin/activate"
echo ""
