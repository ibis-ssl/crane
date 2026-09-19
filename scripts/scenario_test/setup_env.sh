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
# フォークを正本として使う。フォークには以下が入っている:
#
# - VisionWorld が検出の途切れたロボットを削除する。本家では一度でも見えた
#   ロボットが最後の位置に残り続け、send_empty_world() が効く前の初期配置が
#   亡霊としてフィールドに居座るため、ロボット同士の距離を見るテストが存在しない
#   ロボットとの衝突を報告していた。
# - vision の geometry パケットからフィールド寸法を取り出して公開する。
#   scenario シミュレータは Division B（9000x6000）で走るので、テストが Division A の
#   座標を直書きするとシミュレータに静かにクランプされ、要求したのとは別の世界で
#   テストが進む。scenario_test/field_helpers.py がこれを使って座標を導出する。
#
# cm4-sim イメージと同じくcommit SHAで固定する。追従にするとフォーク側のpushだけで
# craneのCIが壊れ、原因がcrane側のリグレッションに見えてしまう。
#
# 固定先は必ず main 上のcommitにすること。PRのブランチ上のcommitを指したままにすると、
# squash mergeでブランチが消えた瞬間に到達不能になり、pip installが失敗する。
# 症状はcrane側のCI失敗として出るので原因が分かりにくい。
RCST_URL="git+https://github.com/ibis-ssl/robocup_scenario_test@941afdb7da384d7ef5901ca2ab814dd7689f5ba2"

echo "robocup_scenario_testライブラリをインストール中..."
# 1回目: 依存を解決する。
pip install -v "${RCST_URL}"
# 2回目: 本体だけを強制的に入れ替える。
#
# pip はバージョン番号が一致すると取得元URLが違っても再インストールを省略する。
# rcst は本家もフォークも 0.1.0 なので、本家版が入った venv が残っていると
# 上記の SHA 固定が無言で効かなくなる。実際に本家 ded9751 が残った状態が発生し、
# フォークにしかない rcst.field_geometry を import できず、scenario_test/conftest.py の
# field フィクスチャが解決できずに全シナリオテストが collection 時点で落ちていた。
# --no-deps を付けるのは、依存は1回目で解決済みで再ビルドが不要なため。
pip install --force-reinstall --no-deps "${RCST_URL}"

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
