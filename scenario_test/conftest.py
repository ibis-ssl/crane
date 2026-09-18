"""
pytestの設定とコマンドライン引数の定義

このファイルは、シナリオテストで使用するpytestのカスタム引数を定義します。
注意: rcstライブラリが既にいくつかのオプションを定義しているため、
     重複を避けるために条件付きで追加します。
"""

import pytest
from field_helpers import make_field
from pass_plan_log import start_recorder


def pytest_addoption(parser):
    """pytestのコマンドライン引数を追加"""
    # rcstライブラリが既に--vision_port, --logging, --log_recorderを定義しているので
    # 追加で必要なオプションがあればここに記述


@pytest.fixture
def field(rcst_comm):
    """vision の geometry から導出したフィールドと、検査付きの配置 API。

    シナリオテストの座標は必ずこれを経由して決めること。区分（Division A / B）を
    決め打ちした座標はシミュレータに静かにクランプされ、テストが要求したのとは
    別の世界で走ってしまう。詳細は field_helpers.py を参照。

    rcst_comm と同じく function スコープ。geometry は detection より頻度が低いので
    最初の1パケットを待つぶんだけ時間がかかる（届かなければ例外）。
    """
    return make_field(rcst_comm)


@pytest.fixture
def pass_plan_log():
    """PassPlan と役割割当の観測ログ。

    pytest は venv で動き rclpy を持たないため、観測は sidecar プロセスに任せ、
    ここでは JSONL を読むだけにする。理由は pass_plan_recorder.py の docstring を参照。

    crane が起動していないなど、/world_model を観測できない場合は setup で例外を出す。
    シナリオの assert 失敗と環境の起動失敗を区別するため、ここで落とすのが正しい。

    field と同じ function スコープ。sidecar の起動に約1秒かかる。
    """
    recorder, log = start_recorder()
    try:
        yield log
    finally:
        recorder.stop()


@pytest.fixture(scope="session")
def vision_port(request):
    """SSL-Visionのポート番号を取得するフィクスチャ"""
    return int(request.config.getoption("--vision_port"))


@pytest.fixture(scope="session")
def logging_enabled(request):
    """ログ記録が有効かどうかを取得するフィクスチャ"""
    return request.config.getoption("--logging")


@pytest.fixture(scope="session")
def log_recorder_path(request):
    """ssl-log-recorderのパスを取得するフィクスチャ"""
    return request.config.getoption("--log_recorder")
