"""
pytestの設定とコマンドライン引数の定義

このファイルは、シナリオテストで使用するpytestのカスタム引数を定義します。
注意: rcstライブラリが既にいくつかのオプションを定義しているため、
     重複を避けるために条件付きで追加します。
"""

import pytest


def pytest_addoption(parser):
    """pytestのコマンドライン引数を追加"""
    # rcstライブラリが既に--vision_port, --logging, --log_recorderを定義しているので
    # 追加で必要なオプションがあればここに記述


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
