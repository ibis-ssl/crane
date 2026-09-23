"""受け手がゴール側からマークされた状態でのパス成立テスト。

マーカーはパスラインを塞がない位置（受け手のゴール側 0.7m）にいるため
パス自体は通せるはず。密着マーク下でのレシーブ品質を試す。
フレーキー対策として 3 試行し 2 回以上の成功で pass とする。
"""

from field_helpers import Field
from pass_helpers import run_pass_trials, setup_under_mark


def test_pass_under_mark(field: Field):
    outcomes = run_pass_trials(field, setup_under_mark, "PASS_UNDER_MARK")
    success_count = outcomes.count("SUCCESS")
    assert success_count >= 2, f"パス成功 {success_count}/3 (要求: 2以上): {outcomes}"
