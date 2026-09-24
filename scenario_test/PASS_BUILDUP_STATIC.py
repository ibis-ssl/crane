"""静的な敵配置でのビルドアップパス成立テスト。

シュートラインは blue の壁で塞がれており（ゴール可視角 ≈ 0）、
attacker はウィングの受け手へのパスを選択するはず。
フレーキー対策として 3 試行し 2 回以上の成功で pass とする。
"""

from field_helpers import Field
from pass_helpers import run_pass_trials, setup_buildup_static


def test_pass_buildup_static(field: Field):
    outcomes = run_pass_trials(field, setup_buildup_static, "PASS_BUILDUP_STATIC")
    success_count = outcomes.count("SUCCESS")
    assert success_count >= 2, f"パス成功 {success_count}/3 (要求: 2以上): {outcomes}"
