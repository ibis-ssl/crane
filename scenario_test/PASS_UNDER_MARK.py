"""受け手がゴール側からマークされた状態でのパス成立テスト。

マーカーはパスラインを塞がない位置（受け手のゴール側 0.7m）にいるため
パス自体は通せるはず。密着マーク下でのレシーブ品質を試す。
フレーキー対策として 3 試行し 2 回以上の成功で pass とする。
"""

from pass_helpers import run_pass_trial, setup_under_mark
from rcst.communication import Communication


def test_pass_under_mark(rcst_comm: Communication):
    results = [run_pass_trial(rcst_comm, setup_under_mark) for _ in range(3)]
    outcomes = [r.outcome for r in results]
    print(f"PASS_UNDER_MARK outcomes: {outcomes}")
    for r in results:
        print(f"  {r.to_dict()}")
    success_count = outcomes.count("SUCCESS")
    assert success_count >= 2, f"パス成功 {success_count}/3 (要求: 2以上): {outcomes}"


if __name__ == "__main__":
    rcst_comm = Communication()
    test_pass_under_mark(rcst_comm)
    rcst_comm.close()
    print("PASS_UNDER_MARK test passed")
