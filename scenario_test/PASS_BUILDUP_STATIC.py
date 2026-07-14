"""静的な敵配置でのビルドアップパス成立テスト。

シュートラインは blue の壁で塞がれており（ゴール可視角 ≈ 0）、
attacker はウィングの受け手へのパスを選択するはず。
フレーキー対策として 3 試行し 2 回以上の成功で pass とする。
"""

from pass_helpers import run_pass_trial, setup_buildup_static
from rcst.communication import Communication


def test_pass_buildup_static(rcst_comm: Communication):
    results = [run_pass_trial(rcst_comm, setup_buildup_static) for _ in range(3)]
    outcomes = [r.outcome for r in results]
    print(f"PASS_BUILDUP_STATIC outcomes: {outcomes}")
    for r in results:
        print(f"  {r.to_dict()}")
    success_count = outcomes.count("SUCCESS")
    assert success_count >= 2, f"パス成功 {success_count}/3 (要求: 2以上): {outcomes}"


if __name__ == "__main__":
    rcst_comm = Communication()
    test_pass_buildup_static(rcst_comm)
    rcst_comm.close()
    print("PASS_BUILDUP_STATIC test passed")
