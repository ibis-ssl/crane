"""静的な敵配置でのビルドアップパス成立テスト。

シュートラインは blue の壁で塞がれており（ゴール可視角 ≈ 0）、
attacker はウィングの受け手へのパスを選択するはず。
フレーキー対策として 3 試行し 2 回以上の成功で pass とする。
"""

from field_helpers import Field
from pass_helpers import run_pass_trial, setup_buildup_static


def test_pass_buildup_static(field: Field):
    results = [run_pass_trial(field, setup_buildup_static) for _ in range(3)]
    outcomes = [r.outcome for r in results]
    print(f"PASS_BUILDUP_STATIC outcomes: {outcomes}")
    # 配置が反映されるまでの待ちと、キック時点で配置からどれだけ崩れていたか。
    # crane は yellow 全機を動かすので、ずれが大きい試行は「テストが作った
    # パスコースとは別の状況」を見ている。判定ではなく切り分けのために出す。
    waits = [f"{r.placement_wait:.2f}" for r in results]
    print(f"  配置待ち[s]: {waits}")
    for i, r in enumerate(results, 1):
        drift = ", ".join(f"Y{k}:{v:.2f}" for k, v in sorted(r.drift_at_kick.items()))
        print(f"  試行{i} キック時の配置ずれ[m]: {drift or '(未計測)'}")
    for r in results:
        print(f"  {r.to_dict()}")
    success_count = outcomes.count("SUCCESS")
    assert success_count >= 2, f"パス成功 {success_count}/3 (要求: 2以上): {outcomes}"
