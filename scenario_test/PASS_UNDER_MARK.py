"""受け手がゴール側からマークされた状態でのパス成立テスト。

マーカーはパスラインを塞がない位置（受け手のゴール側 0.7m）にいるため
パス自体は通せるはず。密着マーク下でのレシーブ品質を試す。
フレーキー対策として 3 試行し 2 回以上の成功で pass とする。
"""

from field_helpers import Field
from pass_helpers import run_pass_trial, setup_under_mark


def test_pass_under_mark(field: Field):
    results = [run_pass_trial(field, setup_under_mark) for _ in range(3)]
    outcomes = [r.outcome for r in results]
    print(f"PASS_UNDER_MARK outcomes: {outcomes}")
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
