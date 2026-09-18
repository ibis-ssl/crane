"""PASS_BUILDUP_STATIC の 6 試行版（ベースライン比較専用の一時テスト）。

3 試行では「0/3 と 1/3」の差が偶然と区別できないため、回帰判定のあいだだけ
試行数を増やして成功率そのものを見る。assert はせず件数を出力する。
"""

from field_helpers import Field
from pass_helpers import run_pass_trial, setup_buildup_static


def test_pass_buildup_static6(field: Field):
    results = [run_pass_trial(field, setup_buildup_static) for _ in range(6)]
    outcomes = [r.outcome for r in results]
    print(f"PASS_BUILDUP_STATIC6 outcomes: {outcomes}")
    for i, r in enumerate(results, 1):
        print(f"  試行{i}: {r.to_dict()}")
    print(f"PASS_BUILDUP_STATIC6 SUCCESS={outcomes.count('SUCCESS')}/6")
