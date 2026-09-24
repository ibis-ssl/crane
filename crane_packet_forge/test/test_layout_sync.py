# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""layout.py が robot_packet.h と同期していることを確かめる。

生成をやり直して committed 版と突き合わせる。ヘッダが動いたらここが落ちる。
落ちたときは `python3 tools/gen_layout.py` で再生成し、AGENTS.md の通り
Orion_CM4 と G474 の robot_packet.h も同期すること。
"""

import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
GENERATOR = PACKAGE_ROOT / "tools" / "gen_layout.py"
GENERATED = PACKAGE_ROOT / "crane_packet_forge" / "layout.py"

_MISSING = object()


def test_layout_matches_header() -> None:
    """生成をやり直した結果と committed 版が「値として」一致すること。

    テキスト比較にしない。layout.py は pre-commit の ruff-format が整形するので、
    生成器の出力とは空白や改行が食い違う。守りたいのは書式ではなく定数の中身。
    """
    result = subprocess.run(
        [sys.executable, str(GENERATOR), "--stdout"],
        capture_output=True,
        text=True,
        check=True,
    )

    regenerated: dict = {}
    exec(compile(result.stdout, str(GENERATOR), "exec"), regenerated)  # noqa: S102

    from crane_packet_forge import layout as committed

    expected = {
        k: v for k, v in regenerated.items() if not k.startswith("__") and k != "math"
    }
    actual = {k: getattr(committed, k, _MISSING) for k in expected}

    differences = {
        k: (expected[k], actual[k]) for k in expected if expected[k] != actual[k]
    }
    assert not differences, (
        f"layout.py が robot_packet.h と食い違っている: {differences}。"
        "`python3 tools/gen_layout.py` で再生成し、パケット仕様の変更なら "
        "Orion_CM4 / G474 との同期と統合通信テストを行う（docs/network.md）。"
    )

    extra = {
        k
        for k in vars(committed)
        if not k.startswith("__") and k != "math" and k not in expected
    }
    assert not extra, (
        f"layout.py に生成器が作らない定義がある（手で足した?）: {sorted(extra)}"
    )


def test_ranges_are_taken_from_serialize_not_guessed() -> None:
    """レンジは名前から推測できない。実際にヘッダから採れているかを固定する。"""
    from crane_packet_forge import layout as L

    # mode 3 の theta は名前が theta でも ±M_PI ではない。
    assert L.RANGES["POLAR:target_global_velocity_theta"] == 32.767
    # vision / target の theta は ±M_PI。
    assert L.RANGES["VISION_GLOBAL_THETA_HIGH"] != 32.767
    assert L.RANGES["TARGET_GLOBAL_THETA_HIGH"] != 32.767


def test_framing_constants() -> None:
    from crane_packet_forge import layout as L

    assert L.CMD_SIZE == 64
    assert L.SLOT_SIZE == 65
    assert L.PACKET_SIZE == 715
    assert L.MAX_ROBOT_ID == L.SLOTS - 1
