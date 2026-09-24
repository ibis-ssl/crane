# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Python の組み立てが C++ と同じバイト列を作ることを確かめる。

golden_vectors.json の正本は C++ 側（robot_packet.h のシリアライザ）。
C++ の test_golden_vectors が同じ JSON を再現できることを確かめているので、
ここが通れば C++ と Python が一致している。

覆えるのは構造体で表現できる部分集合だけ。未定義フラグビットや
control_mode と mode_args の不一致は test_roundtrip.py が受け持つ。
"""

import json
from pathlib import Path

import pytest

GOLDEN = Path(__file__).resolve().parent / "golden_vectors.json"


def load_vectors():
    data = json.loads(GOLDEN.read_text())
    return data["base"], data["vectors"]


def test_golden_file_is_not_empty() -> None:
    _base, vectors = load_vectors()
    assert vectors, "golden_vectors.json が空。dump_golden で生成する"


@pytest.mark.parametrize("vector", load_vectors()[1], ids=lambda v: v["name"])
def test_python_matches_cpp(vector) -> None:
    from crane_packet_forge.assemble import assemble_command
    from crane_packet_forge.spec import PacketSpec

    base, _ = load_vectors()
    spec = PacketSpec(base=base)
    for key, value in vector["fields"].items():
        spec.set_field(key, value)

    command = assemble_command(
        spec, check_counter=vector["fields"].get("check_counter", 0)
    )
    assert command.hex() == vector["hex"], (
        f"{vector['name']}: Python の組み立てが C++ と違う。"
        "レンジ・オフセット・量子化のどれかがずれている"
    )


def test_two_byte_fields_are_exercised_with_distinct_values() -> None:
    """レンジ変更がこの照合をすり抜けないよう、基準が薄くなっていないか見張る。

    2 バイトフィールドが全て 0 のままだと、±32.767 と ±M_PI を取り違えても
    バイト列が変わらず、照合が素通りしてしまう。
    """
    from crane_packet_forge import fields as F

    _base, vectors = load_vectors()
    two_byte_keys = {f.key for f in F.ALL_FIELDS if f.kind == F.TWO_BYTE}
    covered = set()
    for vector in vectors:
        for key, value in vector["fields"].items():
            if key in two_byte_keys and value != 0:
                covered.add(key)

    missing = two_byte_keys - covered
    assert not missing, (
        "非ゼロで叩かれていない 2 バイトフィールドがある。"
        f"レンジ変更を検出できない: {sorted(missing)}"
    )
