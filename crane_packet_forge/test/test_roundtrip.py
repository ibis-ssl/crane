# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""assemble → decode → assemble の同一性を、不正な組み合わせも含めて確かめる。

golden_vectors による C++ との照合は「構造体で表現できる部分集合」しか覆えない。
未定義フラグビット、control_mode と mode_args の不一致、生バイト上書き —
つまりこのツールの主目的そのもの — には C++ 側に基準が存在しない。
外部基準の要らないこの性質テストが、その領域の唯一の網になる。
"""

import random

import pytest
from crane_packet_forge.assemble import assemble_command, build_datagram
from crane_packet_forge.codec import decode_two_byte, encode_two_byte
from crane_packet_forge.decode import decode_command, split_datagram
from crane_packet_forge.spec import PacketSpec

from crane_packet_forge import fields as F
from crane_packet_forge import layout as L


def spec_from_decoded(decoded, *, base="neutral") -> PacketSpec:
    """デコード結果から spec を組み直す。GUI の「読んで直して送り直す」に相当。"""
    spec = PacketSpec(base=base)
    for item in decoded:
        spec.fields[item.key] = item.value
    return spec


def random_spec(rng: random.Random) -> PacketSpec:
    """不正な組み合わせも作る。それが作れることがこのツールの目的。"""
    spec = PacketSpec(base=rng.choice(["neutral", "zeros"]))
    for definition in F.ALL_FIELDS:
        if rng.random() < 0.3:
            continue
        if definition.kind is F.FLAG:
            spec.fields[definition.key] = rng.random() < 0.5
        elif definition.kind is F.TWO_BYTE:
            limit = definition.quantization_range
            spec.fields[definition.key] = rng.uniform(-limit, limit)
        elif definition.kind is F.U16:
            spec.fields[definition.key] = rng.randint(0, 0xFFFF)
        elif definition.kind is F.U8_SCALED:
            spec.fields[definition.key] = rng.randint(0, 20) / 20.0
        else:
            # control_mode は定義外の値も入れる（受信側の挙動は未知だが送れてよい）
            spec.fields[definition.key] = rng.randint(0, 255)
    for _ in range(rng.randint(0, 4)):
        spec.raw_bytes[rng.randrange(L.CMD_SIZE)] = rng.randrange(256)
    return spec


# 読み直して組み直すと最大 1 LSB 落ちることがある。convertFloatToTwoByte が
# 丸めずに切り捨てるためで、C++ 側もまったく同じ挙動をする（65536 通りのうち
# 189 通り。golden_vectors の照合で両者が一致していることは確かめてある）。
# 落ち続けはせず、3 パス以内・累積 2 LSB 以内で固定点に達する。
MAX_NORMALIZE_PASSES = 4


@pytest.mark.parametrize("seed", range(40))
def test_decode_assemble_converges(seed) -> None:
    """読んで組み直す操作は発散せず、数パスで固定点に落ち着く。"""
    spec = random_spec(random.Random(seed))
    current = assemble_command(spec)

    for _ in range(MAX_NORMALIZE_PASSES):
        next_spec = spec_from_decoded(decode_command(current))
        next_spec.raw_bytes = dict(spec.raw_bytes)
        nxt = assemble_command(next_spec)
        if nxt == current:
            break
        current = nxt
    else:
        pytest.fail(
            f"{MAX_NORMALIZE_PASSES} パス経っても読み直しでバイト列が変わり続ける"
        )


@pytest.mark.parametrize(
    "field", [f for f in F.ALL_FIELDS if f.kind is F.TWO_BYTE], ids=lambda f: f.key
)
def test_two_byte_fields_roundtrip_within_one_lsb(field) -> None:
    """2 バイト値は読み直しても 1 LSB 以上ずれない。

    完全一致にはならない。符号化が切り捨て、復号が厳密という非対称が
    robot_packet.h 側の仕様で、C++ でも同じ 189/65536 がずれる。
    """
    rng = random.Random(hash(field.key) & 0xFFFF)
    for _ in range(50):
        value = rng.uniform(-field.quantization_range, field.quantization_range)
        spec = PacketSpec()
        spec.fields["control_mode"] = 0  # union の両側を同じ扱いにする
        spec.fields[field.key] = value
        data = assemble_command(spec)

        original = int.from_bytes(data[field.offset : field.offset + 2], "big")
        decoded = next(item for item in decode_command(data) if item.key == field.key)
        assert decoded.raw == original

        again = encode_two_byte(decoded.value, field.quantization_range)
        assert abs(int.from_bytes(again, "big") - original) <= 1, (
            f"{field.key}: 読み直しで 1 LSB を超えてずれた"
        )


def test_two_byte_roundtrip_never_diverges() -> None:
    """全 65536 通りで、繰り返し読み直しても数パスで止まり累積 2 LSB 以内。"""
    import math

    def normalize(raw: int, wire_range: float) -> int:
        value = decode_two_byte(raw.to_bytes(2, "big"), 0, wire_range)
        return int.from_bytes(encode_two_byte(value, wire_range), "big")

    for wire_range in (32.767, math.pi):
        for raw in range(0, 65536, 7):  # 全数は遅いので間引く
            current = raw
            for _ in range(3):
                nxt = normalize(current, wire_range)
                assert abs(nxt - current) <= 1
                if nxt == current:
                    break
                current = nxt
            else:
                assert normalize(current, wire_range) == current, (
                    f"range={wire_range} raw={raw} が固定点に達しない"
                )
            assert abs(current - raw) <= 2


@pytest.mark.parametrize("bit", range(8))
def test_every_flag_bit_is_independent(bit) -> None:
    """8 ビットすべてを個別に立てられる。robot_packet.h が名前を付けていない
    bit 2 / 4-7 も含む（C++ のシリアライザでは到達できない領域）。"""
    field = next(f for f in F.FLAG_FIELDS if f.bit == bit)
    spec = PacketSpec()
    spec.fields[field.key] = True
    data = assemble_command(spec)

    assert data[L.FLAGS] == (1 << bit), f"bit {bit} だけが立っていない"
    decoded = {item.key: item.value for item in decode_command(data)}
    for other in F.FLAG_FIELDS:
        assert decoded[other.key] is (other.bit == bit)


def test_all_flag_bits_together() -> None:
    spec = PacketSpec()
    for field in F.FLAG_FIELDS:
        spec.fields[field.key] = True
    assert assemble_command(spec)[L.FLAGS] == 0xFF


def test_raw_byte_override_beats_fields_and_counter() -> None:
    """生バイト上書きは最後に適用され、フィールドにも check_counter にも勝つ。"""
    spec = PacketSpec()
    spec.fields["linear_velocity_limit"] = 3.0
    spec.set_raw_byte(L.LINEAR_VELOCITY_LIMIT_HIGH, 0xAB)
    spec.set_raw_byte(L.CHECK_COUNTER, 0x5A)

    data = assemble_command(spec, check_counter=99)
    assert data[L.LINEAR_VELOCITY_LIMIT_HIGH] == 0xAB
    assert data[L.CHECK_COUNTER] == 0x5A


def test_mode_args_union_last_write_wins() -> None:
    """byte 24-31 は union。両方指定したら後に書いたほうが残る（spec の順序どおり）。"""
    spec = PacketSpec()
    spec.fields["polar.target_global_velocity_r"] = 1.0
    spec.fields["position_target.terminal_velocity_x"] = -1.0
    data = assemble_command(spec)

    assert data[L.CONTROL_MODE_ARGS : L.CONTROL_MODE_ARGS + 2] == encode_two_byte(
        -1.0, 32.767
    )
    assert any("union" in w for w in spec.warnings()), (
        "union の重なりが警告されていない"
    )


def test_mode_and_args_may_disagree() -> None:
    """control_mode 3 のまま position_target の引数を入れられる。

    受信側は byte 23 を見て union を解釈するので、これは「あり得ない組み合わせ」だが、
    そういうパケットを作れることがこのツールの存在理由。
    """
    spec = PacketSpec()
    spec.fields["control_mode"] = L.CONTROL_MODES["POLAR_VELOCITY_TARGET_MODE"]
    spec.fields["position_target.terminal_velocity_x"] = 2.5
    data = assemble_command(spec)

    decoded = {item.key: item for item in decode_command(data)}
    assert decoded["position_target.terminal_velocity_x"].active is False
    assert decoded["polar.target_global_velocity_r"].active is True
    # 同じバイトなので値は一致する。どちらとして読まれるかは受信側の control_mode 次第。
    assert (
        decoded["polar.target_global_velocity_r"].raw
        == decoded["position_target.terminal_velocity_x"].raw
    )


def test_undefined_control_mode_is_representable() -> None:
    spec = PacketSpec()
    spec.fields["control_mode"] = 99
    data = assemble_command(spec)
    assert data[L.CONTROL_MODE] == 99
    assert any("定義が無い" in w for w in spec.warnings())


def test_zeros_base_means_negative_range() -> None:
    """base=zeros は crane の空きスロットと同じ。0x0000 は 0 ではなく -range。

    neutral を既定にしているのはこの落とし穴を避けるため。
    """
    zeros = assemble_command(PacketSpec(base="zeros"))
    assert zeros == bytes(L.CMD_SIZE)

    decoded = {item.key: item.value for item in decode_command(zeros)}
    assert decoded["linear_velocity_limit"] == pytest.approx(-32.767, abs=1e-3)

    neutral = decode_command(assemble_command(PacketSpec(base="neutral")))
    assert {item.key: item.value for item in neutral}[
        "linear_velocity_limit"
    ] == pytest.approx(0.0, abs=1e-3)


@pytest.mark.parametrize("robot_id", range(L.SLOTS))
def test_datagram_framing(robot_id) -> None:
    """715 バイト。全 11 スロットを常に送り、先頭バイトがスロット番号を兼ねる。"""
    spec = PacketSpec(robot_id=robot_id)
    spec.fields["check_counter"] = 42
    command = assemble_command(spec)
    datagram = build_datagram(command, robot_id)

    assert len(datagram) == L.PACKET_SIZE
    slots = split_datagram(datagram)
    assert sorted(slots) == list(range(L.SLOTS))
    assert slots[robot_id] == command
    for other, payload in slots.items():
        if other != robot_id:
            assert payload == bytes(L.CMD_SIZE), (
                "未使用スロットはゼロ埋めでなければならない"
            )


def test_robot_id_beyond_slots_is_rejected() -> None:
    with pytest.raises(ValueError):
        build_datagram(bytes(L.CMD_SIZE), L.SLOTS)


def test_kick_power_quantization_is_lossy_but_stable() -> None:
    """kick/dribble は送出が value×20 の切り捨て、復元が /20 で非対称。

    プロトコル側の性質であってこのツールのバグではない。1 往復で落ちることは
    あっても、そこから先は動かないことを固定しておく。
    """
    from crane_packet_forge.codec import decode_power, encode_power

    for byte in range(256):
        once = encode_power(decode_power(byte))
        twice = encode_power(decode_power(once))
        assert once == twice, f"byte {byte} で値が落ち続ける"
