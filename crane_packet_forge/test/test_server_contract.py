# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""GUI が依存するサーバ側の約束事。

ページを実際に開いて分かった不具合の再発防止。spec の JSON から空の
raw_bytes / steps を落としていたため、GUI 側が undefined を掴んで
`Cannot use 'in' operator` で初期化ごと失敗していた。
"""

from pathlib import Path

import pytest
from crane_packet_forge.server import ForgeState, resolve_web_root
from crane_packet_forge.spec import PacketSpec

from crane_packet_forge import fields as F
from crane_packet_forge import layout as L


def test_spec_json_always_has_every_key() -> None:
    payload = PacketSpec().to_json()
    for key in (
        "target",
        "robot_id",
        "rate_hz",
        "base",
        "fields",
        "raw_bytes",
        "steps",
    ):
        assert key in payload, (
            f"空でも {key} を落とさないこと（GUI が undefined を掴む）"
        )


def test_spec_json_roundtrips_through_from_json() -> None:
    spec = PacketSpec(target="sim", robot_id=4)
    spec.set_field("control_mode", 3)
    spec.set_raw_byte(40, 0xAB)
    restored = PacketSpec.from_json(spec.to_json())
    assert restored.to_json() == spec.to_json()


def test_snapshot_shape_matches_what_the_page_reads() -> None:
    snapshot = ForgeState().snapshot()
    for key in (
        "spec",
        "address",
        "command_hex",
        "fields",
        "warnings",
        "conflicts",
        "sender",
    ):
        assert key in snapshot
    assert len(snapshot["command_hex"]) == L.CMD_SIZE * 2
    assert {item["key"] for item in snapshot["fields"]} == {f.key for f in F.ALL_FIELDS}


def test_flags_byte_has_an_owner() -> None:
    """byte 22 が「未使用」に見えると GUI が薄く表示してしまう。"""
    owner = F.owner_of_byte(L.FLAGS)
    assert owner is not None
    assert owner.group == "flags"
    assert "flags" in F.describe_byte(L.FLAGS)


def test_web_root_contains_the_page() -> None:
    root = resolve_web_root()
    assert (root / "index.html").is_file()
    assert (root / "forge.js").is_file()


def test_web_root_falls_back_to_the_ament_share(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """install 空間では web はモジュールの隣ではなく share にある。

    --symlink-install だと __file__ がソースツリーへ戻るのでこの経路を通らない。
    素の install で壊れないよう、候補から外れていないことを見る。
    """
    from crane_packet_forge import server

    monkeypatch.setattr(server, "DEFAULT_WEB_ROOT", Path("/nonexistent/web"))
    monkeypatch.delenv("WEB_ROOT", raising=False)
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("crane_packet_forge")) / "web"
    except Exception:  # noqa: BLE001
        pytest.skip("ament が無い環境では share 側を確かめられない")

    if not (share / "index.html").is_file():
        pytest.skip("install 空間に web が無い（未ビルド）")
    assert server.resolve_web_root() == share
