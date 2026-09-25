# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""CLI の入口を実際に通す。

`_confirm` が警告ありの経路で落ちる不具合（EventLog に無いメソッドを呼んでいた）を
mypy が見つけた。型検査だけに頼らず、警告の出る経路を実行して押さえておく。
"""

import argparse
import json

import pytest
from crane_packet_forge.cli import build_parser, main, parse_schedule
from crane_packet_forge.events import EventLog
from crane_packet_forge.spec import PacketSpec, SpecError

from crane_packet_forge import events


def test_dry_run_json_shape(capsys) -> None:
    code = main(
        [
            "send",
            "--robot",
            "7",
            "--dry-run",
            "--json",
            "--set",
            "control_mode=3",
            "--set",
            "polar.target_global_velocity_r=0.3",
        ]
    )
    assert code == events.EXIT_OK

    payload = json.loads(capsys.readouterr().out)
    assert payload["address"] == "192.168.20.107"
    assert len(payload["command_hex"]) == 128  # 64 バイト
    assert payload["datagram_size"] == 715
    assert any(f["key"] == "polar.target_global_velocity_r" for f in payload["fields"])


def test_dry_run_opens_no_socket_even_with_unreachable_target(capsys) -> None:
    """--dry-run はソケットを開かない。届かない宛先でも成功する。"""
    code = main(["send", "--target", "203.0.113.1", "--dry-run", "--json"])
    assert code == events.EXIT_OK
    capsys.readouterr()


def test_confirm_path_with_warnings_does_not_crash() -> None:
    """警告がある経路を実際に通す。非 tty なので確認待ちにはならない。"""
    from crane_packet_forge.cli import _confirm

    spec = PacketSpec()
    spec.fields["control_mode"] = 4  # 旧 CM4 で暴走する旨の警告が出る
    assert spec.warnings()

    log = EventLog(quiet=True)
    assert _confirm(spec, log, skip=True) is True


def test_unknown_field_exits_with_usage_code(capsys) -> None:
    assert main(["send", "--set", "polar.r=1", "--dry-run"]) == events.EXIT_USAGE
    assert "未知のフィールド" in capsys.readouterr().err


def test_robot_id_above_slot_count_is_rejected(capsys) -> None:
    assert main(["send", "--robot", "11", "--dry-run"]) == events.EXIT_USAGE
    capsys.readouterr()


def test_fields_subcommand_lists_every_key(capsys) -> None:
    from crane_packet_forge import fields as F

    assert main(["fields"]) == events.EXIT_OK
    out = capsys.readouterr().out
    for definition in F.ALL_FIELDS:
        assert definition.key in out


def test_decode_roundtrips_dry_run_output(capsys) -> None:
    main(
        [
            "send",
            "--set",
            "control_mode=3",
            "--set",
            "polar.target_global_velocity_r=1.75",
            "--dry-run",
            "--json",
        ]
    )
    command_hex = json.loads(capsys.readouterr().out)["command_hex"]

    assert main(["decode", "--hex", command_hex, "--json"]) == events.EXIT_OK
    decoded = json.loads(capsys.readouterr().out)
    value = next(f for f in decoded if f["key"] == "polar.target_global_velocity_r")
    assert value["value"] == pytest.approx(1.75, abs=1e-3)


def test_schedule_parsing() -> None:
    steps = parse_schedule(
        "polar.target_global_velocity_r=0:8;"
        "polar.target_global_velocity_r=0.2,flags.is_vision_available=1:2"
    )
    assert [s.duration_s for s in steps] == [8.0, 2.0]
    assert steps[1].fields["flags.is_vision_available"] is True


def test_schedule_help_example_parses() -> None:
    sub = next(
        a for a in build_parser()._actions if isinstance(a, argparse._SubParsersAction)
    )
    send = sub.choices["send"]
    help_text = next(a.help for a in send._actions if a.dest == "schedule")
    example = help_text.split("'")[1]
    assert [s.duration_s for s in parse_schedule(example)] == [8.0, 2.0]


def test_schedule_rejects_missing_duration() -> None:
    with pytest.raises(SpecError):
        parse_schedule("polar.target_global_velocity_r=0")
