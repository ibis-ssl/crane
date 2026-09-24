# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""SvgExtractor のレイヤー累積を固定する特性テスト（rosbag2 は使わず reader を差し替える）."""

from types import SimpleNamespace

import pytest
from crane_mcap_tools.svg_video import svg_extractor
from crane_mcap_tools.svg_video.svg_extractor import SvgExtractor

SNAP = "/aggregated_svgs"
UPD = "/visualizer_svgs"


def snapshot(epoch, seq, **layers):
    return SimpleNamespace(
        epoch=epoch,
        seq=seq,
        layers=[SimpleNamespace(layer=k, svg_primitives=v) for k, v in layers.items()],
    )


def update(epoch, seq, *ops):
    return SimpleNamespace(
        epoch=epoch,
        seq=seq,
        updates=[
            SimpleNamespace(layer=layer, operation=op, svg_primitives=prims)
            for layer, op, prims in ops
        ],
    )


class FakeReader:
    def __init__(self, topics, records):
        self._topics = topics
        self._records = list(records)

    def get_all_topics_and_types(self):
        return [SimpleNamespace(name=n, type=t) for n, t in self._topics.items()]

    def has_next(self):
        return bool(self._records)

    def read_next(self):
        return self._records.pop(0)


@pytest.fixture
def bag(monkeypatch):
    """(topic, msg, timestamp_ns) のリストを MCAP の中身として差し込む."""
    type_lookups = []

    def install(records, topics=None):
        topics = topics or {SNAP: "snap_type", UPD: "upd_type", "/other": "x"}
        reader = FakeReader(topics, [(t, msg, ts) for t, msg, ts in records])
        monkeypatch.setattr(
            svg_extractor, "open_sequential_reader", lambda path: reader
        )

        def fake_get_message_type(type_name, cache):
            type_lookups.append(type_name)
            return type_name

        monkeypatch.setattr(svg_extractor, "get_message_type", fake_get_message_type)
        monkeypatch.setattr(svg_extractor, "deserialize_message", lambda data, t: data)
        return type_lookups

    return install


def extract(**kwargs):
    return list(SvgExtractor().extract_from_mcap("dummy.mcap", **kwargs))


def layers_of(frames):
    return [f.layers for f in frames]


def test_snapshot_replaces_all_layers(bag):
    bag(
        [
            (SNAP, snapshot(1, 0, a=["<a/>"], b=["<b/>"]), 10),
            (SNAP, snapshot(1, 1, c=["<c/>"]), 20),
        ]
    )

    frames = extract()

    assert [(f.timestamp_ns, f.epoch, f.seq) for f in frames] == [
        (10, 1, 0),
        (20, 1, 1),
    ]
    assert layers_of(frames) == [{"a": ["<a/>"], "b": ["<b/>"]}, {"c": ["<c/>"]}]


def test_update_operations(bag):
    bag(
        [
            (SNAP, snapshot(1, 0, a=["<a1/>"], b=["<b/>"]), 10),
            (
                UPD,
                update(1, 1, ("a", "append", ["<a2/>"]), ("new", "append", ["<n/>"])),
                20,
            ),
            (UPD, update(1, 2, ("b", "REPLACE", ["<b2/>"])), 30),
            (UPD, update(1, 3, ("a", "clear", [])), 40),
            (UPD, update(1, 4, ("b", "remove", ["<x/>"])), 50),
        ]
    )

    assert layers_of(extract()) == [
        {"a": ["<a1/>"], "b": ["<b/>"]},
        {"a": ["<a1/>", "<a2/>"], "b": ["<b/>"]},
        {"a": ["<a1/>", "<a2/>"], "b": ["<b2/>"]},
        {"a": [], "b": ["<b2/>"]},
        {"a": [], "b": ["<b2/>"]},
    ]


def test_epoch_change_resets_state_before_applying(bag):
    bag(
        [
            (UPD, update(1, 0, ("a", "replace", ["<a/>"])), 10),
            (UPD, update(2, 0, ("b", "replace", ["<b/>"])), 20),
            (UPD, update(2, 1, ("b", "append", ["<b2/>"])), 30),
        ]
    )

    frames = extract()

    assert [f.epoch for f in frames] == [1, 2, 2]
    assert layers_of(frames) == [
        {"a": ["<a/>"]},
        {"b": ["<b/>"]},
        {"b": ["<b/>", "<b2/>"]},
    ]


def test_messages_are_sorted_and_other_topics_ignored(bag):
    type_lookups = bag(
        [
            (UPD, update(0, 1, ("a", "append", ["<a2/>"])), 20),
            ("/other", None, 15),
            (SNAP, snapshot(0, 0, a=["<a1/>"]), 10),
        ]
    )

    frames = extract()

    assert [f.timestamp_ns for f in frames] == [10, 20]
    assert layers_of(frames) == [{"a": ["<a1/>"]}, {"a": ["<a1/>", "<a2/>"]}]
    assert sorted(type_lookups) == ["snap_type", "upd_type"]


def test_time_range_skips_before_start_and_stops_after_end(bag):
    bag(
        [
            (UPD, update(0, 0, ("a", "replace", ["<early/>"])), 1_000_000_000),
            (UPD, update(0, 1, ("a", "append", ["<in/>"])), 2_000_000_000),
            (UPD, update(0, 2, ("b", "replace", ["<in2/>"])), 3_000_000_000),
            (UPD, update(0, 3, ("c", "replace", ["<late/>"])), 3_000_000_001),
            (UPD, update(0, 4, ("d", "replace", ["<after/>"])), 2_500_000_000),
        ]
    )

    frames = extract(start_time_sec=1.5, end_time_sec=3.0)

    assert [f.seq for f in frames] == [1, 2]
    assert layers_of(frames) == [{}, {"b": ["<in2/>"]}]


def test_yielded_frames_are_independent_copies(bag):
    bag(
        [
            (UPD, update(0, 0, ("a", "replace", ["<a1/>"])), 10),
            (UPD, update(0, 1, ("a", "append", ["<a2/>"])), 20),
        ]
    )

    first, second = extract()

    assert first.layers == {"a": ["<a1/>"]}
    assert second.layers == {"a": ["<a1/>", "<a2/>"]}


def test_bag_without_svg_topics_is_rejected(bag):
    bag([], topics={"/other": "x"})

    with pytest.raises(
        ValueError, match="Neither /aggregated_svgs nor /visualizer_svgs"
    ):
        extract()
