# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""mcap_analysis の特性テスト（MCAP・Gemini API は使わず差し替える）."""

import sys
from types import ModuleType, SimpleNamespace

import pytest
from crane_mcap_tools.mcap_analysis import extractor, gemini_client
from crane_mcap_tools.mcap_analysis.extractor import (
    AnnotationContext,
    MCAPAnnotationExtractor,
    WorldModelSnapshot,
)
from crane_mcap_tools.mcap_analysis.gemini_client import GeminiAnalysisClient
from crane_mcap_tools.mcap_analysis.mcap_tools import MCAPToolsHandler

SEC = 1_000_000_000
EVENT_NS = 10 * SEC


def robot(robot_id, x, y, vx=0.0, vy=0.0):
    return {
        "id": robot_id,
        "position": (x, y),
        "theta": 0.0,
        "velocity": (vx, vy),
    }


def snapshot(
    t_ns, ball_pos=(0.0, 0.0, 0.0), ball_vel=(0.0, 0.0, 0.0), ours=(), theirs=()
):
    return WorldModelSnapshot(
        timestamp_ns=t_ns,
        ball_position=ball_pos,
        ball_velocity=ball_vel,
        our_robots=list(ours),
        their_robots=list(theirs),
    )


def annotation(snapshots):
    return AnnotationContext(
        category=0,
        priority=0,
        label="l",
        description="d",
        event_timestamp_ns=EVENT_NS,
        has_position=False,
        world_model_context=list(snapshots),
    )


@pytest.fixture
def handler():
    snapshots = [
        snapshot(
            EVENT_NS - SEC,
            ball_pos=(0.0, 0.0, 0.0),
            ball_vel=(3.0, 4.0, 0.0),
            ours=[robot(1, 0.0, 0.0, 1.0, 0.0), robot(2, 0.2, 0.0)],
            theirs=[robot(1, 5.0, 0.0)],
        ),
        snapshot(
            EVENT_NS,
            ball_pos=(4.0, 0.0, 0.0),
            ball_vel=(0.0, 0.0, 0.0),
            ours=[robot(1, 3.0, 4.0, 0.0, 2.0), robot(2, 2.0, 0.0)],
            theirs=[robot(1, 4.1, 0.0)],
        ),
    ]
    return MCAPToolsHandler(annotation(snapshots))


def test_handle_unknown_function_returns_error(handler):
    assert handler.handle("no_such_tool", {}) == {
        "error": "Unknown function: no_such_tool"
    }


def test_handle_turns_handler_exception_into_error():
    result = MCAPToolsHandler(annotation([])).handle("get_world_model_at_time", {})
    assert set(result) == {"error"}


def test_world_model_at_time_picks_snapshot_closest_to_offset(handler):
    result = handler.handle("get_world_model_at_time", {"time_offset_sec": -0.8})
    assert result["timestamp_sec"] == 9.0
    assert result["ball_velocity"] == (3.0, 4.0, 0.0)


def test_robot_trajectory_selects_team_and_sums_distance(handler):
    ours = handler.handle("get_robot_trajectory", {"robot_id": 1, "is_ours": True})
    assert [p["position"] for p in ours["trajectory"]] == [(0.0, 0.0), (3.0, 4.0)]
    assert ours["total_distance"] == pytest.approx(5.0)

    theirs = handler.handle("get_robot_trajectory", {"robot_id": 1, "is_ours": False})
    assert [p["position"] for p in theirs["trajectory"]] == [(5.0, 0.0), (4.1, 0.0)]


def test_ball_trajectory_reports_max_speed(handler):
    result = handler.handle("get_ball_trajectory", {})
    assert result["total_distance"] == pytest.approx(4.0)
    assert result["max_speed"] == pytest.approx(5.0)


def test_calculate_distance(handler):
    assert handler.handle(
        "calculate_distance", {"point1": [0, 0], "point2": [3, 4]}
    ) == {"distance": 5.0}
    assert handler.handle("calculate_distance", {"point1": [0], "point2": [3, 4]}) == {
        "error": "Invalid points"
    }


def test_robot_speed_history(handler):
    result = handler.handle("get_robot_speed_history", {"robot_id": 1})
    assert [s["speed"] for s in result["speed_history"]] == [1.0, 2.0]
    assert (result["max_speed"], result["min_speed"], result["avg_speed"]) == (
        2.0,
        1.0,
        1.5,
    )
    assert handler.handle("get_robot_speed_history", {"robot_id": 9}) == {
        "error": "Robot not found in snapshots"
    }


def test_ball_speed_history(handler):
    result = handler.handle("get_ball_speed_history", {})
    assert [s["speed"] for s in result["speed_history"]] == [5.0, 0.0]
    assert result["avg_speed"] == 2.5


def test_find_closest_robot_to_ball_by_team(handler):
    both = handler.handle("find_closest_robot_to_ball", {"team": "both"})
    assert (both["robot_id"], both["is_ours"]) == (1, False)
    assert both["distance"] == pytest.approx(0.1)

    ours = handler.handle("find_closest_robot_to_ball", {"team": "ours"})
    assert (ours["robot_id"], ours["is_ours"], ours["distance"]) == (2, True, 2.0)


def test_check_robot_collision(handler):
    result = handler.handle("check_robot_collision", {"threshold_distance": 0.3})
    assert result["collision_count"] == 1
    assert result["collisions"][0]["both_ours"] is True
    assert (
        result["collisions"][0]["robot1_id"],
        result["collisions"][0]["robot2_id"],
    ) == (1, 2)


@pytest.fixture
def fake_genai(monkeypatch):
    """google-genai SDK の代わりに、Client の生成引数を記録するだけのモジュールを入れる."""
    created = []
    genai = ModuleType("google.genai")
    types = ModuleType("google.genai.types")
    genai.types = types
    genai.Client = lambda **kwargs: created.append(kwargs) or SimpleNamespace()
    monkeypatch.setitem(sys.modules, "google.genai", genai)
    monkeypatch.setitem(sys.modules, "google.genai.types", types)
    return SimpleNamespace(created=created, types=types)


def test_client_init_creates_sdk_client_with_api_key(fake_genai):
    client = GeminiAnalysisClient(api_key="key", rate_limit_delay=0.25)
    assert fake_genai.created == [{"api_key": "key"}]
    assert client._types is fake_genai.types


@pytest.mark.parametrize("count", [0, 1, 3])
def test_batch_with_tools_keeps_order_and_sleeps_between_items(
    fake_genai, monkeypatch, count
):
    client = GeminiAnalysisClient(api_key="key", rate_limit_delay=0.25)
    calls = []
    sleeps = []
    monkeypatch.setattr(
        client,
        "analyze_annotation_with_tools",
        lambda *args: calls.append(args) or f"result-{len(calls)}",
    )
    monkeypatch.setattr(gemini_client.time, "sleep", sleeps.append)

    annotations = [f"a{i}" for i in range(count)]
    prompts = [(f"p{i}", f"s{i}") for i in range(count)]
    results = client.analyze_batch_with_tools(annotations, prompts, max_tool_calls=7)

    assert results == [f"result-{i + 1}" for i in range(count)]
    assert calls == [(f"a{i}", f"p{i}", f"s{i}", 7) for i in range(count)]
    assert sleeps == [0.25] * max(count - 1, 0)


class FakeReader:
    def __init__(self, topics, messages):
        self._topics = topics
        self._messages = list(messages)

    def get_all_topics_and_types(self):
        return [SimpleNamespace(name=n, type=t) for n, t in self._topics.items()]

    def has_next(self):
        return bool(self._messages)

    def read_next(self):
        return self._messages.pop(0)


def world_msg(ball_x):
    vec = SimpleNamespace(x=ball_x, y=0.0, z=0.0)
    return SimpleNamespace(
        ball_info=SimpleNamespace(position=vec, velocity=vec),
        robot_info_ours=[],
        robot_info_theirs=[],
    )


def annotation_msg():
    return SimpleNamespace(
        event_timestamp_ns=EVENT_NS,
        has_position=False,
        has_robot_context=False,
        category=3,
        priority=1,
        label="l",
        description="d",
    )


def test_extract_from_mcap_resolves_types_with_cache_and_samples_window(monkeypatch):
    topics = {
        "/human_annotations": "Annotation",
        "/world_model": "WorldModel",
        "/other": "X",
    }
    payloads = {
        "ann": annotation_msg(),
        "w0": world_msg(0.0),
        "w1": world_msg(1.0),
        "w2": world_msg(2.0),
        "w3": world_msg(3.0),
    }
    reader = FakeReader(
        topics,
        [
            ("/world_model", "w0", EVENT_NS - 4 * SEC),
            ("/world_model", "w1", EVENT_NS - SEC),
            ("/other", "x", EVENT_NS),
            ("/human_annotations", "ann", EVENT_NS),
            ("/world_model", "w2", EVENT_NS + SEC),
            ("/world_model", "w3", EVENT_NS + 3 * SEC),
        ],
    )
    type_requests = []

    def fake_get_message_type(type_name, cache):
        type_requests.append((type_name, cache))
        return type_name

    monkeypatch.setattr(extractor, "resolve_mcap_path", lambda path: path)
    monkeypatch.setattr(extractor, "open_sequential_reader", lambda path: reader)
    monkeypatch.setattr(extractor, "get_message_type", fake_get_message_type)
    monkeypatch.setattr(
        extractor, "deserialize_message", lambda data, _: payloads[data]
    )

    ex = MCAPAnnotationExtractor(context_before_sec=3.0, context_after_sec=2.0)
    contexts = ex.extract_from_mcap("dummy.mcap")

    assert [name for name, _ in type_requests] == [
        "WorldModel",
        "WorldModel",
        "Annotation",
        "WorldModel",
        "WorldModel",
    ]
    assert all(cache is ex._msg_types for _, cache in type_requests)
    assert len(contexts) == 1
    assert contexts[0].category == 3
    assert [s.ball_position[0] for s in contexts[0].world_model_context] == [1.0, 2.0]
