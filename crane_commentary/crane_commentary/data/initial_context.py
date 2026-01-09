# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Initial Context Generator for Gemini Live API."""

import json
from typing import Optional, Dict, Any


def get_team_profile_from_data(
    team_name: str, team_profiles: Dict[str, Any]
) -> Dict[str, Any]:
    """
    Get team profile by name with fuzzy matching from provided data.

    Args:
        team_name: Team name to search for
        team_profiles: Dictionary of team profiles

    Returns:
        Dictionary containing team profile information
    """
    profiles = team_profiles.get("profiles", {})
    default_profile = team_profiles.get("default_profile", {})

    # Exact match
    if team_name in profiles:
        return profiles[team_name]

    # Case-insensitive partial match
    name_lower = team_name.lower()
    for key, profile in profiles.items():
        if key.lower() in name_lower or name_lower in key.lower():
            return profile

    # Return default profile if no match found
    return default_profile


def get_team_reading_from_data(team_key: str, team_profiles: Dict[str, Any]) -> str:
    """Get team name reading from provided data."""
    profile = get_team_profile_from_data(team_key, team_profiles)
    return profile.get("reading", team_key)


def generate_initial_context(
    ssl_rules: Dict[str, Any],
    team_profiles: Dict[str, Any],
    our_team_name: str = "ibis",
    their_team_name: Optional[str] = None,
) -> str:
    """
    Generate initial context JSON for commentary session.

    Args:
        ssl_rules: Dictionary of SSL rules
        team_profiles: Dictionary of team profiles
        our_team_name: Name of our team (default: "ibis")
        their_team_name: Name of opponent team (optional)

    Returns:
        JSON string containing initial context
    """
    # Build context dictionary
    context = {
        "type": "initial_context",
        "ssl_rules": {
            "summary": "RoboCup Small Size League の基本ルール",
            "field": ssl_rules["basic_info"]["field_size"],
            "robots": ssl_rules["basic_info"]["robots"],
            "match_duration": ssl_rules["basic_info"]["match_duration"],
            "speed_limits": {
                "ball_speed_limit_mps": ssl_rules["basic_info"]["ball_speed_limit"],
                "robot_speed_in_stop_mps": ssl_rules["basic_info"][
                    "robot_speed_in_stop"
                ],
            },
            "key_fouls": [
                {"name": name, **details}
                for name, details in list(ssl_rules["fouls"].items())[:8]
            ],
            "set_plays": ssl_rules["set_plays"],
        },
        "our_team": {
            "name": get_team_reading_from_data(
                our_team_name, team_profiles
            ),  # 読み方を使用
            "key": our_team_name,  # キーも保持
            **get_team_profile_from_data(our_team_name, team_profiles),
        },
    }

    # Add opponent team info if available
    if their_team_name:
        context["their_team"] = {
            "name": get_team_reading_from_data(
                their_team_name, team_profiles
            ),  # 読み方を使用
            "key": their_team_name,  # キーも保持
            **get_team_profile_from_data(their_team_name, team_profiles),
        }
    else:
        context["their_team"] = {
            "name": "未定",
            "note": "相手チーム情報は試合開始時に更新されます",
        }

    # Add commentary hints
    our_team_reading = get_team_reading_from_data(our_team_name, team_profiles)
    context["commentary_hints"] = {
        "our_team_focus": f"{our_team_reading}の特徴を活かした実況を心がけてください",
        "opponent_analysis": "相手チームの特徴も適宜言及してください",
        "rule_awareness": "ファール発生時は該当ルールを簡潔に説明してください",
        "style_note": "簡潔で熱のこもった実況を続けてください",
    }

    # Convert to JSON string with Japanese support
    return json.dumps(context, ensure_ascii=False, indent=2)
