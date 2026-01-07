# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Initial Context Generator for Gemini Live API."""

import json
from typing import Optional

from .ssl_rules import SSL_RULES
from .team_profiles import get_team_profile, get_team_reading


def generate_initial_context(
    our_team_name: str = "ibis",
    their_team_name: Optional[str] = None,
) -> str:
    """
    Generate initial context JSON for commentary session.

    This context is sent once after Gemini API connection is established,
    providing SSL rules and team information to enable context-aware commentary.

    Args:
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
            "field": SSL_RULES["basic_info"]["field_size"],
            "robots": SSL_RULES["basic_info"]["robots"],
            "match_duration": SSL_RULES["basic_info"]["match_duration"],
            "speed_limits": {
                "ball_speed_limit_mps": SSL_RULES["basic_info"]["ball_speed_limit"],
                "robot_speed_in_stop_mps": SSL_RULES["basic_info"][
                    "robot_speed_in_stop"
                ],
            },
            "key_fouls": [
                {"name": name, **details}
                for name, details in list(SSL_RULES["fouls"].items())[:8]
            ],
            "set_plays": SSL_RULES["set_plays"],
        },
        "our_team": {
            "name": get_team_reading(our_team_name),  # 読み方を使用
            "key": our_team_name,  # キーも保持
            **get_team_profile(our_team_name),
        },
    }

    # Add opponent team info if available
    if their_team_name:
        context["their_team"] = {
            "name": get_team_reading(their_team_name),  # 読み方を使用
            "key": their_team_name,  # キーも保持
            **get_team_profile(their_team_name),
        }
    else:
        context["their_team"] = {
            "name": "未定",
            "note": "相手チーム情報は試合開始時に更新されます",
        }

    # Add commentary hints
    our_team_reading = get_team_reading(our_team_name)
    context["commentary_hints"] = {
        "our_team_focus": f"{our_team_reading}の特徴を活かした実況を心がけてください",
        "opponent_analysis": "相手チームの特徴も適宜言及してください",
        "rule_awareness": "ファール発生時は該当ルールを簡潔に説明してください",
        "style_note": "簡潔で熱のこもった実況を続けてください",
    }

    # Convert to JSON string with Japanese support
    return json.dumps(context, ensure_ascii=False, indent=2)
