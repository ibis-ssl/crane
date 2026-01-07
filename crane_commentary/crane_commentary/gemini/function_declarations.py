# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Function declarations for Gemini Live API Function Calling."""

# Function declarations for Gemini API tools configuration
FUNCTION_DECLARATIONS = [
    {
        "name": "get_game_state",
        "description": "現在の試合状態を取得する。スコア、経過時間、プレイ状況（INPLAY/HALT/STOP等）、"
        "勢い（どちらのチームが押しているか）を含む。",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": [],
        },
    },
    {
        "name": "get_ball_trajectory",
        "description": "ボールの直近の軌跡を取得する。現在位置・速度と、過去数秒間の位置履歴を含む。"
        "ボールの動きを分析する際に使用。",
        "parameters": {
            "type": "object",
            "properties": {
                "seconds": {
                    "type": "number",
                    "description": "取得する過去の秒数（デフォルト: 3秒、最大: 10秒）",
                }
            },
            "required": [],
        },
    },
    {
        "name": "get_robot_status",
        "description": "指定したロボットの詳細状態を取得する。位置、速度、ボール接触状態、"
        "ゴールキーパーかどうか、役割のヒントを含む。",
        "parameters": {
            "type": "object",
            "properties": {
                "robot_id": {
                    "type": "integer",
                    "description": "ロボットID（0-15）",
                },
                "is_ours": {
                    "type": "boolean",
                    "description": "自チーム（ibis）のロボットかどうか",
                },
            },
            "required": ["robot_id", "is_ours"],
        },
    },
    {
        "name": "get_all_robots_summary",
        "description": "全ロボットの概要を取得する。各チームのアクティブロボット数、"
        "ゴールキーパーID、各ロボットの位置ゾーン、陣形サマリーを含む。",
        "parameters": {
            "type": "object",
            "properties": {
                "team": {
                    "type": "string",
                    "enum": ["ours", "theirs", "all"],
                    "description": "対象チーム（デフォルト: all）",
                }
            },
            "required": [],
        },
    },
]
