# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""SSL Team Profiles for Commentary System."""

from typing import Any

# Known SSL teams with their characteristics
TEAM_PROFILES: dict[str, dict[str, Any]] = {
    # 世界強豪チーム
    "TIGERs Mannheim": {
        "reading": "タイガース",
        "country": "ドイツ",
        "characteristics": [
            "最強チーム",
            "華麗なパス回し",
        ],
        "style": "パス主体",
        "famous_for": "他チームを寄せ付けない圧倒的な試合運び",
    },
    "ER-Force": {
        "reading": "イーアール・フォース",
        "country": "ドイツ",
        "characteristics": [
            "世界大会上位常連",
            "独自シミュレータ開発",
        ],
        "style": "バランス型",
        "famous_for": "高度なシミュレーション技術と安定した戦術",
    },
    "ZJUNLict": {
        "reading": "ジュンリクト",
        "country": "中国",
        "characteristics": [
            "浙江大学の強豪チーム",
            "強力なドリブル",
            "日本大会に来ることがある",
        ],
        "style": "パス主体",
        "famous_for": "リアルタイム動的パスポイント探索による高度なパス戦術",
    },
    "RoboCin": {
        "reading": "ロボシン",
        "country": "ブラジル",
        "characteristics": [
            "南米強豪チーム",
        ],
        "style": "創造的",
        "famous_for": "独創的なプレースタイル",
    },
    # 日本チーム
    "ibis": {
        "reading": "アイビス",
        "country": "日本",
        "characteristics": [
            "プライベートチーム",
            "人数が少ない（4人）",
            "ジャパンオープン2024でロボット学会賞",
            "ジャパンオープン2025で三位",
            "ロボットの高さが低い",
        ],
        "style": "技術重視",
        "famous_for": "高度な衝突回避アルゴリズムとイベント検出システム",
    },
    "RoboDragons": {
        "reading": "ロボドラゴンズ",
        "country": "日本",
        "characteristics": [
            "愛知県立大学チーム",
            "日本の強豪チーム",
            "最近沢山の新入生が入った",
            "MPCを使っている",
            "時々プログラムが落ちる",
        ],
        "style": "創造的",
        "famous_for": "かつての最強チーム、CMDragonsとの共同開発のベースをもつ",
    },
    "Roots": {
        "reading": "ルーツ",
        "country": "日本",
        "characteristics": [
            "consai_ros2フレームワーク開発",
            "オープンソースへの貢献",
            "日本SSLコミュニティの中心的存在",
        ],
        "style": "コミュニティ重視",
        "famous_for": "ROS2ベースSSLフレームワークの開発と普及",
    },
    "Ri-one": {
        "reading": "リオン",
        "country": "日本",
        "characteristics": [
            "立命館大学チーム",
            "世界大会にも出場したことのあるチーム",
        ],
        "style": "シンプル",
        "famous_for": "Pythonによる軽量かつ理解しやすいSSLフレームワーク",
    },
    "KIKS": {
        "reading": "キックス",
        "country": "日本",
        "characteristics": [
            "豊田高専チーム",
            "日本の強豪チーム",
            "最近は長年のライバルRoboDragonsに勝つことが多い",
            "最近、Javaでプログラムを書き直した",
            "ロボットの動きがきれい",
        ],
        "style": "シンプル",
        "famous_for": "Pythonによる軽量かつ理解しやすいSSLフレームワーク",
    },
    # その他デフォルト
    "opponent": {
        "reading": "相手チーム",
        "country": "不明",
        "characteristics": ["対戦相手"],
        "style": "不明",
        "famous_for": "分析データなし",
    },
    "unknown": {
        "reading": "不明なチーム",
        "country": "不明",
        "characteristics": ["新規参入チームまたは情報なし"],
        "style": "不明",
        "famous_for": "分析データなし",
    },
}

# Default profile for unknown teams
DEFAULT_PROFILE: dict[str, Any] = {
    "reading": "不明なチーム",
    "country": "不明",
    "characteristics": ["新規参入チームまたは情報なし"],
    "style": "不明",
    "famous_for": "分析データなし",
}


def get_team_profile(team_name: str) -> dict[str, Any]:
    """
    Get team profile by name with fuzzy matching.

    Args:
        team_name: Team name to search for

    Returns:
        Dictionary containing team profile information
    """
    # Exact match
    if team_name in TEAM_PROFILES:
        return TEAM_PROFILES[team_name]

    # Case-insensitive partial match
    name_lower = team_name.lower()
    for key, profile in TEAM_PROFILES.items():
        if key.lower() in name_lower or name_lower in key.lower():
            return profile

    # Return default profile if no match found
    return DEFAULT_PROFILE


def get_team_reading(team_key: str) -> str:
    """
    Get team name reading (pronunciation) from team key.

    Args:
        team_key: Team key (e.g., "ibis", "TIGERs Mannheim")

    Returns:
        Team name reading in katakana/hiragana (e.g., "アイビス", "タイガース・マンハイム")
    """
    profile = get_team_profile(team_key)
    return profile.get("reading", team_key)
