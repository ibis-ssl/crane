# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Data modules for SSL commentary context."""

from .initial_context import (
    generate_initial_context,
    get_team_profile_from_data,
    get_team_reading_from_data,
)

__all__ = [
    "generate_initial_context",
    "get_team_profile_from_data",
    "get_team_reading_from_data",
]
