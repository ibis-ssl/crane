# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Data modules for SSL commentary context."""

from .ssl_rules import SSL_RULES
from .team_profiles import TEAM_PROFILES, get_team_profile
from .initial_context import generate_initial_context

__all__ = [
    "SSL_RULES",
    "TEAM_PROFILES",
    "get_team_profile",
    "generate_initial_context",
]
