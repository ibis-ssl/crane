# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Gemini Multimodal Live API client for real-time audio generation."""

from .live_api_client import GeminiLiveApiClient
from .function_handler import FunctionHandler
from .function_declarations import FUNCTION_DECLARATIONS

__all__ = ["GeminiLiveApiClient", "FunctionHandler", "FUNCTION_DECLARATIONS"]
