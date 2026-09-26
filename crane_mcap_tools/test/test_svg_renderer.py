# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""create_renderer のテスト."""

import pytest
from crane_mcap_tools.svg_video.renderers import ResvgPyRenderer
from crane_mcap_tools.svg_video.svg_renderer import create_renderer


def test_unavailable_backend_suggests_its_install_command(monkeypatch):
    monkeypatch.setattr(ResvgPyRenderer, "is_available", classmethod(lambda cls: False))

    with pytest.raises(ValueError, match="pip install resvg-py"):
        create_renderer("resvg")
