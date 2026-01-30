"""SVG rendering backends for video generation.

このパッケージは、複数のSVGレンダリングバックエンドを統一インターフェースで提供します。
"""

from .base import OutputFormat, SvgRendererBase
from .cairosvg_renderer import CairoSvgRenderer
from .resvg_renderer import ResvgPyRenderer
from .rsvg_renderer import RsvgCliRenderer

__all__ = [
    "SvgRendererBase",
    "OutputFormat",
    "CairoSvgRenderer",
    "ResvgPyRenderer",
    "RsvgCliRenderer",
]
