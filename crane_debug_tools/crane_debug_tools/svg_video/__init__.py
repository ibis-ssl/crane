"""SVG video generation module for RoboCup SSL matches.

このモジュールは、MCAPファイルからSVGメッセージを抽出し、動画を生成します。
"""

from .svg_assembler import SvgAssembler
from .svg_extractor import SvgExtractor
from .svg_renderer import (
    SvgRenderer,
    create_renderer,
    list_available_backends,
)
from .video_generator import VideoGenerator

__all__ = [
    "SvgExtractor",
    "SvgAssembler",
    "SvgRenderer",
    "VideoGenerator",
    "create_renderer",
    "list_available_backends",
]
