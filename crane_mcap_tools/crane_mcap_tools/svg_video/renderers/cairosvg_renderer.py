"""CairoSVG renderer implementation.

cairosvgライブラリを使用したSVGレンダリングバックエンド。
Python純正の実装で、インストールが容易だが速度は遅い。
"""

import logging

from .base import OutputFormat, SvgRendererBase

logger = logging.getLogger(__name__)


class CairoSvgRenderer(SvgRendererBase):
    """cairosvgを使用したSVGレンダラー."""

    def __init__(
        self,
        width: int = 1920,
        height: int = 1080,
        dpi: int = 96,
        output_format: OutputFormat = OutputFormat.PNG,
    ):
        super().__init__(width, height, dpi, output_format)

        if output_format == OutputFormat.RAW_RGBA:
            logger.warning(
                "CairoSvgRenderer does not support RAW_RGBA output. "
                "Falling back to PNG."
            )
            self.output_format = OutputFormat.PNG

        # cairosvgのインポートと保存（毎フレームのインポート回避）
        try:
            import cairosvg

            self._cairosvg = cairosvg
        except ImportError as e:
            raise ImportError(
                "cairosvg is required. Install with: pip install cairosvg"
            ) from e

    def render(self, svg_string: str) -> bytes:
        try:
            png_bytes = self._cairosvg.svg2png(
                bytestring=svg_string.encode("utf-8"),
                output_width=self.width,
                output_height=self.height,
                dpi=self.dpi,
            )
            return png_bytes
        except Exception as e:
            logger.error(f"Failed to render SVG: {e}")
            raise

    @classmethod
    def is_available(cls) -> bool:
        try:
            import cairosvg  # noqa: F401

            return True
        except ImportError:
            return False

    @classmethod
    def get_name(cls) -> str:
        return "cairosvg"

    @classmethod
    def get_install_command(cls) -> str:
        return "pip install cairosvg"

    @classmethod
    def get_description(cls) -> str:
        return "Pure Python SVG renderer (slow but widely compatible)"
