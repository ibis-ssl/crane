"""resvg-py renderer implementation.

resvg（Rust製高速SVGレンダラー）のPythonバインディングを使用したバックエンド。
cairosvgの3-10倍高速で、RAW RGBA出力もサポート。
"""

import io
import logging

from .base import OutputFormat, SvgRendererBase

logger = logging.getLogger(__name__)


class ResvgPyRenderer(SvgRendererBase):
    """resvg-pyを使用したSVGレンダラー（最速）."""

    def __init__(
        self,
        width: int = 1920,
        height: int = 1080,
        dpi: int = 96,
        output_format: OutputFormat = OutputFormat.PNG,
    ):
        super().__init__(width, height, dpi, output_format)

        try:
            import resvg_py

            self._svg_to_bytes = resvg_py.svg_to_bytes
        except ImportError as e:
            raise ImportError(
                "resvg-py is required. Install with: pip install resvg-py"
            ) from e

        if output_format == OutputFormat.RAW_RGBA:
            try:
                from PIL import Image

                self._Image = Image
            except ImportError as e:
                raise ImportError(
                    "Pillow is required for RAW_RGBA output. Install with: pip install Pillow"
                ) from e

    def render(self, svg_string: str) -> bytes:
        try:
            png_bytes = self._svg_to_bytes(
                svg_string=svg_string,
                width=self.width,
                height=self.height,
                dpi=self.dpi,
            )

            if self.output_format == OutputFormat.RAW_RGBA:
                img = self._Image.open(io.BytesIO(png_bytes))
                if img.mode != "RGBA":
                    img = img.convert("RGBA")
                return img.tobytes()
            else:
                return png_bytes

        except Exception as e:
            logger.error(f"Failed to render SVG with resvg-py: {e}")
            raise

    @classmethod
    def is_available(cls) -> bool:
        try:
            import resvg_py  # noqa: F401

            return True
        except ImportError:
            return False

    @classmethod
    def get_name(cls) -> str:
        return "resvg"

    @classmethod
    def get_install_command(cls) -> str:
        return "pip install resvg-py"

    @classmethod
    def get_description(cls) -> str:
        return "Rust-based high-performance SVG renderer (3-10x faster than cairosvg)"
