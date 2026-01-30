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
        """
        初期化.

        Args:
            width: 出力画像幅（ピクセル）
            height: 出力画像高さ（ピクセル）
            dpi: 解像度（dots per inch）
            output_format: 出力フォーマット（PNG or RAW_RGBA）
        """
        super().__init__(width, height, dpi, output_format)

        # resvg_pyのインポート
        try:
            import resvg_py

            self._svg_to_bytes = resvg_py.svg_to_bytes
        except ImportError as e:
            raise ImportError(
                "resvg-py is required. Install with: pip install resvg-py"
            ) from e

        # RAW出力用にPILを準備
        if output_format == OutputFormat.RAW_RGBA:
            try:
                from PIL import Image

                self._Image = Image
            except ImportError as e:
                raise ImportError(
                    "Pillow is required for RAW_RGBA output. Install with: pip install Pillow"
                ) from e

    def render(self, svg_string: str) -> bytes:
        """
        SVG文字列を画像バイト列に変換.

        Args:
            svg_string: SVGドキュメント文字列

        Returns:
            画像バイト列（PNG or RAW RGBA、output_formatに依存）
        """
        try:
            # resvg_pyでレンダリング（PNG形式で出力）
            png_bytes = self._svg_to_bytes(
                svg_string=svg_string,
                width=self.width,
                height=self.height,
                dpi=self.dpi,
            )

            if self.output_format == OutputFormat.RAW_RGBA:
                # PNGをRAW RGBAに変換
                img = self._Image.open(io.BytesIO(png_bytes))
                # RGBAモードに変換（透明度を保持）
                if img.mode != "RGBA":
                    img = img.convert("RGBA")
                return img.tobytes()
            else:
                # PNG形式のまま返す
                return png_bytes

        except Exception as e:
            logger.error(f"Failed to render SVG with resvg-py: {e}")
            raise

    @classmethod
    def is_available(cls) -> bool:
        """
        resvg-pyが使用可能かチェック.

        Returns:
            使用可能な場合True
        """
        try:
            import resvg_py  # noqa: F401

            return True
        except ImportError:
            return False

    @classmethod
    def get_name(cls) -> str:
        """レンダラー名を取得."""
        return "resvg"

    @classmethod
    def get_description(cls) -> str:
        """レンダラーの説明を取得."""
        return "Rust-based high-performance SVG renderer (3-10x faster than cairosvg)"
