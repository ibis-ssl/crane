"""SVG to PNG renderer using cairosvg.

このモジュールは、SVG文字列をPNGバイト列にラスタライズします。
"""

import logging

logger = logging.getLogger(__name__)


class SvgRenderer:
    """SVGをPNGにラスタライズ."""

    def __init__(self, width: int = 1920, height: int = 1080, dpi: int = 96):
        """
        初期化.

        Args:
            width: 出力PNG幅（ピクセル）
            height: 出力PNG高さ（ピクセル）
            dpi: 解像度（dots per inch）
        """
        self.width = width
        self.height = height
        self.dpi = dpi

        # cairosvgのインポート確認
        try:
            import cairosvg  # noqa: F401
        except ImportError as e:
            raise ImportError(
                "cairosvg is required. Install with: pip install cairosvg"
            ) from e

    def render(self, svg_string: str) -> bytes:
        """
        SVG文字列をPNGバイト列に変換.

        Args:
            svg_string: SVGドキュメント文字列

        Returns:
            PNGバイト列
        """
        import cairosvg

        try:
            png_bytes = cairosvg.svg2png(
                bytestring=svg_string.encode("utf-8"),
                output_width=self.width,
                output_height=self.height,
                dpi=self.dpi,
            )
            return png_bytes
        except Exception as e:
            logger.error(f"Failed to render SVG: {e}")
            raise
