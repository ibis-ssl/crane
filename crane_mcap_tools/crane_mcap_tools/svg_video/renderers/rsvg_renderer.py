"""rsvg-convert renderer implementation.

librsvg2-binのrsvg-convertコマンドを使用したバックエンド。
apt install librsvg2-binでインストール可能。Docker環境に適している。
"""

import logging
import shutil
import subprocess

from .base import OutputFormat, SvgRendererBase

logger = logging.getLogger(__name__)


class RsvgCliRenderer(SvgRendererBase):
    """rsvg-convertコマンドを使用したSVGレンダラー."""

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
            output_format: 出力フォーマット（PNG only）
        """
        super().__init__(width, height, dpi, output_format)

        if output_format == OutputFormat.RAW_RGBA:
            logger.warning(
                "RsvgCliRenderer does not support RAW_RGBA output. Falling back to PNG."
            )
            self.output_format = OutputFormat.PNG

        # rsvg-convertの存在確認
        if not self.is_available():
            raise RuntimeError(
                "rsvg-convert is not available. Install with: apt install librsvg2-bin"
            )

    def render(self, svg_string: str) -> bytes:
        """
        SVG文字列をPNGバイト列に変換.

        Args:
            svg_string: SVGドキュメント文字列

        Returns:
            PNGバイト列
        """
        try:
            # rsvg-convertコマンドを実行
            # 標準入力からSVGを受け取り、標準出力にPNGを出力
            result = subprocess.run(
                [
                    "rsvg-convert",
                    "--width",
                    str(self.width),
                    "--height",
                    str(self.height),
                    "--dpi-x",
                    str(self.dpi),
                    "--dpi-y",
                    str(self.dpi),
                    "--format",
                    "png",
                ],
                input=svg_string.encode("utf-8"),
                capture_output=True,
                check=True,
            )
            return result.stdout

        except subprocess.CalledProcessError as e:
            logger.error(f"rsvg-convert failed: {e.stderr.decode('utf-8')}")
            raise RuntimeError(f"Failed to render SVG: {e}") from e
        except Exception as e:
            logger.error(f"Failed to render SVG with rsvg-convert: {e}")
            raise

    @classmethod
    def is_available(cls) -> bool:
        """
        rsvg-convertが使用可能かチェック.

        Returns:
            使用可能な場合True
        """
        return shutil.which("rsvg-convert") is not None

    @classmethod
    def get_name(cls) -> str:
        """レンダラー名を取得."""
        return "rsvg"

    @classmethod
    def get_description(cls) -> str:
        """レンダラーの説明を取得."""
        return "CLI-based SVG renderer using librsvg (apt install librsvg2-bin)"
