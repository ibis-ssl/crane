"""Base class for SVG renderers.

抽象基底クラスを定義し、すべてのレンダリングバックエンドの共通インターフェースを提供します。
"""

from abc import ABC, abstractmethod
from enum import Enum


class OutputFormat(Enum):
    """レンダリング出力フォーマット."""

    PNG = "png"
    RAW_RGBA = "raw_rgba"


class SvgRendererBase(ABC):
    """SVGレンダラーの抽象基底クラス.

    すべてのレンダリングバックエンドはこのクラスを継承し、
    render()メソッドとis_available()メソッドを実装する必要があります。
    """

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
        self.width = width
        self.height = height
        self.dpi = dpi
        self.output_format = output_format

    @abstractmethod
    def render(self, svg_string: str) -> bytes:
        """
        SVG文字列を画像バイト列に変換.

        Args:
            svg_string: SVGドキュメント文字列

        Returns:
            画像バイト列（フォーマットはoutput_formatに依存）
        """
        pass

    @classmethod
    @abstractmethod
    def is_available(cls) -> bool:
        """
        このレンダラーが使用可能かチェック.

        Returns:
            使用可能な場合True
        """
        pass

    @classmethod
    @abstractmethod
    def get_name(cls) -> str:
        """
        レンダラーの名前を取得.

        Returns:
            レンダラー名（例: "cairosvg", "resvg"）
        """
        pass

    @classmethod
    def get_description(cls) -> str:
        """
        レンダラーの説明を取得.

        Returns:
            レンダラーの簡単な説明
        """
        return ""
