"""SVG renderer with multiple backend support.

このモジュールは、SVG文字列を画像にラスタライズします。
複数のレンダリングバックエンドをサポートし、利用可能な最適なものを自動選択します。
"""

import logging

from .renderers import (
    CairoSvgRenderer,
    OutputFormat,
    ResvgPyRenderer,
    SvgRendererBase,
)

logger = logging.getLogger(__name__)

_RENDERER_CLASSES = [
    ResvgPyRenderer,  # 最速（推奨、pip install resvg-py）
    CairoSvgRenderer,  # フォールバック（pip install cairosvg）
]


def create_renderer(
    backend: str | None = None,
    width: int = 1920,
    height: int = 1080,
    dpi: int = 96,
    output_format: OutputFormat = OutputFormat.PNG,
) -> SvgRendererBase:
    """
    SVGレンダラーを作成.

    Args:
        backend: レンダリングバックエンド名
                 ("auto", "cairosvg", "resvg", "rsvg" のいずれか)
                 Noneまたは"auto"の場合、利用可能な最適なバックエンドを自動選択
        width: 出力画像幅（ピクセル）
        height: 出力画像高さ（ピクセル）
        dpi: 解像度（dots per inch）
        output_format: 出力フォーマット

    Returns:
        SvgRendererBase: レンダラーインスタンス

    Raises:
        ValueError: 指定されたバックエンドが利用不可能な場合
        RuntimeError: 利用可能なバックエンドが存在しない場合
    """
    # バックエンド名 -> クラスのマッピング
    backend_map = {cls.get_name(): cls for cls in _RENDERER_CLASSES}

    if backend is None or backend == "auto":
        # 自動選択: 利用可能な最初のレンダラーを使用
        for renderer_cls in _RENDERER_CLASSES:
            if renderer_cls.is_available():
                logger.info(f"Using {renderer_cls.get_name()} renderer (auto-selected)")
                return renderer_cls(width, height, dpi, output_format)

        # 利用可能なレンダラーが1つもない
        raise RuntimeError(
            "No SVG renderer backend is available. "
            "Please install at least one: pip install cairosvg"
        )

    # 明示的なバックエンド指定
    if backend not in backend_map:
        available = ", ".join(backend_map.keys())
        raise ValueError(f"Unknown backend: {backend}. Available backends: {available}")

    renderer_cls = backend_map[backend]
    if not renderer_cls.is_available():
        raise ValueError(
            f"Backend '{backend}' is not available. "
            f"Please install it: pip install {backend}"
        )

    logger.info(f"Using {renderer_cls.get_name()} renderer")
    return renderer_cls(width, height, dpi, output_format)


def list_available_backends() -> list[tuple[str, bool, str]]:
    """
    利用可能なレンダリングバックエンドを一覧表示.

    Returns:
        list[tuple[str, bool, str]]: (バックエンド名, 利用可能?, 説明) のリスト
    """
    return [
        (cls.get_name(), cls.is_available(), cls.get_description())
        for cls in _RENDERER_CLASSES
    ]
