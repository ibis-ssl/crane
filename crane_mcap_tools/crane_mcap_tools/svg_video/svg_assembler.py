"""SVG assembler for combining layers into complete SVG documents.

このモジュールは、svg_viewer.jsのupdateSvgDisplay()ロジックをPythonに移植し、
複数のレイヤーを単一の完全なSVGドキュメントに合成します。
"""

import logging

logger = logging.getLogger(__name__)


class SvgAssembler:
    """複数のSVGレイヤーを単一のSVGドキュメントに合成."""

    def __init__(
        self,
        viewbox: str = "-6000 -4500 12000 9000",
        background_color: str = "#6c757d",
        grid_interval: int = 1000,
        grid_color: str = "#adb5bd",
        grid_opacity: float = 0.3,
    ):
        """
        初期化.

        Args:
            viewbox: SVGのviewBox属性（SSL field dimensions in mm）
            background_color: 背景色
            grid_interval: グリッド間隔（mm）
            grid_color: グリッド線の色
            grid_opacity: グリッドの不透明度
        """
        self.viewbox = viewbox
        self.background_color = background_color
        self.grid_interval = grid_interval
        self.grid_color = grid_color
        self.grid_opacity = grid_opacity

        # viewBoxを事前パース（キャッシュ）
        self._viewbox_tuple = self._parse_viewbox()

        # 固定部分をキャッシュ（遅延初期化）
        self._cached_header: list[str] | None = None
        self._cached_defs: list[str] | None = None
        self._cached_background: str | None = None
        self._cached_grid: str | None = None

    def assemble(
        self, layers: dict[str, list[str]], visible_layers: set[str] | None = None
    ) -> str:
        """
        レイヤーを合成してSVGドキュメントを生成.

        Args:
            layers: レイヤー名 -> SVGプリミティブのリストのマッピング
            visible_layers: 表示するレイヤーのセット（Noneの場合は全レイヤー表示）

        Returns:
            完全なSVGドキュメント文字列
        """
        if visible_layers is None:
            visible_layers = set(layers.keys())

        # 固定部分のキャッシュ（初回のみ生成）
        if self._cached_header is None:
            self._cached_header = [
                '<?xml version="1.0" encoding="UTF-8"?>',
                '<svg xmlns="http://www.w3.org/2000/svg"',
                f'     viewBox="{self.viewbox}"',
                '     width="100%" height="100%"',
                '     preserveAspectRatio="xMidYMid meet">',
            ]
            self._cached_defs = self._generate_defs()
            self._cached_background = self._generate_background()
            self._cached_grid = self._generate_grid()

        # キャッシュからコピーして使用
        svg_parts = list(self._cached_header)
        svg_parts.extend(self._cached_defs)
        svg_parts.append(self._cached_background)
        svg_parts.append(self._cached_grid)

        # 各レイヤーをグループとして追加
        for layer_name, primitives in layers.items():
            if layer_name in visible_layers and primitives:
                svg_parts.append(f'  <g class="layer-{self._escape_xml(layer_name)}">')
                for primitive in primitives:
                    # プリミティブをインデント
                    indented_primitive = self._indent_primitive(primitive)
                    svg_parts.append(indented_primitive)
                svg_parts.append("  </g>")

        # SVGクロージング
        svg_parts.append("</svg>")

        return "\n".join(svg_parts)

    def _generate_defs(self) -> list[str]:
        """グリッドパターン定義を生成."""
        return [
            "  <defs>",
            f'    <pattern id="grid" width="{self.grid_interval}" height="{self.grid_interval}"',
            '             patternUnits="userSpaceOnUse">',
            f'      <path d="M {self.grid_interval} 0 L 0 0 0 {self.grid_interval}"',
            '            fill="none"',
            f'            stroke="{self.grid_color}"',
            '            stroke-width="20"/>',
            "    </pattern>",
            "  </defs>",
        ]

    def _generate_background(self) -> str:
        """背景レクトを生成."""
        x, y, w, h = self._viewbox_tuple  # キャッシュしたviewBoxを使用
        return (
            f'  <rect x="{x}" y="{y}" width="{w}" height="{h}" '
            f'fill="{self.background_color}"/>'
        )

    def _generate_grid(self) -> str:
        """グリッドレクトを生成."""
        x, y, w, h = self._viewbox_tuple  # キャッシュしたviewBoxを使用
        return (
            f'  <rect x="{x}" y="{y}" width="{w}" height="{h}" '
            f'fill="url(#grid)" opacity="{self.grid_opacity}"/>'
        )

    def _parse_viewbox(self) -> tuple[float, float, float, float]:
        """viewBoxをパース."""
        parts = self.viewbox.split()
        if len(parts) != 4:
            raise ValueError(f"Invalid viewBox: {self.viewbox}")
        return tuple(float(p) for p in parts)

    def _indent_primitive(self, primitive: str, indent: str = "    ") -> str:
        """プリミティブSVG文字列をインデント."""
        lines = primitive.strip().split("\n")
        return "\n".join(indent + line for line in lines)

    def _escape_xml(self, text: str) -> str:
        """XML属性値をエスケープ."""
        return (
            text.replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace('"', "&quot;")
            .replace("'", "&apos;")
        )
