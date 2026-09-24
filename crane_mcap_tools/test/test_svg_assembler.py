# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""SvgAssembler.assemble の出力を固定する特性テスト."""

import pytest
from crane_mcap_tools.svg_video.svg_assembler import SvgAssembler

DEFAULT_PREFIX = [
    '<?xml version="1.0" encoding="UTF-8"?>',
    '<svg xmlns="http://www.w3.org/2000/svg"',
    '     viewBox="-6000 -4500 12000 9000"',
    '     width="100%" height="100%"',
    '     preserveAspectRatio="xMidYMid meet">',
    "  <defs>",
    '    <pattern id="grid" width="1000" height="1000"',
    '             patternUnits="userSpaceOnUse">',
    '      <path d="M 1000 0 L 0 0 0 1000"',
    '            fill="none"',
    '            stroke="#adb5bd"',
    '            stroke-width="20"/>',
    "    </pattern>",
    "  </defs>",
    '  <rect x="-6000.0" y="-4500.0" width="12000.0" height="9000.0" fill="#6c757d"/>',
    (
        '  <rect x="-6000.0" y="-4500.0" width="12000.0" height="9000.0" '
        'fill="url(#grid)" opacity="0.3"/>'
    ),
]


def test_empty_layers_produce_only_fixed_parts():
    assert SvgAssembler().assemble({}) == "\n".join([*DEFAULT_PREFIX, "</svg>"])


def test_constructor_arguments_reach_fixed_parts():
    svg = SvgAssembler(
        viewbox="0 0 100 50",
        background_color="#000",
        grid_interval=10,
        grid_color="red",
        grid_opacity=0.5,
    ).assemble({})

    assert svg.splitlines()[2:] == [
        '     viewBox="0 0 100 50"',
        '     width="100%" height="100%"',
        '     preserveAspectRatio="xMidYMid meet">',
        "  <defs>",
        '    <pattern id="grid" width="10" height="10"',
        '             patternUnits="userSpaceOnUse">',
        '      <path d="M 10 0 L 0 0 0 10"',
        '            fill="none"',
        '            stroke="red"',
        '            stroke-width="20"/>',
        "    </pattern>",
        "  </defs>",
        '  <rect x="0.0" y="0.0" width="100.0" height="50.0" fill="#000"/>',
        (
            '  <rect x="0.0" y="0.0" width="100.0" height="50.0" '
            'fill="url(#grid)" opacity="0.5"/>'
        ),
        "</svg>",
    ]


def test_layers_are_wrapped_indented_and_escaped_in_insertion_order():
    layers = {
        "robots": ['<circle r="1"/>', "  <g>\n<rect/>\n</g>  "],
        "a&b<\"'>": ["<line/>"],
    }

    assert SvgAssembler().assemble(layers) == "\n".join(
        [
            *DEFAULT_PREFIX,
            '  <g class="layer-robots">',
            '    <circle r="1"/>',
            "    <g>",
            "    <rect/>",
            "    </g>",
            "  </g>",
            '  <g class="layer-a&amp;b&lt;&quot;&apos;&gt;">',
            "    <line/>",
            "  </g>",
            "</svg>",
        ]
    )


def test_invisible_and_empty_layers_are_skipped():
    layers = {"ball": ["<circle/>"], "hidden": ["<rect/>"], "empty": []}

    svg = SvgAssembler().assemble(layers, visible_layers={"ball", "empty"})

    assert svg.splitlines()[len(DEFAULT_PREFIX) :] == [
        '  <g class="layer-ball">',
        "    <circle/>",
        "  </g>",
        "</svg>",
    ]


def test_repeated_assemble_gives_same_fixed_parts():
    assembler = SvgAssembler()
    first = assembler.assemble({"a": ["<rect/>"]})
    assembler.assemble({"b": ["<line/>"]})

    assert assembler.assemble({"a": ["<rect/>"]}) == first


def test_invalid_viewbox_is_rejected_at_construction():
    with pytest.raises(ValueError, match="Invalid viewBox"):
        SvgAssembler(viewbox="0 0 100")
