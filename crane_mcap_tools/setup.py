"""Setup script for crane_mcap_tools package."""

import os

from setuptools import setup

package_name = "crane_mcap_tools"

setup(
    name=package_name,
    version="1.0.355",
    packages=[
        package_name,
        f"{package_name}.mcap_analysis",
        f"{package_name}.bag_analysis",
        f"{package_name}.svg_video",
        f"{package_name}.svg_video.renderers",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("lib", package_name),
            ["scripts/mcap_analyzer.py", "scripts/svg_video_generator.py"],
        ),
    ],
    install_requires=[
        "setuptools",
        "cairosvg>=2.5.0",  # SVG to PNG conversion (fallback renderer)
    ],
    tests_require=["pytest"],
    extras_require={
        "fast": [
            "resvg-py>=0.4.0",  # Rust-based high-performance renderer (推奨)
            "Pillow>=10.0.0",  # resvg-pyのPNG出力に必要
        ],
    },
    zip_safe=True,
    maintainer="ibis ssl",
    maintainer_email="ibis.ssl.team@gmail.com",
    description="Python版Rosbag解析・MCAP解析・SVG動画生成ツール",
    license="MIT",
    entry_points={
        "console_scripts": [
            "crane_bag_py = crane_mcap_tools.bag_analysis.cli:main",
        ],
    },
)
