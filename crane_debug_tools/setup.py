"""Setup script for crane_debug_tools package."""

from setuptools import setup

package_name = "crane_debug_tools"

setup(
    name=package_name,
    version="1.0.121",
    packages=[
        package_name,
        f"{package_name}.mcap_analysis",
        f"{package_name}.bag_analysis",
        f"{package_name}.svg_video",
        f"{package_name}.svg_video.renderers",
    ],
    install_requires=[
        "setuptools",
        "cairosvg>=2.5.0",  # SVG to PNG conversion (fallback renderer)
    ],
    extras_require={
        "fast": [
            "resvg-py>=0.4.0",  # Rust-based high-performance renderer (推奨)
            "Pillow>=10.0.0",  # resvg-pyのPNG出力に必要
        ],
    },
    zip_safe=True,
    maintainer="ibis ssl",
    maintainer_email="ibis.ssl.team@gmail.com",
    description="Modern debugging tools for crane robot skills",
    license="MIT",
)
