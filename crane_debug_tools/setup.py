"""Setup script for crane_debug_tools package."""

from setuptools import setup

package_name = "crane_debug_tools"

setup(
    name=package_name,
    version="1.0.121",
    packages=[
        package_name,
        f"{package_name}.mcap_analysis",
        f"{package_name}.svg_video",
    ],
    install_requires=[
        "setuptools",
        "cairosvg>=2.5.0",  # SVG to PNG conversion
    ],
    zip_safe=True,
    maintainer="ibis ssl",
    maintainer_email="ibis.ssl.team@gmail.com",
    description="Modern debugging tools for crane robot skills",
    license="MIT",
)
