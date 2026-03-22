"""Setup script for crane_ball_calibration_ui package."""

import os
from glob import glob

from setuptools import setup

package_name = "crane_ball_calibration_ui"

setup(
    name=package_name,
    version="1.0.270",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "static"),
            glob("crane_ball_calibration_ui/static/*"),
        ),
    ],
    install_requires=["setuptools", "numpy", "fastapi", "uvicorn[standard]", "pyyaml"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="ibis ssl",
    maintainer_email="ibis.ssl.team@gmail.com",
    description="ボール物理モデルパラメータのインタラクティブ最適化WebUI",
    license="MIT",
    entry_points={
        "console_scripts": [
            "calibration_ui = crane_ball_calibration_ui.app:main",
        ],
    },
)
