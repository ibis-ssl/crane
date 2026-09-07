#!/usr/bin/env python3
"""Download web fonts from Google Fonts CDN for local hosting.

This script downloads Material Symbols Outlined, Noto Sans JP, and Roboto
font files and generates CSS files that reference them locally.
Run once before development, or during Docker build.

Usage:
    python3 download_fonts.py [--output-dir DIR]
"""

import argparse
import os
import re
import sys
import urllib.request

UA = (
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
    "AppleWebKit/537.36 (KHTML, like Gecko) "
    "Chrome/120.0.0.0 Safari/537.36"
)

FONTS = {
    "material-symbols-outlined": {
        "url": (
            "https://fonts.googleapis.com/css2"
            "?family=Material+Symbols+Outlined"
            ":opsz,wght,FILL,GRAD@20..48,100..700,0..1,-25..200"
        ),
        "family": "Material Symbols Outlined",
        "subdir": ".",
        "prefix": "material-symbols",
        "extra_css": (
            "\n.material-symbols-outlined {\n"
            "  font-family: 'Material Symbols Outlined';\n"
            "  font-weight: normal;\n"
            "  font-style: normal;\n"
            "  font-size: 24px;\n"
            "  line-height: 1;\n"
            "  letter-spacing: normal;\n"
            "  text-transform: none;\n"
            "  display: inline-block;\n"
            "  white-space: nowrap;\n"
            "  word-wrap: normal;\n"
            "  direction: ltr;\n"
            "  -webkit-font-feature-settings: 'liga';\n"
            "  -webkit-font-smoothing: antialiased;\n"
            "}\n"
        ),
    },
    "noto-sans-jp": {
        "url": "https://fonts.googleapis.com/css2?family=Noto+Sans+JP:wght@400;500;700",
        "family": "Noto Sans JP",
        "subdir": "noto-sans-jp",
        "prefix": "noto-sans-jp",
    },
    "roboto": {
        "url": "https://fonts.googleapis.com/css2?family=Roboto:wght@400;500;700",
        "family": "Roboto",
        "subdir": "roboto",
        "prefix": "roboto",
    },
}


def download_css(url: str) -> str:
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    with urllib.request.urlopen(req, timeout=60) as resp:
        return resp.read().decode("utf-8")


def process_font(name: str, cfg: dict, output_dir: str) -> None:
    subdir = cfg["subdir"]
    prefix = cfg["prefix"]
    font_dir = os.path.join(output_dir, subdir) if subdir != "." else output_dir

    os.makedirs(font_dir, exist_ok=True)

    print(f"=== Downloading {name} ===")
    css_text = download_css(cfg["url"])

    urls = re.findall(r"url\((https://fonts\.gstatic\.com/[^)]+\.woff2)\)", css_text)
    unique_urls = list(dict.fromkeys(urls))

    url_map = {}
    for i, url in enumerate(unique_urls):
        local_name = f"{prefix}-{i}.woff2"
        local_path = os.path.join(font_dir, local_name)
        print(f"  {local_name} ...", end=" ", flush=True)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=60) as resp:
                data = resp.read()
            with open(local_path, "wb") as f:
                f.write(data)
            print(f"{len(data):,} bytes")
            url_map[url] = f"{subdir}/{local_name}" if subdir != "." else local_name
        except Exception as e:
            print(f"FAILED: {e}")
            sys.exit(1)

    for old_url, new_path in url_map.items():
        css_text = css_text.replace(f"url({old_url})", f"url({new_path})")

    if "extra_css" in cfg:
        css_text += cfg["extra_css"]

    css_path = os.path.join(output_dir, f"{name}.css")
    with open(css_path, "w") as f:
        f.write(css_text)
    print(f"  -> {css_path}")


def main():
    parser = argparse.ArgumentParser(description="Download web fonts for local hosting")
    parser.add_argument(
        "--output-dir",
        default=os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "assets", "fonts"
        ),
        help="Output directory for font files (default: assets/fonts/ next to this script)",
    )
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    for name, cfg in FONTS.items():
        process_font(name, cfg, args.output_dir)

    print("\nDone! All fonts downloaded to:", args.output_dir)


if __name__ == "__main__":
    main()
