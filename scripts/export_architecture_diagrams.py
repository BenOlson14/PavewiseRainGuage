#!/usr/bin/env python3
"""Export architecture Mermaid diagrams to PNG and PDF assets.

This script reads Mermaid blocks from docs/system-architecture.md, renders them
through the public Kroki Mermaid endpoint, and writes PNG/PDF files under
`docs/presentation-assets/`.

Binary assets are intentionally not committed to the repository so pull requests
remain text-only and compatible with tooling that rejects binary diffs.
"""

from __future__ import annotations

import io
import sys
import urllib.request
from pathlib import Path

DOC_PATH = Path(__file__).resolve().parent.parent / "docs" / "system-architecture.md"
OUT_DIR = DOC_PATH.parent / "presentation-assets"
KROKI_URL = "https://kroki.io/mermaid/png"
USER_AGENT = "Mozilla/5.0"
OUTPUT_NAMES = [
    "deployment-networking",
    "firmware-wake-cycle",
    "server-ingest-flow",
]


def require_pillow():
    try:
        from PIL import Image  # type: ignore
    except ImportError as exc:  # pragma: no cover - depends on local env
        raise SystemExit(
            "Missing dependency: pillow. Install it with `python -m pip install pillow`."
        ) from exc
    return Image


def extract_mermaid_blocks(markdown: str) -> list[str]:
    blocks: list[str] = []
    parts = markdown.split("```mermaid\n")
    for part in parts[1:]:
        block, _rest = part.split("\n```", 1)
        blocks.append(block.strip() + "\n")
    return blocks


def render_png(mermaid_source: str) -> bytes:
    request_obj = urllib.request.Request(
        KROKI_URL,
        data=mermaid_source.encode("utf-8"),
        method="POST",
        headers={
            "User-Agent": USER_AGENT,
            "Content-Type": "text/plain;charset=UTF-8",
        },
    )
    with urllib.request.urlopen(request_obj, timeout=60) as response:
        return response.read()


def main() -> int:
    Image = require_pillow()

    markdown = DOC_PATH.read_text()
    blocks = extract_mermaid_blocks(markdown)
    if len(blocks) < len(OUTPUT_NAMES):
        raise SystemExit(
            f"Expected at least {len(OUTPUT_NAMES)} Mermaid blocks, found {len(blocks)}."
        )

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    pdf_images = []

    for output_name, block in zip(OUTPUT_NAMES, blocks):
        png_bytes = render_png(block)
        png_path = OUT_DIR / f"{output_name}.png"
        pdf_path = OUT_DIR / f"{output_name}.pdf"
        png_path.write_bytes(png_bytes)

        image = Image.open(io.BytesIO(png_bytes)).convert("RGB")
        image.save(pdf_path, "PDF", resolution=200.0)
        pdf_images.append(image)
        print(f"wrote {png_path.relative_to(DOC_PATH.parent.parent)}")
        print(f"wrote {pdf_path.relative_to(DOC_PATH.parent.parent)}")

    combined_pdf = OUT_DIR / "pavewise-system-architecture-diagrams.pdf"
    pdf_images[0].save(
        combined_pdf,
        "PDF",
        resolution=200.0,
        save_all=True,
        append_images=pdf_images[1:],
    )
    print(f"wrote {combined_pdf.relative_to(DOC_PATH.parent.parent)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
