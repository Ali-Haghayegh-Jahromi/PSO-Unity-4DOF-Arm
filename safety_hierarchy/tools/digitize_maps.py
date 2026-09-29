#!/usr/bin/env python3
"""Digitize the static obstacles of Figs. 7-11 of the paper into rectangles.

Needs the paper PDF (not included in the repository) and PyMuPDF, Pillow,
NumPy, SciPy:
    pip install pymupdf pillow numpy scipy
    python3 tools/digitize_maps.py paper.pdf

Method: the five figures are embedded 513x314 px bitmaps (PDF pages 9-10).
The 1 m grid lines give the scale (29 vertical lines x=-14..14, 14.66 px/m,
origin at pixel (256.5, 157)). Black pixels are decomposed greedily into
maximal rectangles, converted to metres and printed. Slivers thinner than
~0.1 m are anti-aliasing artefacts and were dropped by hand; the values in
maps/*.txt are these rectangles rounded to 0.05 m, with walls that touch the
figure frame extended to the world boundary.
"""
import sys

import numpy as np
import pymupdf
from PIL import Image
from scipy import ndimage

PPM = (461.5 - 51.0) / 28.0  # pixels per metre, from the first and last grid line
X0_PX, Y0_PX = 256.5, 157.0  # pixel position of the world origin


def max_rect(mask):
    """Largest all-true rectangle (area, r0, r1, c0, c1), histogram method."""
    rows, cols = mask.shape
    h = np.zeros(cols, int)
    best = (0, 0, 0, 0, 0)
    for r in range(rows):
        h = np.where(mask[r], h + 1, 0)
        stack = []
        for c in range(cols + 1):
            cur = h[c] if c < cols else 0
            start = c
            while stack and stack[-1][1] >= cur:
                s0, hh = stack.pop()
                if hh * (c - s0) > best[0]:
                    best = (hh * (c - s0), r - hh + 1, r + 1, s0, c)
                start = s0
            stack.append((start, cur))
    return best


def figure_images(pdf_path):
    doc = pymupdf.open(pdf_path)
    images = []
    for page in (8, 9):  # 0-based: Figs. 7-9 on page 9, Figs. 10-11 on page 10
        infos = sorted(doc[page].get_image_info(xrefs=True), key=lambda i: i["bbox"][1])
        for info in infos:
            pix = pymupdf.Pixmap(doc, info["xref"])
            images.append(Image.frombytes("RGB", (pix.width, pix.height), pix.samples))
    return images  # map1 .. map5


def main():
    for k, img in enumerate(figure_images(sys.argv[1]), start=1):
        a = np.asarray(img.convert("L")).astype(float)
        inside = np.zeros_like(a, bool)
        inside[52:263, 41:473] = True  # inside the figure frame
        black = ndimage.binary_closing((a < 110) & inside, iterations=1) & inside
        print(f"# map{k}")
        while True:
            area, r0, r1, c0, c1 = max_rect(black)
            if area < 12:
                break
            black[r0:r1, c0:c1] = False
            x0, x1 = (c0 - X0_PX) / PPM, (c1 - X0_PX) / PPM
            y0, y1 = (Y0_PX - r1) / PPM, (Y0_PX - r0) / PPM
            print(f"rect {x0:6.2f} {y0:6.2f} {x1:6.2f} {y1:6.2f}   # {x1 - x0:.2f} x {y1 - y0:.2f} m")


if __name__ == "__main__":
    main()
