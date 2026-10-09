#!/usr/bin/env python3
"""Auto-crop white margins from the exported report figures.

The MATLAB -> PDF (vector, painters) -> pdftoppm pipeline produces correct
continuous lines but keeps the PDF page margins; this trims them to the
content plus a small uniform border. Safe to re-run (idempotent).
"""
import sys
from pathlib import Path

from PIL import Image, ImageChops


def trim(path: Path, border: int = 14, thresh: int = 250) -> None:
    img = Image.open(path).convert('RGB')
    # background = the corner pixel colour (white in our exports)
    bg = Image.new('RGB', img.size, img.getpixel((2, 2)))
    diff = ImageChops.difference(img, bg).convert('L')
    bbox = diff.point(lambda p: 255 if p > (255 - thresh) else 0).getbbox()
    if not bbox:
        return                      # fully blank; leave as is
    l, t, r, b = bbox
    l = max(0, l - border)
    t = max(0, t - border)
    r = min(img.width, r + border)
    b = min(img.height, b + border)
    img.crop((l, t, r, b)).save(path)


if __name__ == '__main__':
    targets = sys.argv[1:] or ['.']
    n = 0
    for t in targets:
        p = Path(t)
        files = [p] if p.is_file() else sorted(p.glob('fig_*.png'))
        for f in files:
            trim(f)
            n += 1
            print(f'cropped {f}')
    print(f'{n} figure(s) processed')
