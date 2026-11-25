#!/usr/bin/env python3
"""Convert a PlayCanvas SOG v3 scene (meta.json + webp textures) to SOG v2 so Spark 2.x can load it.

v3 differs from v2 in two places only (see playcanvas/splat-transform src/lib/readers/read-sog.ts):
  * quats.webp: v3 stores a 24-bit exponential-map rotation in RGB; v2 stores "smallest three"
    components in RGB with the index of the dropped (largest) component as 252..255 in A.
  * sh0.webp alpha: v3 stores opacity as b = round(204 * sqrt(A)) (A may exceed 1); v2 stores A * 255.
Means, scales, sh0 colour and shN are encoded identically and are copied unchanged.

usage: python3 sog_v3_to_v2.py <src_dir> <dst_dir>
"""
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image


def load_rgba(path):
    return np.array(Image.open(path).convert('RGBA'))


def save_rgba(arr, path):
    # lossless + exact: keep RGB under alpha 0 untouched (it carries data, not colour)
    Image.fromarray(arr, 'RGBA').save(path, 'WEBP', lossless=True, exact=True, quality=100, method=4)


def convert_quats(src, dst):
    q = load_rgba(src).astype(np.float64)
    px, py, pz = ((q[..., c] - 128.0) / 127.0 for c in range(3))
    r = np.sqrt(px * px + py * py + pz * pz)
    t = (math.pi / 2) * r
    k = np.where(r > 1e-12, np.sin(t) / np.maximum(r, 1e-12), math.pi / 2)
    comps = np.stack([np.cos(t), px * k, py * k, pz * k], axis=-1)        # [w, x, y, z]
    comps /= np.maximum(np.linalg.norm(comps, axis=-1, keepdims=True), 1e-12)
    largest = np.argmax(np.abs(comps), axis=-1)
    sign = np.where(np.take_along_axis(comps, largest[..., None], -1) < 0, -1.0, 1.0)
    comps *= sign                                                         # largest component positive
    # the three kept components, in [w,x,y,z] order with the largest removed (v2 QUAT_IDX)
    keep = np.array([[1, 2, 3], [0, 2, 3], [0, 1, 3], [0, 1, 2]])[largest]
    kept = np.take_along_axis(comps, keep, -1)
    out = np.empty(q.shape, np.uint8)
    out[..., :3] = np.clip(np.round((kept / math.sqrt(2) + 0.5) * 255.0), 0, 255).astype(np.uint8)
    out[..., 3] = (252 + largest).astype(np.uint8)
    save_rgba(out, dst)
    # report the round-trip error of the re-encode
    back = (out[..., :3].astype(np.float64) / 255.0 - 0.5) * math.sqrt(2)
    rest = np.sqrt(np.maximum(0, 1 - (back ** 2).sum(-1)))
    rec = np.zeros_like(comps)
    np.put_along_axis(rec, keep, back, -1)
    np.put_along_axis(rec, largest[..., None], rest[..., None], -1)
    dot = np.abs((rec * comps).sum(-1)).clip(0, 1)
    err = np.degrees(2 * np.arccos(dot))
    print(f'quats: max error {err.max():.2f} deg, mean {err.mean():.3f} deg')


def convert_sh0(src, dst):
    c = load_rgba(src)
    a = c[..., 3].astype(np.float64)
    opacity = np.minimum((a / 204.0) ** 2, 1.0)
    c = c.copy()
    c[..., 3] = np.clip(np.round(opacity * 255.0), 0, 255).astype(np.uint8)
    save_rgba(c, dst)
    print(f'sh0: {(a > 204).sum()} texels had opacity > 1 (clamped)')


def main(src_dir, dst_dir):
    src, dst = Path(src_dir), Path(dst_dir)
    dst.mkdir(parents=True, exist_ok=True)
    meta = json.loads((src / 'meta.json').read_text())
    assert meta.get('version') == 3, 'expected a SOG v3 meta.json'
    if meta.get('blind'):
        print('note: dropping v3 blind data (not used by Spark)')
    convert_quats(src / meta['quats']['files'][0], dst / meta['quats']['files'][0])
    convert_sh0(src / meta['sh0']['files'][0], dst / meta['sh0']['files'][0])
    for key in ('means', 'scales', 'shN'):
        for f in meta.get(key, {}).get('files', []):
            shutil.copyfile(src / f, dst / f)
    out = {k: v for k, v in meta.items() if k not in ('opacity', 'blind', 'model')}
    out['version'] = 2
    (dst / 'meta.json').write_text(json.dumps(out))
    print('wrote', dst / 'meta.json')


if __name__ == '__main__':
    main(*sys.argv[1:3])
