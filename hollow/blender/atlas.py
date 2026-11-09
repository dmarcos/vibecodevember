# Pack blender/frames/f00..f10.png into ../horseman.png (4x3 grid, same layout the page expects)
import os
from PIL import Image

here = os.path.dirname(os.path.abspath(__file__))
W, H, COLS, ROWS, NF = 840, 760, 4, 3, 11
atlas = Image.new('RGBA', (W * COLS, H * ROWS), (0, 0, 0, 0))
for i in range(NF):
    im = Image.open(os.path.join(here, 'frames', f'f{i:02d}.png')).convert('RGBA')
    atlas.paste(im, ((i % COLS) * W, (i // COLS) * H))
out = os.path.join(here, '..', 'horseman.png')
atlas.save(out, optimize=True)
print('wrote', os.path.abspath(out), os.path.getsize(out) // 1024, 'KB')
