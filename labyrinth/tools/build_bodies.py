# Sculpts the character bodies and clothes for Labyrinth from blended metaball forms,
# adds cloth folds / pleats, decimates, bakes ambient occlusion into vertex colours
# and exports labyrinth/models/bodies.glb.
#
#   /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup --python tools/build_bodies.py -- models/bodies.glb
#
# Coordinates below are in the game's rig space (x right, y up, z forward, metres at rig scale 1).
# Every mesh is modelled in the rig's rest pose (arms hanging straight down) so the game can skin
# it to its own joint hierarchy.
import bpy, bmesh, math, sys
from mathutils import Vector, Quaternion, noise

OUT = sys.argv[sys.argv.index('--') + 1] if '--' in sys.argv else '/tmp/bodies.glb'
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
K = 1 / .575  # metaball radius → surface radius

def T(p):  # rig (x, y-up, z-front) → Blender (x, -z, z-up)
    return Vector((p[0], -p[2], p[1]))

class Part:
    def __init__(self, name, res=.008, target=6000):
        self.name, self.target = name, target
        self.mb = bpy.data.metaballs.new(name); self.mb.resolution = res; self.mb.render_resolution = res; self.mb.threshold = .6
        self.ob = bpy.data.objects.new(name, self.mb); sc.collection.objects.link(self.ob)
        self.folds, self.pleats, self.lumps = [], [], 0
    def ball(self, c, r, s=2.0):
        e = self.mb.elements.new(); e.type = 'BALL'; e.co = T(c); e.radius = r * K; e.stiffness = s; return self
    def ell(self, c, size, s=2.0, rot=None):
        e = self.mb.elements.new(); e.type = 'ELLIPSOID'; e.co = T(c); e.radius = K; e.stiffness = s
        e.size_x, e.size_y, e.size_z = size[0], size[2], size[1]
        if rot: e.rotation = Quaternion(Vector(rot[0]), rot[1])
        return self
    def cap(self, a, b, r, s=2.0):
        A, B = T(a), T(b); d = B - A
        e = self.mb.elements.new(); e.type = 'CAPSULE'; e.co = (A + B) / 2; e.radius = r * K; e.stiffness = s
        e.size_x = d.length / 2; e.size_y = e.size_z = 1
        e.rotation = Vector((1, 0, 0)).rotation_difference(d.normalized()); return self
    def taper(self, pts, radii, steps=6, s=2.0):
        # chain of short capsules with stepped radii → smooth tapered limb
        for i in range(len(pts) - 1):
            for k in range(steps):
                t0, t1 = k / steps, (k + 1) / steps
                a = [pts[i][j] + (pts[i + 1][j] - pts[i][j]) * t0 for j in range(3)]
                b = [pts[i][j] + (pts[i + 1][j] - pts[i][j]) * t1 for j in range(3)]
                r = radii[i] + (radii[i + 1] - radii[i]) * (t0 + t1) / 2
                self.cap(a, b, r * .82, s)   # overlapping neighbours add up, so each is a bit thinner
        return self
    def fold(self, a, b, amp=.004, freq=38, phase=1.6):
        self.folds.append((T(a), T(b), amp, freq, phase)); return self
    def pleat(self, a, b, n=8, amp=.012):
        self.pleats.append((T(a), T(b), n, amp)); return self


parts = []
def P(*a, **k): p = Part(*a, **k); parts.append(p); return p

# ============================================================== contestant (W = 1)
for s, S in ((1, 'L'), (-1, 'R')):
    x = .235 * s
    P(f'P_arm{S}', target=2600).ell((x - .012 * s, 1.465, 0), (.075, .068, .074)) \
        .taper([(x, 1.45, 0), (x, 1.3, 0), (x, 1.18, -.008), (x, 1.04, .004), (x, .95, .004)], [.062, .056, .05, .046, .04]) \
        .ell((x + .016 * s, 1.41, 0), (.06, .07, .062)).ell((x, 1.3, .012), (.05, .08, .05)) \
        .ball((x, 1.18, -.022), .036).ell((x + .004 * s, 1.09, .006), (.046, .07, .046)) \
        .ell((x, .94, .002), (.046, .026, .048)) \
        .fold((x, 1.24, 0), (x, 1.11, 0), .0045, 42).fold((x, .965, 0), (x, .925, 0), .0035, 90)
    lx = .095 * s
    P(f'P_leg{S}', target=2600).taper([(lx, .93, 0), (lx, .72, .012), (lx, .5, .005), (lx, .3, -.012), (lx, .14, 0), (lx, .1, 0)], [.088, .078, .06, .058, .046, .05]) \
        .ell((lx, .74, .022), (.07, .13, .06)).ell((lx - .02 * s, .78, -.005), (.06, .1, .065)) \
        .ell((lx, .35, -.03), (.054, .1, .055)).ball((lx, .5, .026), .05) \
        .ell((lx, .095, 0), (.058, .03, .06)) \
        .fold((lx, .56, 0), (lx, .42, 0), .004, 36).fold((lx, .2, 0), (lx, .09, 0), .005, 48)
P('P_jacket', target=4200).ell((0, 1.11, 0), (.162, .13, .124)).ell((0, 1.29, 0), (.182, .15, .13)).ell((0, 1.42, -.005), (.17, .085, .115)) \
    .ell((.08, 1.34, .055), (.08, .07, .055)).ell((-.08, 1.34, .055), (.08, .07, .055)).ell((0, 1.0, 0), (.158, .06, .122)) \
    .ell((0, 1.32, -.06), (.15, .13, .08)).ball((0, 1.52, -.01), .085) \
    .fold((0, 1.08, 0), (0, .98, 0), .003, 30)
P('P_pants', target=1600).ell((0, .945, 0), (.172, .12, .14)).ell((0, 1.0, 0), (.163, .06, .126)).ell((0, .9, -.05), (.14, .1, .1))

# ============================================================== oni (W = 1.28)
for s, S in ((1, 'L'), (-1, 'R')):
    x = .3 * s
    P(f'O_arm{S}', target=3400).ell((x + .005 * s, 1.465, 0), (.112, .1, .104)) \
        .taper([(x, 1.44, 0), (x, 1.19, -.01), (x, 1.05, .006), (x, .94, .008)], [.085, .066, .06, .046]) \
        .ell((x + .022 * s, 1.42, 0), (.085, .085, .09)) \
        .ell((x, 1.33, .045), (.068, .088, .062)).ell((x, 1.31, -.05), (.062, .095, .058)) \
        .ball((x, 1.18, -.04), .044).ell((x + .014 * s, 1.1, .018), (.078, .085, .074)).ell((x - .01 * s, 1.08, -.01), (.06, .07, .062)) \
        .ell((x, .935, 0), (.05, .028, .058))
    lx = .122 * s
    P(f'O_leg{S}', target=3400).taper([(lx, .93, 0), (lx, .7, .015), (lx, .5, .01), (lx, .3, -.01), (lx, .14, 0), (lx, .09, 0)], [.12, .104, .076, .074, .052, .056]) \
        .ell((lx, .72, .05), (.1, .15, .08)).ell((lx - .03 * s, .76, 0), (.07, .12, .08)).ball((lx, .5, .05), .05) \
        .ell((lx, .36, -.05), (.08, .12, .074)).ell((lx + .03 * s, .33, -.02), (.05, .1, .06)) \
        .ball((lx + .04 * s, .11, 0), .022).ball((lx - .04 * s, .11, 0), .022) \
        .ell((lx, .036, .065), (.074, .04, .122)) \
        .ball((lx - .037, .028, .165), .026).ball((lx, .03, .175), .028).ball((lx + .037, .028, .165), .026)
P('O_torso', target=6500).ell((0, .93, 0), (.2, .12, .17)).ell((0, 1.16, .06), (.215, .2, .19)).ell((0, 1.36, -.01), (.25, .14, .17)) \
    .ell((.1, 1.42, .12), (.115, .08, .066)).ell((-.1, 1.42, .12), (.115, .08, .066)) \
    .ell((0, 1.56, -.05), (.165, .07, .1)).ell((.16, 1.3, -.07), (.05, .12, .06)).ell((-.16, 1.3, -.07), (.05, .12, .06)) \
    .ell((0, 1.37, -.1), (.2, .15, .1)).ell((.17, 1.12, .02), (.06, .1, .09)).ell((-.17, 1.12, .02), (.06, .1, .09)) \
    .cap((0, 1.55, 0), (0, 1.66, 0), .085)

# ============================================================== host (W = 1.12)
for s, S in ((1, 'L'), (-1, 'R')):
    x = .263 * s
    P(f'M_sleeve{S}', target=3000).ell((x, 1.46, 0), (.09, .08, .09)).taper([(x, 1.45, 0), (x, 1.19, 0)], [.088, .1]) \
        .ell((x + .006 * s, 1.02, -.025), (.072, .17, .1)).ell((x + .006 * s, .9, -.03), (.078, .07, .105)) \
        .fold((x, 1.4, 0), (x, 1.2, 0), .005, 26).fold((x, 1.15, 0), (x, .8, 0), .008, 14)
    lx = .106 * s
    P(f'M_hakama{S}', target=3000).taper([(lx, .93, 0), (lx, .6, 0), (lx, .3, 0), (lx, .07, 0)], [.13, .145, .16, .175]) \
        .pleat((lx, .95, 0), (lx, .02, 0), 9, .014)
P('M_robe', target=4500).ell((0, 1.05, 0), (.205, .13, .168)).ell((0, 1.25, 0), (.222, .16, .176)).ell((0, 1.42, -.005), (.24, .1, .17)) \
    .ball((0, 1.54, -.01), .095).fold((0, 1.35, .1), (0, 1.05, .1), .004, 22)
P('M_hips', target=1500).ell((0, .93, 0), (.215, .15, .18)).pleat((0, 1.02, 0), (0, .8, 0), 14, .01)

# ============================================================== general (W = 1.1)
for s, S in ((1, 'L'), (-1, 'R')):
    x = .2585 * s
    P(f'G_arm{S}', target=2600).ell((x - .01 * s, 1.465, 0), (.082, .075, .08)) \
        .taper([(x, 1.45, 0), (x, 1.3, 0), (x, 1.18, -.008), (x, 1.04, .004), (x, .94, .004)], [.064, .058, .052, .048, .043]) \
        .ell((x + .016 * s, 1.41, 0), (.064, .072, .066)).ball((x, 1.18, -.022), .038).ell((x + .004 * s, 1.09, .006), (.048, .07, .048)) \
        .fold((x, 1.24, 0), (x, 1.12, 0), .004, 40)
    lx = .1045 * s
    P(f'G_breeches{S}', target=2400).taper([(lx, .93, 0), (lx, .76, .01), (lx, .6, 0), (lx, .52, 0)], [.1, .135, .11, .078]) \
        .ell((lx + .03 * s, .74, 0), (.11, .12, .1)).fold((lx, .9, 0), (lx, .55, 0), .004, 24)
    P(f'G_boot{S}', target=2400).taper([(lx, .56, 0), (lx, .45, 0), (lx, .2, -.01), (lx, .07, 0)], [.085, .074, .07, .058]) \
        .ell((lx, .04, .045), (.06, .045, .13)).ell((lx, .03, -.07), (.048, .05, .05))
P('G_coat', target=4500).ell((0, 1.06, 0), (.182, .12, .148)).ell((0, 1.28, 0), (.202, .16, .156)).ell((0, 1.43, -.005), (.2, .09, .14)) \
    .ell((.08, 1.35, .06), (.085, .07, .05)).ell((-.08, 1.35, .06), (.085, .07, .05)).ball((0, 1.53, -.01), .09) \
    .ell((0, .9, 0), (.235, .14, .19)).fold((0, .98, .12), (0, .8, .12), .006, 18)
P('G_hips', target=1500).ell((0, .945, 0), (.185, .13, .155))

# ---------------------------------------------------------------- polygonise + detail + decimate
def seg_param(v, a, b):
    d = b - a; t = (v - a).dot(d) / d.length_squared; return t, d

meshes = []
for p in parts:
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(p.ob.evaluated_get(dg)); me.name = p.name
    ob = bpy.data.objects.new(p.name + '_m', me); sc.collection.objects.link(ob)
    bpy.data.objects.remove(p.ob)
    bm = bmesh.new(); bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bm.normal_update()
    for v in bm.verts:
        n, co = v.normal.copy(), v.co.copy(); off = 0.0
        for (a, b, amp, freq, ph) in p.folds:
            t, d = seg_param(co, a, b)
            if 0 < t < 1:
                axis = d.normalized(); radial = (co - a) - axis * (co - a).dot(axis)
                w = math.sin(t * math.pi) ** 1.5
                off += amp * w * math.sin(t * d.length * freq + math.sin(math.atan2(radial.x, radial.y) * 2) * ph)
        for (a, b, k, amp) in p.pleats:
            t, d = seg_param(co, a, b)
            if 0 < t < 1:
                axis = d.normalized(); radial = (co - a) - axis * (co - a).dot(axis)
                ang = math.atan2(radial.x, radial.y)
                off += amp * (abs(math.sin(ang * k / 2)) - .5) * min(1, t * 4)
        off += noise.noise(co * 40) * .0012
        v.co = co + n * off
    bm.to_mesh(me); bm.free()
    for poly in me.polygons: poly.use_smooth = True
    m = ob.modifiers.new('dec', 'DECIMATE'); m.ratio = min(1, p.target / max(1, len(me.polygons)))
    sm = ob.modifiers.new('sm', 'CORRECTIVE_SMOOTH'); sm.factor = .5; sm.iterations = 3; sm.use_only_smooth = True
    dg = bpy.context.evaluated_depsgraph_get()
    me2 = bpy.data.meshes.new_from_object(ob.evaluated_get(dg)); ob.modifiers.clear(); ob.data = me2; me2.name = p.name
    for poly in me2.polygons: poly.use_smooth = True
    ob.name = p.name
    meshes.append(ob)
    print('built', p.name, len(me2.vertices), 'verts', len(me2.polygons), 'faces')

# ---------------------------------------------------------------- ambient occlusion → vertex colours
sc.render.engine = 'CYCLES'
sc.cycles.device = 'CPU'; sc.cycles.samples = 48
sc.world = bpy.data.worlds.new('w'); sc.world.light_settings.distance = .16
offs = {'P': 0, 'O': 4, 'M': 8, 'G': 12}
for ob in meshes: ob.location.x = offs[ob.name[0]]
mat = bpy.data.materials.new('m')
for ob in meshes:
    ob.data.materials.append(mat)
    ca = ob.data.color_attributes.new('Col', 'FLOAT_COLOR', 'POINT'); ob.data.color_attributes.active_color = ca
for ob in meshes:
    bpy.ops.object.select_all(action='DESELECT'); ob.select_set(True); bpy.context.view_layer.objects.active = ob
    sc.render.bake.target = 'VERTEX_COLORS'
    bpy.ops.object.bake(type='AO')
    ca = ob.data.color_attributes['Col']
    for i, v in enumerate(ob.data.vertices):
        ao = ca.data[i].color[0]
        k = (.42 + .58 * ao ** 1.2) * (.86 + .14 * max(-1, min(1, v.normal.z)))
        ca.data[i].color = (k, k, k * .985, 1)
    print('baked', ob.name)
for ob in meshes: ob.location.x = 0; ob.data.materials.clear()

bpy.ops.object.select_all(action='DESELECT')
for ob in meshes: ob.select_set(True)
bpy.ops.export_scene.gltf(filepath=OUT, export_format='GLB', use_selection=True, export_apply=True,
                          export_vertex_color='ACTIVE', export_normals=True, export_texcoords=False, export_materials='NONE')
print('exported', OUT)
