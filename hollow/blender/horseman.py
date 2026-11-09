# Headless horseman: procedural model + Muybridge gallop, rendered to a sprite atlas.
# Run:  /Applications/Blender.app/Contents/MacOS/Blender -b -P blender/horseman.py -- [out_dir] [frame]
import bpy, bmesh, math, sys, os
from mathutils import Vector

TAU = math.tau
NF, DUTY = 11, 0.27
FR_W, FR_H, FR_OX, FR_OY, FR_S = 840, 760, 440, 730, 190

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
OUT = os.path.abspath(argv[0]) if argv else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'frames')
ONLY = int(argv[1]) if len(argv) > 1 else None
os.makedirs(OUT, exist_ok=True)

def hash1(n):
    s = math.sin(n * 127.1 + 311.7) * 43758.5453
    return s - math.floor(s)
def lerp(a, b, t): return a + (b - a) * t
def v2(x, y): return (x, y)
def add(a, b, k=1): return (a[0] + b[0] * k, a[1] + b[1] * k)
def sub(a, b): return (a[0] - b[0], a[1] - b[1])
def l2(a, b, t): return (lerp(a[0], b[0], t), lerp(a[1], b[1], t))
def nrm(a):
    l = math.hypot(*a) or 1
    return (a[0] / l, a[1] / l)
def perp(a): return (-a[1], a[0])
def dirv(t): return (math.cos(t), math.sin(t))
def V(p, y=0.0): return Vector((p[0], y, p[1]))
def cr(p0, p1, p2, p3, t):
    t2, t3 = t * t, t * t * t
    return 0.5 * (2 * p1 + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)
def densify(rows, n=8):
    """Catmull-Rom through table rows (all columns), so profiles are curved, not faceted"""
    out = []
    for i in range(len(rows) - 1):
        r0, r1, r2, r3 = rows[max(0, i - 1)], rows[i], rows[i + 1], rows[min(len(rows) - 1, i + 2)]
        for k in range(n):
            t = k / n
            out.append(tuple(cr(a, b, c, d, t) for a, b, c, d in zip(r0, r1, r2, r3)))
    out.append(tuple(rows[-1]))
    return out
def interp(table, x):
    """piecewise-linear lookup in [(x, values...)] sorted by x"""
    if x <= table[0][0]: return table[0][1:]
    for a, b in zip(table, table[1:]):
        if x <= b[0]:
            t = (x - a[0]) / (b[0] - a[0])
            return tuple(lerp(u, w, t) for u, w in zip(a[1:], b[1:]))
    return table[-1][1:]

# ------------------------------------------------------------------ mesh builder
class MB:
    def __init__(self): self.v, self.f = [], []
    def loft(self, centers, ry, rz, sides=18, caps=True, lat=Vector((0, 1, 0))):
        n = len(centers); rings = []
        for i, c in enumerate(centers):
            t = (centers[min(n - 1, i + 1)] - centers[max(0, i - 1)])
            t = t.normalized() if t.length > 1e-9 else Vector((1, 0, 0))
            l = lat - t * lat.dot(t)
            l = l.normalized() if l.length > 1e-6 else Vector((0, 1, 0))
            u = t.cross(l)
            ring = []
            for k in range(sides):
                a = TAU * k / sides
                ring.append(len(self.v)); self.v.append(c + l * (ry[i] * math.cos(a)) + u * (rz[i] * math.sin(a)))
            rings.append((ring, t))
        for i in range(n - 1):
            r0, r1 = rings[i][0], rings[i + 1][0]
            for k in range(sides):
                self.f.append((r0[k], r0[(k + 1) % sides], r1[(k + 1) % sides], r1[k]))
        if caps:
            for end, sign in ((0, -1), (n - 1, 1)):
                ring, t = rings[end]
                ci = len(self.v); self.v.append(centers[end] + t * sign * min(ry[end], rz[end]) * 0.6)
                for k in range(sides):
                    a, b = ring[k], ring[(k + 1) % sides]
                    self.f.append((b, a, ci) if sign < 0 else (a, b, ci))
    def grid(self, fn, nu, nv):
        base = len(self.v)
        for j in range(nv + 1):
            for i in range(nu + 1):
                self.v.append(fn(i / nu, j / nv))
        for j in range(nv):
            for i in range(nu):
                a = base + j * (nu + 1) + i
                self.f.append((a, a + 1, a + nu + 2, a + nu + 1))

def chain(points, prof):
    """resample a polyline of 2D joints with per-sample (seg_index, t, front, back) into centers/radii"""
    cs, rs = [], []
    for seg, t, fr, bk in prof:
        A, B = points[seg], points[min(seg + 1, len(points) - 1)]
        n2 = perp(nrm(sub(B, A))) if A != B else (1, 0)
        c = add(l2(A, B, t), n2, (fr - bk) / 2)
        cs.append(c); rs.append((fr + bk) / 2)
    return cs, rs

# ------------------------------------------------------------------ pose (port of the page's gallop rig)
LEGS = [dict(fore=False, start=0.00, y=-0.14), dict(fore=False, start=0.08, y=0.14),
        dict(fore=True, start=0.30, y=-0.13), dict(fore=True, start=0.40, y=0.13)]

def build(p):
    bodyY = 1.17 + 0.055 * math.cos(TAU * (p - 0.85))
    pitch = 0.075 * math.sin(TAU * (p - 0.05))
    cp, sp = math.cos(pitch), math.sin(pitch)
    def B(x, y): return (x * cp - y * sp, bodyY + x * sp + y * cp)

    horse, hair, rider, cloth, tack, metal = MB(), MB(), MB(), MB(), MB(), MB()

    # ---- body barrel: x, top, bottom, half-width
    body = [(-0.95, 0.09, 0.03, 0.05), (-0.9, 0.17, -0.01, 0.12), (-0.8, 0.31, -0.08, 0.2), (-0.65, 0.39, -0.16, 0.24),
            (-0.45, 0.39, -0.24, 0.26), (-0.2, 0.33, -0.31, 0.27), (0.05, 0.32, -0.345, 0.28), (0.3, 0.38, -0.33, 0.26),
            (0.5, 0.43, -0.3, 0.23), (0.7, 0.33, -0.22, 0.2), (0.82, 0.22, -0.15, 0.17), (0.9, 0.06, -0.06, 0.09)]
    body = densify(body, 6)
    xs = [lerp(-0.95, 0.9, i / 44) for i in range(45)]
    cs, ry, rz = [], [], []
    for x in xs:
        top, bot, w = interp(body, x)
        cs.append(V(B(x, (top + bot) / 2))); rz.append((top - bot) / 2); ry.append(w)
    horse.loft(cs, ry, rz, sides=24)
    # tail dock
    horse.loft([V(B(-0.78, 0.28)), V(B(-0.9, 0.35)), V(B(-0.98, 0.37)), V(B(-1.06, 0.37))], [0.07, 0.06, 0.05, 0.04], [0.07, 0.06, 0.05, 0.04], sides=12)

    # ---- neck & head
    nu = 0.6 + 0.09 * math.sin(TAU * (p - 0.3)) + pitch
    nd, nb = dirv(nu), B(0.6, 0.12)
    nn = perp(nd)
    def N(u, v): return add(add(nb, nd, u), nn, v)
    neck = [(-0.1, 0.32, -0.3, 0.2), (0.1, 0.25, -0.25, 0.17), (0.32, 0.21, -0.18, 0.13), (0.52, 0.16, -0.13, 0.1), (0.7, 0.11, -0.09, 0.085)]
    neck = densify(neck, 6)
    cs, ry, rz = [], [], []
    for i in range(22):
        u = lerp(-0.1, 0.7, i / 21); top, bot, w = interp(neck, u)
        cs.append(V(N(u, (top + bot) / 2))); rz.append((top - bot) / 2); ry.append(w)
    horse.loft(cs, ry, rz, sides=20)
    hA = nu - 1.5 + 0.07 * math.sin(TAU * (p - 0.1))
    hd, poll = dirv(hA), N(0.66, 0.03)
    hn = perp(hd)
    def Hd(a, b): return add(add(poll, hd, a), hn, b)
    head = [(-0.04, 0.09, -0.08, 0.085), (0.08, 0.115, -0.15, 0.105), (0.18, 0.125, -0.15, 0.105), (0.28, 0.105, -0.11, 0.085),
            (0.4, 0.09, -0.08, 0.07), (0.52, 0.085, -0.076, 0.066), (0.6, 0.066, -0.075, 0.062), (0.645, 0.025, -0.045, 0.035)]
    head = densify(head, 6)
    cs, ry, rz = [], [], []
    for i in range(26):
        a = lerp(-0.04, 0.645, i / 25); top, bot, w = interp(head, a)
        cs.append(V(Hd(a, (top + bot) / 2))); rz.append((top - bot) / 2); ry.append(w)
    horse.loft(cs, ry, rz, sides=20)
    for side in (-1, 1):  # ears, pinned back
        eb = Hd(0.03, 0.085); ed = dirv(hA + 2.25)
        pts = [V(add(eb, ed, t * 0.19), side * (0.055 + t * 0.02)) for t in (0, 0.35, 0.7, 1.0)]
        horse.loft(pts, [0.03, 0.034, 0.022, 0.004], [0.038, 0.04, 0.026, 0.004], sides=10)

    # ---- legs
    PL = 0.16
    def ik(R, F, a, b, front):
        dx, dy = F[0] - R[0], F[1] - R[1]; d = math.hypot(dx, dy)
        dmax, dmin = (a + b) * 0.995, abs(a - b) + 0.03
        if d > dmax: F = (R[0] + dx / d * dmax, R[1] + dy / d * dmax); d = dmax
        if d < dmin: F = (R[0] + dx / d * dmin, R[1] + dy / d * dmin); d = dmin
        th = math.atan2(F[1] - R[1], F[0] - R[0])
        al = math.acos(max(-1, min(1, (a * a + d * d - b * b) / (2 * a * d))))
        ang = th + al if front else th - al
        return (R[0] + math.cos(ang) * a, R[1] + math.sin(ang) * a), F
    for li, leg in enumerate(LEGS):
        q = ((p - leg['start']) % 1 + 1) % 1; fore = leg['fore']
        refX = 0.5 if fore else -0.45; Af = 0.42 if fore else 0.36; Ab = 0.4 if fore else 0.52
        if q < DUTY:
            s = q / DUTY; hx = refX + Af - (Af + Ab) * s; hy = 0.035
            pa = 1.75 + 0.4 * math.sin(math.pi * s) + max(0, s - 0.7) * 1.4
        else:
            s = (q - DUTY) / (1 - DUTY); e = s * s * (3 - 2 * s)
            hx = refX - Ab + (Ab + Af) * e - (0.26 if fore else 0.1) * math.sin(math.pi * s) * (1 - s)
            hy = 0.035 + (0.6 if fore else 0.42) * math.sin(math.pi * s ** 0.8)
            fold = math.sin(math.pi * min(1, s / 0.85)) ** 0.8
            pa = lerp(lerp(2.25, 1.8, s), 0.05 if fore else 0.65, fold)
        u = max(-1, min(1, (hx - refX) / 0.45))
        if fore:
            top = B(0.74, 0.04); R = add(B(0.78, 0.02), dirv(-2.25 + 0.25 * u + pitch), 0.3)
        else:
            top = B(-0.58, 0.12); R = add(top, dirv(-1.0 + 0.3 * u + pitch), 0.42)
        J, F = ik(R, add((hx, hy), dirv(pa), PL), 0.5, 0.4 if fore else 0.46, fore)
        H = add(F, dirv(pa + math.pi), PL)
        hdir = nrm(sub(H, F)); toe = add(add(H, perp(hdir), 0.02), hdir, 0.035)
        pts = [top, R, J, F, H, toe]
        if fore:
            prof = [(0, 0, 0.13, 0.13), (0, 0.6, 0.11, 0.1), (1, 0, 0.105, 0.095), (1, 0.25, 0.095, 0.08), (1, 0.5, 0.074, 0.062), (1, 0.8, 0.052, 0.046), (1, 1, 0.06, 0.06),
                    (2, 0.15, 0.044, 0.056), (2, 0.5, 0.037, 0.045), (2, 0.85, 0.038, 0.05), (2, 1, 0.046, 0.072),
                    (3, 0.4, 0.034, 0.036), (3, 0.85, 0.042, 0.048), (4, 0.3, 0.07, 0.045), (4, 0.8, 0.072, 0.04)]
        else:
            prof = [(0, 0, 0.2, 0.2), (0, 0.5, 0.17, 0.18), (1, 0, 0.1, 0.14), (1, 0.2, 0.088, 0.125), (1, 0.5, 0.064, 0.08), (1, 0.8, 0.05, 0.05), (1, 0.96, 0.052, 0.085),
                    (2, 0.03, 0.056, 0.06), (2, 0.3, 0.043, 0.05), (2, 0.75, 0.04, 0.052), (2, 1, 0.047, 0.074),
                    (3, 0.4, 0.034, 0.036), (3, 0.85, 0.042, 0.048), (4, 0.3, 0.07, 0.045), (4, 0.8, 0.072, 0.04)]
        c2, r2 = chain(pts, prof)
        dense = densify([(c[0], c[1], r) for c, r in zip(c2, r2)], 5)
        horse.loft([V((d[0], d[1]), leg['y']) for d in dense], [d[2] * 0.82 for d in dense], [d[2] for d in dense], sides=16)
        # feathering behind the fetlock
        hn2 = perp(hdir)
        for k in range(7):
            root = add(l2(F, H, 0.05 + k * 0.1), hn2, -0.03)
            base = nrm(add((-hn2[0], -hn2[1]), hdir, 0.6)); cur = root; sp3 = []
            for j in range(6):
                t = j / 5; sp3.append(V(cur, leg['y'] + (hash1(k + li * 7) - 0.5) * 0.05))
                d = nrm(add(add(base, (-1, 0), 0.8 * t), (0, -1), 0.4 * t))
                cur = add(cur, add(d, perp(d), 0.35 * math.sin(TAU * p * 2 + k + t * 3)), 0.03 + 0.012 * hash1(k + 3.3))
            hair.loft(sp3, [lerp(0.012, 0.002, j / 5) for j in range(6)], [lerp(0.012, 0.002, j / 5) for j in range(6)], sides=5)

    # ---- mane & forelock & tail
    def strand(root, y, n, step, ang_fn, w0, w1):
        cur = root; pts = []
        for j in range(n + 1):
            t = j / n; pts.append(V(cur, y)); cur = add(cur, dirv(ang_fn(t)), step)
        r = [lerp(w0, w1, j / n) for j in range(n + 1)]
        hair.loft(pts, [x * 0.8 for x in r], r, sides=6)
    def crest(u):
        top, bot, w = interp(neck, u); return N(u, top - 0.02)
    for i in range(46):
        c = i / 45; u = lerp(0.0, 0.66, c)
        ln = (0.14 + 0.3 * math.sin(math.pi * (0.15 + c * 0.85))) * (0.7 + 0.6 * hash1(i * 5.1))
        strand(crest(u), (hash1(i * 3.3) - 0.5) * 0.12, 7, ln / 7,
               lambda t, i=i: math.pi - 0.5 - pitch + 0.45 * t + 0.35 * t * math.sin(TAU * 2 * p - t * 4 + i * 0.9), 0.022, 0.003)
    for i in range(6):
        strand(Hd(0.04 + i * 0.015, 0.1), (i - 2.5) * 0.015, 5, 0.03,
               lambda t, i=i: math.pi - 0.3 + 0.4 * t + 0.3 * math.sin(TAU * 2 * p + i + t * 3), 0.016, 0.003)
    dock0, dock1 = B(-0.93, 0.36), B(-1.06, 0.37)
    for lock in range(7):
        lockA = (lock - 3) * 0.06; ph = lock * 1.3; lockLen = 1.0 + 0.3 * hash1(lock * 2.7)
        for k in range(9):
            idd = lock * 9 + k; rt = l2(dock0, dock1, 0.3 + 0.7 * hash1(idd * 1.7))
            ln = lockLen * (0.6 + 0.45 * hash1(idd * 3.1)); jit = (hash1(idd * 9.2) - 0.5) * 0.08
            strand(rt, (lock - 3) * 0.018 + (hash1(idd) - 0.5) * 0.04, 12, ln / 12,
                   lambda t, a=lockA, j=jit, ph=ph, k=k: math.pi - 0.22 + a + j + 0.55 * t * t - pitch * (1 - t)
                   + 0.32 * t * math.sin(TAU * p - t * 4.5 + ph) + 0.1 * t * math.sin(TAU * 2 * p - t * 9 + ph + k * 0.4),
                   0.024 * (0.6 + 0.6 * hash1(idd)), 0.002)

    # ---- tack
    cs, ry, rz = [], [], []
    for i in range(13):
        x = lerp(-0.3, 0.32, i / 12); top = interp(body, x)[0]
        e = abs((x - 0.01) / 0.31) ** 4
        cs.append(V(B(x, top + 0.02 + 0.07 * e))); ry.append(0.2 + 0.03 * e); rz.append(0.045 + 0.04 * e)
    tack.loft(cs, ry, rz, sides=20)
    bit = Hd(0.545, -0.045)

    # ---- rider frame
    rp = -0.6 * pitch; lag = 0.03 * math.sin(TAU * (p - 0.12))
    cr, sr = math.cos(rp), math.sin(rp)
    def Rd(x, y): return B(x * cr - y * sr, 0.37 + lag + x * sr + y * cr)
    def flap(s, ph): return s * (0.08 * math.sin(TAU * 2 * p - s * 5 + ph) + 0.035 * math.sin(TAU * p - s * 9 + ph * 2))

    # torso
    tor = [(0.0, 0.0, 0.05, 0.19, 0.19), (0.3, 0.06, 0.3, 0.18, 0.17), (0.6, 0.15, 0.55, 0.21, 0.185), (0.85, 0.2, 0.7, 0.23, 0.155), (1.0, 0.21, 0.78, 0.16, 0.1)]
    tor = densify(tor, 6)
    cs, ry, rz = [], [], []
    for i in range(20):
        t = i / 19; x, y, a, b = interp(tor, t)
        cs.append(V(Rd(x, y))); ry.append(a); rz.append(b)
    rider.loft(cs, ry, rz, sides=22)
    rider.loft([V(Rd(0.12, 0.3)), V(Rd(0.05, 0.12)), V(Rd(-0.02, -0.02))], [0.21, 0.24, 0.25], [0.2, 0.22, 0.2], sides=22)
    rider.loft([V(Rd(0.2, 0.72)), V(Rd(0.21, 0.8)), V(Rd(0.215, 0.83))], [0.075, 0.07, 0.065], [0.065, 0.06, 0.055], sides=14)  # stump
    for side in (-1, 1):  # shoulders
        rider.loft([V(Rd(0.24, 0.7), side * 0.12), V(Rd(0.24, 0.69), side * 0.2)], [0.08, 0.075], [0.08, 0.075], sides=14)
    # legs & jackboots
    for side in (-1, 1):
        y0, y1 = side * 0.15, side * 0.31
        hip, knee, ank = Rd(0.0, 0.12), Rd(0.36, -0.1), Rd(0.31, -0.58)
        rider.loft([V(hip, y0), V(l2(hip, knee, 0.5), lerp(y0, y1, 0.6)), V(knee, y1)], [0.1, 0.085, 0.075], [0.105, 0.09, 0.078], sides=16)
        bd = nrm(sub(ank, knee))
        cuffTop = add(knee, bd, -0.1)
        rider.loft([V(cuffTop, y1), V(add(knee, bd, -0.06), y1), V(add(knee, bd, 0.05), y1), V(l2(knee, ank, 0.45), y1), V(l2(knee, ank, 0.8), y1), V(ank, y1)],
                   [0.1, 0.098, 0.07, 0.062, 0.052, 0.048], [0.105, 0.1, 0.075, 0.065, 0.052, 0.048], sides=16)
        rider.loft([V(Rd(0.24, -0.62), y1), V(Rd(0.36, -0.63), y1), V(Rd(0.5, -0.635), y1)], [0.05, 0.045, 0.035], [0.05, 0.042, 0.03], sides=14)
        tack.loft([V(B(-0.02, 0.33), side * 0.27), V(Rd(0.38, -0.6), y1 + side * 0.01)], [0.012, 0.012], [0.004, 0.004], sides=6, caps=False)
        st = [Rd(0.3, -0.7), Rd(0.48, -0.7), Rd(0.39, -0.56), Rd(0.3, -0.7)]
        tack.loft([V(q, y1) for q in st], [0.01] * 4, [0.01] * 4, sides=6)
        spur = Rd(0.14, -0.64)
        metal.loft([V(Rd(0.24, -0.63), y1), V(spur, y1)], [0.01, 0.008], [0.01, 0.008], sides=6)
        for k in range(5):
            a = k / 5 * math.pi + TAU * p
            metal.loft([V(add(spur, dirv(a), -0.03), y1), V(add(spur, dirv(a), 0.03), y1)], [0.005, 0.005], [0.005, 0.005], sides=4)
    # arms
    def arm(sh, el, wr, y0, y1):
        fa = nrm(sub(wr, el))
        cuff = add(wr, fa, -0.11); fist = add(wr, fa, 0.05)
        rider.loft([V(sh, y0), V(l2(sh, el, 0.5), lerp(y0, y1, 0.25)), V(el, lerp(y0, y1, 0.5)), V(add(cuff, fa, -0.02), y1)], [0.085, 0.078, 0.07, 0.062], [0.085, 0.078, 0.07, 0.062], sides=14)
        rider.loft([V(add(cuff, fa, -0.005), y1), V(cuff, y1), V(add(wr, fa, -0.02), y1), V(wr, y1), V(fist, y1), V(add(fist, fa, 0.04), y1)],
                   [0.06, 0.09, 0.07, 0.05, 0.056, 0.038], [0.06, 0.09, 0.07, 0.05, 0.054, 0.038], sides=14)
        return fist
    hand = arm(Rd(0.26, 0.68), Rd(0.44, 0.44), Rd(0.64, 0.38 + 0.015 * math.sin(TAU * p)), -0.21, -0.14)
    sw = 0.1 * math.sin(TAU * (p + 0.15))
    hand2 = arm(Rd(0.16, 0.7), Rd(0.06 + sw * 0.3, 1.0), Rd(0.2 + sw * 0.4, 1.2 + sw * 0.15), 0.21, 0.16)
    # reins
    for side in (-1, 1):
        a, b = hand, Hd(0.545, -0.045)
        mid = add(l2(a, b, 0.5), (0, -1), 0.06 * (1 + 0.5 * math.sin(TAU * 2 * p)))
        pts = [V(l2(l2(a, mid, t), l2(mid, b, t), t), lerp(-0.14 + side * 0.02, side * 0.07, t)) for t in [i / 10 for i in range(11)]]
        tack.loft(pts, [0.008] * 11, [0.008] * 11, sides=6)
    metal.loft([V(bit, -0.08), V(bit, 0.08)], [0.02, 0.02], [0.02, 0.02], sides=10)
    # sabre
    sa = 1.95 + sw * 0.9 + rp; sd = dirv(sa); sn = perp(sd); y = 0.16
    blade = [add(add(hand2, sd, 0.07 + s * 0.86), sn, 0.11 * s * s) for s in [i / 16 for i in range(17)]]
    metal.loft([V(b, y) for b in blade], [0.006] * 17, [lerp(0.019, 0.003, (i / 16) ** 3) for i in range(17)], sides=8)
    metal.loft([V(add(hand2, sd, -0.12), y), V(add(hand2, sd, 0.06), y)], [0.02, 0.018], [0.02, 0.018], sides=10)
    metal.loft([V(add(add(hand2, sd, 0.065), sn, 0.07), y), V(add(add(hand2, sd, 0.065), sn, -0.06), y)], [0.012, 0.012], [0.012, 0.012], sides=8)
    bow = [add(add(hand2, sd, lerp(0.06, -0.11, t)), sn, -0.02 - 0.08 * math.sin(math.pi * t)) for t in [i / 8 for i in range(9)]]
    metal.loft([V(b, y) for b in bow], [0.008] * 9, [0.008] * 9, sides=6)

    # ---- cloth: cloak, coat skirts, collar, cravat
    cuts = [0, 0.11, 0.19, 0.36, 0.47, 0.62, 0.7, 0.86, 1]
    def smax(v):
        for k in range(len(cuts) - 1):
            if v <= cuts[k + 1]:
                v0, v1 = cuts[k], cuts[k + 1]; wv = v1 - v0; tt = (v - v0) / wv
                ext = 0.98 + 0.5 * hash1(k * 4.1 + 1) * (0.4 + wv * 4) + 0.06 * math.sin(TAU * 2 * p + k * 1.7)
                notch = 0.8 + 0.12 * hash1(k * 2.3)
                return lerp(notch, ext, (1 - abs(tt - 0.5) * 2) ** 0.7)
        return 1.0
    def top(s): return Rd(0.1 - 1.3 * s, 0.8 + 0.04 * s - 0.36 * s * s + flap(s, 0) * 1.3)
    def bot(s): return Rd(-0.14 - 0.82 * s, 0.22 - 0.24 * s + flap(s, 0.9) * 1.2)
    def cloak(su, v):
        s = su * smax(v)
        p2 = l2(top(s), bot(s), v)
        y = (0.13 * math.sin(v * TAU * 1.6 + s * 4 - TAU * 2 * p) * min(1, s * 2.5)
             + 0.07 * math.sin(v * TAU * 3.1 - s * 6 + TAU * p)
             + (v - 0.5) * 0.55 * max(0, 1 - s * 2.2) + 0.05)
        return V(p2, y)
    cloth.grid(cloak, 36, 40)
    for side in (-1, 1):  # coat skirts
        def skirt(s, v, side=side):
            a = l2(Rd(0.25 - 0.4 * s, 0.24), Rd(0.3 - 0.95 * s, -0.08 + flap(s, 2 + side) - 0.04 * hash1(int(s * 9))), v)
            return V(a, side * (0.2 + 0.12 * v + 0.02 * math.sin(s * 9)))
        cloth.grid(skirt, 14, 6)
    def collar(s, v):  # torn ring standing around nothing
        a = TAU * s; c = Rd(0.21, 0.74 + v * (0.1 + 0.05 * hash1(int(s * 18) * 3.7)))
        r = 0.1 + 0.03 * v
        return Vector((c[0] + math.cos(a) * r * 0.85, math.sin(a) * r * 1.15, c[1]))
    cloth.grid(collar, 36, 3)
    def capelet(s, v):
        a = TAU * s; c = Rd(lerp(0.2, 0.16, v), lerp(0.76, 0.5, v))
        r = lerp(0.2, 0.3, v) + 0.015 * math.sin(a * 5 + v * 3)
        f = flap(0.3 + 0.2 * v, a) * v
        return Vector((c[0] + math.cos(a) * r * 0.82 - 0.04 * v, math.sin(a) * r * 1.05, c[1] + f))
    cloth.grid(capelet, 40, 6)
    for k in range(3):  # cravat rags
        cur = Rd(0.12 + k * 0.07, 0.83); ln = 0.34 + 0.22 * hash1(k * 6.1); pts = []
        for j in range(9):
            t = j / 8; pts.append(V(cur, -0.06 + k * 0.05))
            cur = add(cur, dirv(math.pi - 0.05 + rp + 0.3 * t + 0.4 * t * math.sin(TAU * 2 * p - t * 5 + k * 2)), ln / 8)
        hair.loft(pts, [0.006] * 9, [lerp(0.025 - k * 0.004, 0.003, j / 8) for j in range(9)], sides=6)

    return dict(Horse=horse, Hair=hair, Rider=rider, Cloth=cloth, Tack=tack, Metal=metal)

# ------------------------------------------------------------------ scene
def setup():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = 'CYCLES'
    try:
        prefs = bpy.context.preferences.addons['cycles'].preferences
        prefs.compute_device_type = 'METAL'; prefs.get_devices()
        for d in prefs.devices: d.use = True
        sc.cycles.device = 'GPU' if os.environ.get('HM_GPU') else 'CPU'
    except Exception as e:
        print('GPU setup failed', e)
    sc.cycles.samples = 96; sc.cycles.use_denoising = True
    sc.render.film_transparent = True
    sc.render.resolution_x, sc.render.resolution_y = FR_W, FR_H
    sc.render.resolution_percentage = 100
    sc.render.image_settings.file_format = 'PNG'; sc.render.image_settings.color_mode = 'RGBA'
    sc.view_settings.view_transform = 'Standard'; sc.view_settings.look = 'None'
    world = bpy.data.worlds.new('W'); sc.world = world; world.use_nodes = True
    world.node_tree.nodes['Background'].inputs[0].default_value = (0.06, 0.06, 0.06, 1)
    world.node_tree.nodes['Background'].inputs[1].default_value = 1.0

    cam = bpy.data.cameras.new('Cam'); cam.type = 'ORTHO'; cam.ortho_scale = FR_W / FR_S
    co = bpy.data.objects.new('Cam', cam); sc.collection.objects.link(co); sc.camera = co
    co.location = ((FR_W / 2 - FR_OX) / FR_S, -10, (FR_OY - FR_H / 2) / FR_S)
    co.rotation_euler = (math.pi / 2, 0, 0)

    def area(name, loc, power, size, target=(0, 0, 1.4)):
        L = bpy.data.lights.new(name, 'AREA'); L.energy = power; L.size = size
        o = bpy.data.objects.new(name, L); sc.collection.objects.link(o); o.location = loc
        d = Vector(target) - Vector(loc); o.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
    area('Rim', (-1.5, 5.0, 5.0), 7000, 2.5)     # behind & above: the forest light catching edges
    area('Kick', (3.5, 3.0, 2.5), 2200, 2.0)      # second back light from the front-side
    area('Fill', (-1.0, -6.0, 3.0), 180, 6.0)     # faint bounce from the camera side

    def mat(name, base, rough, metallic=0.0, sheen=0.0, spec=None):
        m = bpy.data.materials.new(name); m.use_nodes = True
        b = m.node_tree.nodes['Principled BSDF']
        b.inputs['Base Color'].default_value = (base, base, base, 1)
        b.inputs['Roughness'].default_value = rough
        b.inputs['Metallic'].default_value = metallic
        try:
            b.inputs['Sheen Weight'].default_value = sheen; b.inputs['Sheen Roughness'].default_value = 0.4
        except KeyError: pass
        if spec is not None:
            try: b.inputs['Specular IOR Level'].default_value = spec
            except KeyError: pass
        return m
    mats = dict(Horse=mat('Hide', 0.045, 0.55, sheen=0.4), Hair=mat('Hair', 0.03, 0.6, sheen=0.5, spec=0.25),
                Rider=mat('Wool', 0.03, 0.8, sheen=0.5), Cloth=mat('Cloak', 0.028, 0.95, sheen=0.35, spec=0.12),
                Tack=mat('Leather', 0.03, 0.5), Metal=mat('Steel', 0.35, 0.3, metallic=1.0))
    objs = {}
    for name in mats:
        me = bpy.data.meshes.new(name); o = bpy.data.objects.new(name, me); sc.collection.objects.link(o)
        me.materials.append(mats[name]); objs[name] = o
        if name in ('Horse', 'Rider'):
            r = o.modifiers.new('Remesh', 'REMESH'); r.mode = 'VOXEL'; r.voxel_size = 0.009 if name == 'Horse' else 0.008; r.use_smooth_shade = True
            s = o.modifiers.new('Smooth', 'SMOOTH'); s.factor = 0.7; s.iterations = 24
        if name == 'Cloth':
            s = o.modifiers.new('Solid', 'SOLIDIFY'); s.thickness = 0.012; s.offset = 0
            o.modifiers.new('Sub', 'SUBSURF').levels = 1
    return objs

def fill(obj, mb):
    me = obj.data; me.clear_geometry()
    me.from_pydata([tuple(v) for v in mb.v], [], mb.f); me.update()
    bm = bmesh.new(); bm.from_mesh(me); bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:]); bm.to_mesh(me); bm.free()
    for poly in me.polygons: poly.use_smooth = True

objs = setup()
frames = [ONLY] if ONLY is not None else range(NF)
for i in frames:
    parts = build(i / NF)
    for k, mb in parts.items(): fill(objs[k], mb)
    bpy.context.scene.render.filepath = os.path.join(OUT, f'f{i:02d}.png')
    bpy.ops.render.render(write_still=True)
    print('rendered', i)
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(os.path.dirname(OUT), 'horseman.blend'))
