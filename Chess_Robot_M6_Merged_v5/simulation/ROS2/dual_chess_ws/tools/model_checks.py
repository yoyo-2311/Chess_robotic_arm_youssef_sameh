#!/usr/bin/env python3
"""Geometry/kinematics checks of the dual-arm model (needs: xacro on PATH, numpy, yourdfpy, trimesh, python-fcl).
   pip install yourdfpy trimesh python-fcl numpy      (optional developer tools, not needed to build/run the workspace)

  python3 tools/model_checks.py [--design-urdf ../../urdf_convert/urdf/Assem4.SLDASM.urdf]

1. Design chain: robotN_base_link -> link0/1/2 transforms equal the Design-team URDF for random joint1/joint2.
2. Home pose: robot1 vs robot2 and each arm vs board (collision geometry); tool0 position.
3. Positional reach (NOT a motion plan): are all 64 square centres inside the band the tool tip can reach?
Read-only: nothing in the workspace is modified. Orientation, joint limits and collisions are not part of check 3."""
import argparse, os, re, subprocess, sys, tempfile
import numpy as np, trimesh, yourdfpy

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'src'); DESC = os.path.join(SRC, 'dual_chess_description')
ap = argparse.ArgumentParser(); ap.add_argument('--design-urdf', default=os.path.join(ROOT, '..', '..', 'urdf_convert', 'urdf', 'Assem4.SLDASM.urdf'))
a = ap.parse_args()
x = subprocess.run(['xacro', os.path.join(DESC, 'urdf', 'dual_chess.urdf.xacro')], capture_output=True, text=True, check=True).stdout
tmp = tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False); tmp.write(x); tmp.close()
fh = lambda fname: fname.replace('package://dual_chess_description/', DESC + '/')
kw = dict(build_scene_graph=True, build_collision_scene_graph=False, load_meshes=False, load_collision_meshes=False)
u = yourdfpy.URDF.load(tmp.name, filename_handler=fh, **kw)
names = u.actuated_joint_names; idx = {n: i for i, n in enumerate(names)}
def cfg(**k):
    c = np.zeros(u.num_dofs)
    for n, v in k.items():
        for jn in names:
            if jn.endswith(n): c[idx[jn]] = v
    return c
res = []
# 1 -------------------------------------------------------------------------------------------------
if os.path.isfile(a.design_urdf):
    dsd = os.path.dirname(os.path.dirname(os.path.abspath(a.design_urdf)))
    old = yourdfpy.URDF.load(a.design_urdf, filename_handler=lambda fname: fname.replace('package://Assem4.SLDASM/', dsd + '/'), **kw)
    rng = np.random.default_rng(0); worst = 0.0
    for _ in range(200):
        q = rng.uniform(-np.pi, np.pi, 2); old.update_cfg(q)
        for pre in ('robot1_', 'robot2_'):
            c = np.zeros(u.num_dofs); c[idx[pre + 'joint1']] = q[0]; c[idx[pre + 'joint2']] = q[1]; u.update_cfg(c)
            for l in ('link0', 'link1', 'link2'):
                worst = max(worst, np.abs(u.get_transform(pre + l, pre + 'base_link') - old.get_transform(l, 'base_link')).max())
    res.append(('Design chain equals Design URDF (200 random configs, both robots)', worst < 1e-9, f'max difference {worst:.1e}'))
else: res.append(('Design chain check', None, f'design URDF not found: {a.design_urdf}'))
# 2 -------------------------------------------------------------------------------------------------
cache = {}
def mgr(prefix, board=False):
    m = trimesh.collision.CollisionManager()
    for ln, l in u.link_map.items():
        if (board and ln != 'board_link') or (not board and not ln.startswith(prefix)): continue
        for g in l.collisions:
            gm = g.geometry
            if gm.box is not None: me = trimesh.creation.box(extents=gm.box.size)
            elif gm.mesh is not None:
                f = fh(gm.mesh.filename); me = cache.setdefault(f, trimesh.load(f, force='mesh')).copy()
            else: continue
            m.add_object(f'{ln}:{g.name}', me, u.get_transform(ln, 'world') @ (g.origin if g.origin is not None else np.eye(4)))
    return m
home = dict(base_yaw_joint=np.pi / 2, joint1=1.15, joint2=0.35)   # same targets as arm_cad.gazebo.xacro
for label, c in (('zero pose', cfg()), ('HOME pose', cfg(**home))):
    u.update_cfg(c); r1, r2, bd = mgr('robot1_'), mgr('robot2_'), mgr(None, True)
    hit = r1.in_collision_other(r2); d = r1.min_distance_other(r2)
    res.append((f'{label}: robot1 vs robot2 clear', not hit, f'min distance {d:.3f} m'))
    res.append((f'{label}: arms vs board collision geometry clear', not (r1.in_collision_other(bd) or r2.in_collision_other(bd)), ''))
u.update_cfg(cfg(**home))
for r in ('robot1', 'robot2'):
    p = u.get_transform(r + '_tool0', 'world')[:3, 3]; res.append((f'{r} tool0 at HOME (world, m)', None, str(np.round(p, 3))))
# 3 -------------------------------------------------------------------------------------------------
play = float(re.search(r'name="board_play_size" value="([\d.]+)"', open(os.path.join(DESC, 'urdf', 'board', 'board_params_cad.xacro')).read()).group(1))
sq = play / 8; cs = [-play / 2 + sq * (i + .5) for i in range(8)]
base = u.get_transform('robot1_base_yaw_link', 'world'); pts = []
for q1 in np.arange(-np.pi, np.pi, 0.1):
    for q2 in np.arange(-np.pi, np.pi, 0.1):
        for q3 in np.arange(-1.2, 1.21, 0.3):
            c = np.zeros(u.num_dofs); c[idx['robot1_joint1']] = q1; c[idx['robot1_joint2']] = q2; c[idx['robot1_joint3']] = q3
            u.update_cfg(c)
            pts.append(u.get_transform('robot1_tool0', 'robot1_base_yaw_link')[:3, 3])
w = (base[:3, :3] @ np.array(pts).T).T; rr = np.hypot(w[:, 0], w[:, 1]); zz = w[:, 2] + base[2, 3]
band = np.sort(rr[(zz >= 0.03) & (zz <= 0.06)])
bx = base[0, 3]; by = base[1, 3]
ok64 = sum(1 for x_ in cs for y_ in cs if np.min(np.abs(band - np.hypot(x_ - bx, y_ - by))) <= 0.02)
res.append((f'positional reach of robot1: {ok64}/64 square centres (playing area {play*100:.1f} cm, square {sq*100:.3f} cm) inside the tool-tip reach band at z 3-6 cm', ok64 == 64, f'band {band.min():.2f}..{band.max():.2f} m; positions only'))
for t, okf, d in res: print(f"[{'PASS' if okf else 'INFO' if okf is None else 'FAIL'}] {t}  {d}")
sys.exit(0 if all(r[1] is not False for r in res) else 1)
