#!/usr/bin/env python3
"""Validate a Design-team URDF export (e.g. SolidWorks sw_urdf_exporter) before integrating it.
Needs only numpy (+ binary STL meshes). No ROS required.

  python3 tools/validate_design_urdf.py <urdf> [--mesh-root <dir containing the package folder contents>]
Example:
  python3 tools/validate_design_urdf.py simulation/urdf_convert/urdf/Assem4.SLDASM.urdf \
          --mesh-root simulation/urdf_convert

Reports: structure (unique names, single root, tree), DOF and joint types, missing limits, mesh files,
mesh units/size, mass/volume density, COM-vs-mesh-centroid mismatch, inertia plausibility (positive definite,
triangle inequality), duplicate geometry between links (same part exported twice), zero-pose link frames/axes.
It reports facts and suspicious values; it never edits anything. Exit code 1 only for hard errors.
"""
import argparse, os, re, struct, sys, xml.etree.ElementTree as ET
import numpy as np

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @ np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]) @ np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]))
def vec(e, key, n=3, default=0.0):
    return [float(x) for x in (e.get(key) or ' '.join([str(default)]*n)).split()] if e is not None else [default]*n
def pose(origin):
    T = np.eye(4); T[:3,:3] = rpy(*vec(origin,'rpy')); T[:3,3] = vec(origin,'xyz'); return T
def load_stl(path):
    d = open(path,'rb').read(); n = struct.unpack('<I', d[80:84])[0]
    if 84 + 50*n != len(d): raise ValueError('not a binary STL')
    a = np.frombuffer(d[84:], dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]), count=n)
    return a['v'].reshape(-1,3,3).astype(float)
def mesh_props(t):
    v6 = np.einsum('ij,ij->i', t[:,0], np.cross(t[:,1], t[:,2])); vol = v6.sum()/6
    return vol, ((t[:,0]+t[:,1]+t[:,2])/4*v6[:,None]).sum(0)/(6*vol)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('urdf'); ap.add_argument('--mesh-root', default=None)
    a = ap.parse_args(); errs, warns = [], []
    root = ET.parse(a.urdf).getroot()
    links = {l.get('name'): l for l in root.findall('link')}; joints = root.findall('joint')
    print(f"robot: {root.get('name')}   links={len(links)} joints={len(joints)}")
    for n, c in __import__('collections').Counter([l.get('name') for l in root.findall('link')]+[j.get('name') for j in joints]).items():
        if c > 1: errs.append(f'duplicate name {n}')
    parent = {}; kids = {}
    for j in joints:
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        for x in (p, c):
            if x not in links: errs.append(f"joint {j.get('name')} references missing link {x}")
        if c in parent: errs.append(f'link {c} has two parents')
        parent[c] = j; kids.setdefault(p, []).append(c)
    roots = [l for l in links if l not in parent]
    if len(roots) != 1: errs.append(f'expected 1 root, got {roots}')
    root_link = roots[0] if roots else None
    types = {j.get('name'): j.get('type') for j in joints}
    act = [n for n, t in types.items() if t not in ('fixed',)]
    print(f"joint types: {types}\nactuated joints (DOF = {len(act)}): {act}")
    for j in joints:
        t = j.get('type'); ax = vec(j.find('axis'), 'xyz', default=0.0) if j.find('axis') is not None else None
        if t != 'fixed' and (ax is None or np.linalg.norm(ax) < 1e-9): errs.append(f"{j.get('name')}: movable joint without valid axis")
        if t == 'fixed' and ax is not None and np.linalg.norm(ax) < 1e-9: warns.append(f"{j.get('name')}: fixed joint carries a zero <axis> (harmless, non-conformant)")
        lim = j.find('limit')
        if t == 'continuous': warns.append(f"{j.get('name')}: continuous (unlimited) joint, no <limit> -> no effort/velocity/position limits")
        elif t in ('revolute','prismatic') and lim is None: errs.append(f"{j.get('name')}: {t} without <limit>")
    # zero-pose frames
    W = {root_link: np.eye(4)}; order = [root_link]
    while order:
        l = order.pop(0)
        for c in kids.get(l, []):
            W[c] = W[l] @ pose(parent[c].find('origin')); order.append(c)
    print("\nzero-pose link frames in the root frame (origin | z-axis):")
    for l, T in W.items(): print(f"  {l:10s} {np.round(T[:3,3],4)} | z={np.round(T[:3,2],4)}")
    mroot = a.mesh_root or os.path.dirname(os.path.dirname(a.urdf)); meshes = {}
    print("\nper-link mesh / mass checks:")
    for name, l in links.items():
        m = l.find('visual/geometry/mesh'); I = l.find('inertial')
        line = f"  {name}: "
        if m is not None:
            uri = m.get('filename'); rel = re.sub(r'^package://[^/]+/', '', uri); fp = os.path.join(mroot, rel)
            if not os.path.isfile(fp): errs.append(f'{name}: mesh missing {fp}'); print(line + 'MESH MISSING'); continue
            tri = load_stl(fp); v = tri.reshape(-1,3); sz = v.max(0)-v.min(0); vol, cen = mesh_props(tri); meshes[name] = (tri, W[name])
            line += f"mesh size {np.round(sz,4)} m (units look {'metric/metres' if sz.max()<2 else 'NOT metres?'}), volume {vol:.3e} m3; "
            if I is not None:
                mass = float(I.find('mass').get('value')); com = np.array(vec(I.find('origin'),'xyz'))
                dens = mass/vol; dm = np.linalg.norm(com-cen)
                line += f"mass {mass:.4f} kg -> density {dens:.0f} kg/m3; COM-to-mesh-centroid {dm:.4f} m"
                if dm > 0.02: errs.append(f'{name}: inertial COM is {dm:.3f} m from its own mesh centroid (wrong frame in export?)')
                i = I.find('inertia'); ixx,ixy,ixz,iyy,iyz,izz = [float(i.get(k)) for k in ('ixx','ixy','ixz','iyy','iyz','izz')]
                Im = np.array([[ixx,ixy,ixz],[ixy,iyy,iyz],[ixz,iyz,izz]]); ev = np.linalg.eigvalsh(Im)
                if ev.min() <= 0: errs.append(f'{name}: inertia not positive definite')
                if not (ixx+iyy >= izz and ixx+izz >= iyy and iyy+izz >= ixx): errs.append(f'{name}: inertia violates triangle inequality')
        else: line += 'no mesh'
        if I is None: warns.append(f'{name}: no <inertial>')
        print(line)
    names = list(meshes)
    print("\nduplicate-geometry scan (same mesh volume at the same place in the root frame):")
    dup = False
    for i, p in enumerate(names):
        for q in names[i+1:]:
            (tp, Tp), (tq, Tq) = meshes[p], meshes[q]; vp, cp = mesh_props(tp); vq, cq = mesh_props(tq)
            if abs(vp-vq) < 1e-9 and np.linalg.norm((Tp[:3,:3]@cp+Tp[:3,3]) - (Tq[:3,:3]@cq+Tq[:3,3])) < 1e-3:
                dup = True; warns.append(f'{p} and {q} are the SAME part at the SAME place (exported twice) -> double mass/collision')
                print(f"  DUPLICATE: {p} == {q}")
    if not dup: print("  none")
    print("\nWARNINGS:"); [print("  -", w) for w in warns]
    print("ERRORS:"); [print("  -", e) for e in errs] if errs else print("  none")
    return 1 if [e for e in errs if 'inertial COM' not in e] else 0
if __name__ == '__main__': sys.exit(main())
