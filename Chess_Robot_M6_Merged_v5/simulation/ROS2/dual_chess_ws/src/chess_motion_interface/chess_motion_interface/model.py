"""Exact supplied URDF chain, shared by old and new square tools."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

def transform(origin):
    t = np.eye(4)
    if origin is not None:
        t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        t[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    return t


class Model:
    def __init__(self, root):
        self.by_child = {j.find('child').get('link'): j for j in root.findall('joint')}

    def chain(self, link):
        chain = []
        while link in self.by_child:
            j = self.by_child[link]
            chain.append(j)
            link = j.find('parent').get('link')
        return list(reversed(chain))

    def matrix(self, link, q):
        t = np.eye(4)
        for j in self.chain(link):
            t = t @ transform(j.find('origin'))
            if j.get('type') == 'fixed':
                continue
            a = np.fromstring(j.find('axis').get('xyz'), sep=' ')
            v = np.eye(4)
            if j.get('type') == 'prismatic':
                v[:3, 3] = a * q[j.get('name')]
            else:
                v[:3, :3] = Rotation.from_rotvec(a * q[j.get('name')]).as_matrix()
            t = t @ v
        return t

    def fk(self, link, q):
        return self.matrix(link, q)[:3, 3]

    def solutions(self, arm, target):
        active = [j for j in self.chain(arm + '_tool0') if j.get('type') != 'fixed']
        names = [j.get('name') for j in active]
        lo = [float(j.find('limit').get('lower')) for j in active]
        hi = [float(j.find('limit').get('upper')) for j in active]
        result = []
        for a in [-2.5, 0., 2.5]:
            for b in [-2., 2.]:
                sol = least_squares(lambda q: self.fk(arm + '_tool0', dict(zip(names, q))) - target,
                                    [.05, a, b], bounds=(lo, hi), gtol=1e-10)
                error = float(np.linalg.norm(sol.fun))
                if error < 1e-4 and all(np.linalg.norm(sol.x - old['q']) > 1e-3 for old in result):
                    result.append({'q': sol.x.tolist(), 'error': error})
        return result

