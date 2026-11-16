"""Shared board mapping. All dimensions are configurable simulation placeholders."""
import copy
import array
import math
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def matrix(position, rpy):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    t[:3, 3] = position
    return t


class Geometry:
    def __init__(self, raw):
        def canonical(value):
            if isinstance(value,dict):return {k:canonical(v) for k,v in value.items()}
            if isinstance(value,(list,tuple,array.array,np.ndarray)):return [canonical(v) for v in value]
            if isinstance(value,np.generic):return value.item()
            return copy.deepcopy(value)
        self.raw = canonical(raw)
        b = self.raw['board']
        if b.get('grid_mode', 'square_size') not in ['fit_board','square_size']:
            raise ValueError('grid_mode must be fit_board or square_size')
        for key in ['border_x','border_y']:
            if key in b and (not math.isfinite(b[key]) or b[key] < 0):
                raise ValueError('board borders must be finite and nonnegative')
        if b.get('grid_mode') == 'fit_board':
            b['square_size_x'] = (b['length']-2*b['border_x'])/8
            b['square_size_y'] = (b['width']-2*b['border_y'])/8
        def positive(value, label):
            if not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(f'{label} must be finite and positive')
        for key in ['length', 'width', 'square_size_x', 'square_size_y', 'thickness', 'surface_height']:
            positive(b[key], 'board.' + key)
        if b['length'] < 8*b['square_size_x'] or b['width'] < 8*b['square_size_y']:
            raise ValueError('board length/width must contain eight configured squares')
        if b['surface_height'] > b['thickness']:
            raise ValueError('playing surface cannot exceed board thickness')
        for section in [b, self.raw['support'], *self.raw['mounts'].values()]:
            for key in ['position', 'rpy']:
                if len(section[key]) != 3 or not all(math.isfinite(float(x)) for x in section[key]):
                    raise ValueError(f'{key} must contain three finite values')
        for h in self.raw['pieces']['heights'].values():
            positive(h, 'piece height')
        positive(self.raw['pieces']['radius'], 'piece radius')
        for v in self.raw['support']['size']:
            positive(v, 'support size')
        for key in ['approach_clearance', 'transfer_clearance', 'place_clearance', 'cartesian_step', 'operator_timeout', 'max_board_tilt']:
            positive(self.raw['motion'][key], key)
        for key in ['velocity_scale', 'acceleration_scale', 'grasp_fraction', 'minimum_cartesian_fraction']:
            v = self.raw['motion'][key]
            if not 0 < v <= 1:
                raise ValueError(f'{key} must be in (0,1]')
        for arm, bin in self.raw['capture_bins'].items():
            if not isinstance(bin['slots'], int) or bin['slots'] < 1:
                raise ValueError('capture bin slots must be positive integers')
            if not isinstance(bin.get('columns'),int) or bin['columns'] < 1:
                raise ValueError('bin columns must be a positive integer')
            for k in ['origin', 'step', 'row_step']:
                if len(bin[k]) != 3 or not all(math.isfinite(float(v)) for v in bin[k]):
                    raise ValueError('invalid bin geometry')
        self.T = matrix(b['position'], b['rpy'])
        support = self.raw['support']
        if support['enabled']:
            # World objects do not collide with each other in MoveIt. Reject a board
            # embedded in its supporting platform explicitly (conservative OBB projection).
            inv = np.linalg.inv(matrix(support['position'],support['rpy']))
            corners = np.array([(inv @ self.T @ np.array([x,y,z,1.]))[:3]
                                for x in [-b['length']/2,b['length']/2]
                                for y in [-b['width']/2,b['width']/2] for z in [0.,b['thickness']]])
            low,high = corners.min(0),corners.max(0)
            half = np.array(support['size'])/2
            if np.all(high > -half+1e-6) and np.all(low < half-1e-6):
                raise ValueError('board_intersects_support: adjust the low board pose or support geometry')

    @classmethod
    def load(cls, path):
        with open(path) as stream:
            return cls(yaml.safe_load(stream))

    def world(self, point):
        return (self.T @ np.array([*point, 1.]))[:3]

    def local_square(self, square, height=0.):
        if not isinstance(square, str) or len(square) != 2 or square[0] not in 'abcdefgh' or square[1] not in '12345678':
            raise ValueError('invalid_square: expected a1 through h8')
        b = self.raw['board']
        return np.array([(ord(square[0])-ord('a')-3.5)*b['square_size_x'],
                         (int(square[1])-1-3.5)*b['square_size_y'], b['surface_height'] + height])

    def square(self, square, height=0.):
        return self.world(self.local_square(square, height))

    def piece_height(self, symbol):
        return self.raw['pieces']['heights'][symbol.lower()]

    def grasp(self, symbol):
        return self.piece_height(symbol)*self.raw['motion']['grasp_fraction']

    def travel(self, carried_height=0.):
        m = self.raw['motion']
        return max(self.raw['pieces']['heights'].values()) + max(m['transfer_clearance'],m['approach_clearance']) + carried_height*m['grasp_fraction']

    def bin_local(self, arm, slot):
        b = self.raw['capture_bins'][arm]
        if slot >= b['slots']:
            raise ValueError('capture_bin_full')
        return np.array(b['origin']) + (slot%b['columns'])*np.array(b['step']) + (slot//b['columns'])*np.array(b['row_step'])

    def target_in_frame(self, square, frame_T_world, height=0.):
        return (np.linalg.inv(frame_T_world) @ np.array([*self.square(square, height), 1.]))[:3]
