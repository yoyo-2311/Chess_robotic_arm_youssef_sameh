"""Read-only CAD adapter; no ROS nodes, game logic, controller or GUI ownership."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def transform(xyz,rpy):
    t=np.eye(4);t[:3,3]=xyz;t[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();return t


def read_stl(path):
    data=Path(path).read_bytes()
    if len(data)<84:raise ValueError('Truncated binary STL')
    count=struct.unpack_from('<I',data,80)[0]
    if len(data)!=84+50*count:raise ValueError('Invalid binary STL length')
    raw=np.frombuffer(data,offset=84,count=count,dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')]))
    return raw['vertices'].astype(float)


class Assembly:
    def __init__(self,share=None):
        if share is None:
            from ament_index_python.packages import get_package_share_directory
            share=get_package_share_directory('chess_workcell_description')
        self.share=Path(share)
        self.urdf=ET.parse(self.share/'urdf/workcell.urdf').getroot()

    def components(self,configuration):
        """Return collision meshes with explicit world poses, never scale assets.

        The enclosure stays fixed in its configured CAD frame. The loose board
        follows the existing board pose so a future CV correction does not move
        the control box or the robot mounts.
        """
        b=configuration['board'];w=configuration['workcell']
        if not np.allclose([b['length'],b['width'],b['thickness'],b['surface_height']],[.28,.28,.01,.01],atol=1e-7):
            raise ValueError('CAD board is 0.28 x 0.28 x 0.01 m; use matching profile or a new CAD asset')
        cad=transform(w['position'],w['rpy'])
        board=transform(b['position'],b['rpy']) @ transform([0,0,b['surface_height']],[np.pi/2,0,0])
        return [('workcell_controle_box',self.share/'meshes'/w.get('mesh','controle_box.STL'),cad),
                ('chess_board_body',self.share/'meshes/chess_board.STL',board)]
