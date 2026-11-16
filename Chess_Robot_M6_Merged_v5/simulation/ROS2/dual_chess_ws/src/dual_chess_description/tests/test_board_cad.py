"""Configurable board regression tests; original CAD stays available as reference."""
from pathlib import Path
import xml.etree.ElementTree as E
import tempfile
import numpy as np
import pytest
import yaml
import xacro
from chess_motion_interface.geometry import Geometry
from chess_motion_interface.model import Model

DESC = Path(__file__).resolve().parents[1]
CONFIG = DESC/'config/chess_geometry.yaml'


def expanded(raw=None):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/'geometry.yaml'
        path.write_text(yaml.safe_dump(raw or Geometry.load(CONFIG).raw))
        return E.fromstring(xacro.process_file(str(DESC/'urdf/dual_chess.urdf.xacro'),mappings={'geometry_file':str(path)}).toxml())


def test_board_mesh_retained_and_low_surface_is_not_base_z():
    assert (DESC/'meshes/chess_board/Board_1_original.STL').stat().st_size > 100000
    g = Geometry.load(CONFIG)
    assert g.square('e4')[2] == pytest.approx(.01)
    s = g.raw['support']
    assert s['position'][2]+s['size'][2]/2 == pytest.approx(0.)
    assert g.raw['mounts']['robot1']['position'][2] == pytest.approx(.04)


@pytest.mark.parametrize('changed',[False,True])
def test_all_square_tf_centres_match_config_for_rectangular_rotated_board(changed):
    raw = Geometry.load(CONFIG).raw
    if changed:
        raw['board'].update(length=.28,width=.20,position=[.015,-.02,.003],rpy=[.02,0.,.25])
    g = Geometry(raw)
    root = expanded(raw)
    model = Model(root)
    assert len([l for l in root.findall('link') if l.get('name','').startswith('board_square_')]) == 64
    for f in 'abcdefgh':
        for r in '12345678':
            assert model.fk('board_square_'+f+r,{}) == pytest.approx(g.square(f+r),abs=1e-10)
    assert root.find("joint[@name='robot1_joint1']").get('type') == 'prismatic'
    assert root.find("joint[@name='robot1_joint1']/limit").get('upper') == '0.10'


def test_collision_dimensions_and_square_colours_follow_config():
    root = expanded()
    box = root.find("link[@name='board_link']/collision[@name='playing_area_collision']/geometry/box")
    assert np.fromstring(box.get('size'),sep=' ') == pytest.approx([.24,.24,.01])
    assert root.find("link[@name='chess_support_link']/collision") is not None
    assert {l.find('visual/material').get('name') for l in root.findall('link') if l.get('name','').startswith('board_square_')} == {'square_dark','square_light'}


def test_independent_square_pitches_and_separate_manual_launch():
    raw = Geometry.load(CONFIG).raw
    raw['board'].update(grid_mode='square_size',square_size_x=.022,square_size_y=.025)
    g = Geometry(raw)
    m = Model(expanded(raw))
    assert m.fk('board_square_h8',{}) == pytest.approx(g.square('h8'))
    launch = (DESC/'launch/display.launch.py').read_text()
    assert 'IfCondition' in launch and 'UnlessCondition' in launch
    assert 'joint_state_publisher_gui' in launch and 'controller_manager' not in launch
    assert (DESC/'launch/manual.launch.py').exists()


def test_zero_border_flat_board_has_no_zero_sized_rim_geometry():
    raw = Geometry.load(CONFIG).raw
    raw['board'].update(grid_mode='square_size',length=.21,width=.21)
    root = expanded(raw)
    for box in root.findall('.//box'):
        assert np.all(np.fromstring(box.get('size'),sep=' ')>0)
