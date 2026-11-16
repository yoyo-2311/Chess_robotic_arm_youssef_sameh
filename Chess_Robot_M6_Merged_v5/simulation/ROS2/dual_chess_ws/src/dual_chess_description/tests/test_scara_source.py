"""Check that integration retains the supplied CAD mechanics and mesh bytes."""
from pathlib import Path
import xml.etree.ElementTree as E

DESC = Path(__file__).resolve().parents[1]
X = {'x': 'http://www.ros.org/wiki/xacro'}


def test_source_joint_transforms_limits_and_inertias_preserved():
    source = E.parse(DESC / 'reference/scara_v2/urdf/scara_v2.urdf').getroot()
    macro = E.parse(DESC / 'urdf/arm/scara_v2.macro.xacro').getroot().find("x:macro[@name='scara_chess_arm']", X)
    for joint in source.findall('joint'):
        if joint.get('name') == 'world_to_base':
            continue
        integrated = macro.find("joint[@name='${prefix}" + joint.get('name') + "']")
        assert integrated is not None
        assert integrated.get('type') == joint.get('type')
        for tag in ['origin', 'axis', 'limit']:
            a, b = joint.find(tag), integrated.find(tag)
            assert (a.attrib if a is not None else None) == (b.attrib if b is not None else None)
    for link in source.findall('link'):
        inertia = link.find('inertial')
        if inertia is None:
            continue
        integrated = macro.find("link[@name='${prefix}" + link.get('name') + "']/inertial")
        assert E.tostring(inertia).strip() == E.tostring(integrated).strip()
    for mesh in (DESC / 'reference/scara_v2/meshes').glob('*.STL'):
        assert mesh.read_bytes() == (DESC / 'meshes/scara_v2' / mesh.name).read_bytes()
