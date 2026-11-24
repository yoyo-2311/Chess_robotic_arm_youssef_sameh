#!/usr/bin/env python3
"""IK-only audit of the actual Xacro model; NOT a collision/execution proof."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory
from chess_motion_interface.geometry import Geometry
from chess_motion_interface.model import Model


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    profile=args.geometry.resolve();geometry=Geometry.load(profile)
    description=Path(get_package_share_directory('dual_chess_description'))/'urdf/dual_chess.urdf.xacro'
    model=Model(ET.fromstring(xacro.process_file(str(description),mappings={
        'geometry_file':str(profile),'include_board':'false'}).toxml()))
    report={'geometry':str(profile),'sha256':hashlib.sha256(profile.read_bytes()).hexdigest(),
            'scope':'Position IK only; does not certify collision clearance or trajectories','arms':{}}
    for arm in ['robot1','robot2']:
        failures=[];tested=0;cache={}
        for file in 'abcdefgh':
            for rank in '12345678':
                square=file+rank
                for stage,height in [('pawn_grasp',geometry.grasp('p')),('knight_grasp',geometry.grasp('n')),('pawn_place',geometry.grasp('p')+geometry.raw['motion']['place_clearance']),('knight_place',geometry.grasp('n')+geometry.raw['motion']['place_clearance']),('approach',geometry.travel(.026,max(geometry.grasp_fractions()))),('retreat',geometry.travel(.026,max(geometry.grasp_fractions())))]:
                    tested+=1;target=geometry.square(square,height)
                    key=tuple(target)
                    if key not in cache:cache[key]=bool(model.solutions(arm,target))
                    if not cache[key]:failures.append({'square':square,'stage':stage,'world':target.tolist()})
        report['arms'][arm]={'tested':tested,'unreachable':failures}
        print(arm,tested,'targets;',len(failures),'IK failures',flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
