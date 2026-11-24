#!/usr/bin/env python3
"""Reproducible STL measurements. Yaw and arm/seat identity need CAD mates."""
import argparse,json,hashlib
from pathlib import Path
import yaml
from chess_workcell_description.assembly import Assembly
from chess_workcell_description.mounting import measure_seats,measure_base,mount_for_seat


def derive(profile,package,base_mesh):
    raw=yaml.safe_load(profile.read_text())
    box=Assembly(package).components(raw)[0]
    seats=measure_seats(package/'meshes/controle_box.STL',box[2]);base=measure_base(base_mesh)
    first=raw.get('mounting_evidence',{}).get('robot1_seat','A')
    order=seats if first=='A' else list(reversed(seats))
    mounts={arm:mount_for_seat(seat,base,raw['mounts'][arm]['rpy'][2])
            for arm,seat in zip(['robot1','robot2'],order)}
    return {'reference_frame':'world','units':'metres/radians','seats':seats,'physical_base_face':base,
            'mounts':mounts,'yaw_status':'Unconfirmed: retained configured yaw; unkeyed circular platform has no mating frame or bolt pattern',
            'robot1_seat':first,'assignment_confirmed':False,
            'assignment_status':'UNCONFIRMED preview: robot1/White on '+first+'; robot2/Black on '+('B' if first=='A' else 'A')+'; user has no seat identity information yet',
            'mesh_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [package/'meshes/controle_box.STL',box[1],base_mesh]}}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--geometry',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--apply',action='store_true');a=p.parse_args()
    ws=Path(__file__).resolve().parents[1]
    report=derive(a.geometry,ws/'src/chess_workcell_description',ws/'src/dual_chess_description/meshes/scara_v2/base_link.STL')
    if a.apply:
        raw=yaml.safe_load(a.geometry.read_text());raw['mounts']=report['mounts']
        raw['mounting_evidence']={'reference_frame':'world','yaw_status':report['yaw_status'],'assignment_status':report['assignment_status'],'robot1_seat':report['robot1_seat'],'assignment_confirmed':False}
        a.geometry.write_text('# Centers/planes measured from CAD; yaw/seat identity unconfirmed. See docs/14_mount_alignment.md.\n'+yaml.safe_dump(raw,sort_keys=False))
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
