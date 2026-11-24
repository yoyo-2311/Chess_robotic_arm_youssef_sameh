#!/usr/bin/env python3
"""Numerical FK envelope and capture-tray IK, with the supplied mounting faces."""
import argparse,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.optimize import differential_evolution
import xacro
from chess_motion_interface.geometry import Geometry
from chess_motion_interface.model import Model


def main():
 p=argparse.ArgumentParser();p.add_argument('--geometry',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
 ws=Path(__file__).resolve().parents[1];g=Geometry.load(a.geometry)
 root=ET.fromstring(xacro.process_file(str(ws/'src/dual_chess_description/urdf/dual_chess.urdf.xacro'),mappings={'geometry_file':str(a.geometry.resolve()),'include_board':'false'}).toxml());model=Model(root)
 report={'scope':'FK global numerical optimization (seeded), plus exact-chain endpoint IK. Not a collision/path proof.','arms':{}}
 for arm in ['robot1','robot2']:
  chain=[j for j in model.chain(arm+'_tool0') if j.get('type')!='fixed'];names=[j.get('name') for j in chain]
  bounds=[(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in chain]
  def z(q):return float(model.fk(arm+'_tool0',dict(zip(names,q)))[2])
  low=differential_evolution(z,bounds,seed=23,tol=1e-10,polish=True)
  high=differential_evolution(lambda q:-z(q),bounds,seed=23,tol=1e-10,polish=True)
  bins=[]
  for slot in range(g.raw['capture_bins'][arm]['slots']):
   local=g.bin_local(arm,slot)
   for stage,height in [('place',g.grasp('p')),('approach',g.travel(.026))]:
    target=g.world(local+[0,0,height]);solutions=model.solutions(arm,target)
    bins.append({'slot':slot,'stage':stage,'world':target.tolist(),'ik_solutions':len(solutions)})
  report['arms'][arm]={'tcp_z_min':float(low.fun),'tcp_z_max':float(-high.fun),'minimizer_joint_values':dict(zip(names,low.x.tolist())),
   'board_surface_z':float(g.square('a1')[2]),'highest_tested_board_approach_z':float(g.square('a1',g.travel(.026,1.))[2]),'capture_trays':bins}
  print(arm,report['arms'][arm]['tcp_z_min'],report['arms'][arm]['tcp_z_max'],flush=True)
 a.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
