#!/usr/bin/env python3
"""Retessellate measured circular pockets; preserve every other CAD triangle.

The original 22-sided pocket chords intrude ~1.4mm into a nominal 140mm radius
hole; the 139.7mm robot foot then falsely collides. This restores the measured
circle at <=0.01mm chord error, not a changed diameter or collision exemption.
"""
import json,struct
from pathlib import Path
import numpy as np
from chess_workcell_description.assembly import read_stl,Assembly
import yaml


def refine(triangles,t,centers,radius=.14,step=np.pi/180):
    world=(t @ np.column_stack([triangles.reshape(-1,3),np.ones(triangles.size//3)]).T).T[:,:3].reshape(-1,3,3)
    result=[];walls=set();changed=0
    def append(v,normal):
        if np.dot(np.cross(v[1]-v[0],v[2]-v[0]),normal)<0:v=v[[0,2,1]]
        result.append(v)
    for tri in world:
        normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);handled=False
        for index,center in enumerate(centers):
            center=np.asarray(center[:2]);dist=np.linalg.norm(tri[:,:2]-center,axis=1)
            ring=np.abs(dist-radius)<2e-7
            if not all(np.min(abs(tri[:,2,None]-np.array([-.033,-.03])),axis=1)<2e-7):continue
            if ring.all() and np.ptp(tri[:,2])>.002:
                walls.add(index);handled=True;changed+=1;break
            ids=np.flatnonzero(ring)
            if len(ids)!=2 or abs(tri[ids[0],2]-tri[ids[1],2])>2e-7:continue
            first,last=tri[ids];a,b=np.arctan2((np.array([first,last])[:,:2]-center)[:,1],(np.array([first,last])[:,:2]-center)[:,0])
            delta=(b-a+np.pi)%(2*np.pi)-np.pi
            if abs(abs(delta)-2*np.pi/22)>1e-5:continue
            count=int(np.ceil(abs(delta)/step));angle=np.linspace(a,a+delta,count+1)
            arc=np.column_stack([center[0]+radius*np.cos(angle),center[1]+radius*np.sin(angle),np.full(count+1,first[2])])
            # Original endpoints ensure watertight joins with unchanged triangles.
            arc[0]=first;arc[-1]=last;other=tri[next(i for i in range(3) if i not in ids)]
            for one,two in zip(arc[:-1],arc[1:]):append(np.array([other,one,two]),normal)
            handled=True;changed+=1;break
        if not handled:result.append(tri)
    assert walls=={0,1}
    # Use the exact same angular subdivision/endpoints as the neighboring faces.
    for center in centers:
        center=np.asarray(center[:2])
        v=world.reshape(-1,3);rim=v[(abs(np.linalg.norm(v[:,:2]-center,axis=1)-radius)<2e-7)&(abs(v[:,2]+.03)<2e-7)]
        rim=np.unique(rim,axis=0);angles=np.arctan2(rim[:,1]-center[1],rim[:,0]-center[0]);order=np.argsort(angles);rim=rim[order];angles=angles[order]
        for i,a in enumerate(angles):
            j=(i+1)%len(angles);b=angles[j]+(2*np.pi if j==0 else 0);count=int(np.ceil((b-a)/step));angle=np.linspace(a,b,count+1)
            xy=center+radius*np.column_stack([np.cos(angle),np.sin(angle)]);xy[0]=rim[i,:2];xy[-1]=rim[j,:2]
            for one,two in zip(xy[:-1],xy[1:]):
                low1=np.r_[one,-.033];low2=np.r_[two,-.033];high1=np.r_[one,-.03];high2=np.r_[two,-.03]
                inward=np.r_[center-(one+two)/2,0]
                append(np.array([low1,low2,high2]),inward);append(np.array([low1,high2,high1]),inward)
    result=np.asarray(result);inv=np.linalg.inv(t)
    native=(inv @ np.column_stack([result.reshape(-1,3),np.ones(result.size//3)]).T).T[:,:3].reshape(-1,3,3)
    return native,changed


def main():
    ws=Path(__file__).resolve().parents[1];package=ws/'src/chess_workcell_description'
    raw=yaml.safe_load((package/'config/workcell_sim.yaml').read_text());t=Assembly(package).components(raw)[0][2]
    triangles,changed=refine(read_stl(package/'meshes/controle_box.STL'),t,[[-.4,0],[.4,0]])
    path=package/'meshes/controle_box_refined.STL'
    with path.open('wb') as out:
        out.write(b'Measured 140mm mounting pockets; original preserved'.ljust(80,b' '));out.write(struct.pack('<I',len(triangles)))
        for tri in triangles:
            normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);normal/=max(np.linalg.norm(normal),1e-20)
            out.write(struct.pack('<12fH',*normal,*tri.reshape(-1),0))
    print(json.dumps({'output':str(path),'triangles':len(triangles),'original_triangles_retessellated':changed,'radius_m':.14,'max_chord_error_m':.14*(1-np.cos(np.pi/360))},indent=2))


if __name__=='__main__':main()
