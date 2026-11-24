"""Measure exported mounting faces; never infer clocking from round features."""
import numpy as np
from scipy.spatial import ConvexHull, QhullError
from .assembly import read_stl, transform


def fit_outline(points):
    """Fit the convex polygon rim, excluding interior triangulation/hole points."""
    rim=points[ConvexHull(points).vertices]
    design=np.column_stack([2*rim[:,0],2*rim[:,1],np.ones(len(rim))])
    x,y,c=np.linalg.lstsq(design,np.sum(rim*rim,axis=1),rcond=None)[0]
    radius=float(np.sqrt(c+x*x+y*y))
    error=float(np.max(np.abs(np.linalg.norm(rim-[x,y],axis=1)-radius)))
    if error>2e-6:raise ValueError('Mount outline is not circular within STL precision')
    return np.array([x,y]),radius,error,len(rim)


def measure_seats(mesh,world_from_mesh):
    vertices=read_stl(mesh).reshape(-1,3)
    vertices=(world_from_mesh @ np.column_stack([vertices,np.ones(len(vertices))]).T).T[:,:3]
    # The exported horizontal disk pockets have 140mm rims. Search the mesh
    # planes; do not supply a guessed center or elevate the platform.
    vertices=np.unique(vertices,axis=0)
    seats=[]
    for z in np.unique(np.round(vertices[:,2],6)):
        plane=vertices[np.abs(vertices[:,2]-z)<3e-7]
        for sign in [-1,1]:
            side=plane[plane[:,0]*sign>.15]
            if len(side)<16:continue
            try:center,radius,error,count=fit_outline(side[:,:2])
            except (ValueError,QhullError):
                # Degenerate/noncircular planes are not mounting disks.
                continue
            if abs(radius-.14)>2e-6:continue
            # Filled disk, rather than the upper edge of the cylindrical wall.
            if np.min(np.linalg.norm(side[:,:2]-center,axis=1))>2e-6:continue
            seats.append({'center':[float(center[0]),float(center[1]),float(np.median(side[:,2]))],
                          'radius':radius,'fit_max_error':error,'rim_vertices':count})
    if len(seats)!=2:raise ValueError(f'Expected two exported mounting disks, found {len(seats)}')
    return sorted(seats,key=lambda x:x['center'][0])


def measure_base(mesh):
    vertices=np.unique(read_stl(mesh).reshape(-1,3),axis=0)
    # Native +Y is down after the existing world_to_base Rx(-pi/2).
    bottom=float(vertices[:,1].max())
    face=vertices[np.abs(vertices[:,1]-bottom)<3e-7]
    center,radius,error,count=fit_outline(face[:,[0,2]])
    return {'center_native':[float(center[0]),bottom,float(center[1])],
            'radius':radius,'fit_max_error':error,'rim_vertices':count,
            'normal_native':[0.,1.,0.]}


def mount_for_seat(seat,base,yaw):
    # Existing arm Xacro applies Rx(-pi/2) between mount and base_link.
    rotation=transform([0,0,0],[0,0,yaw]) @ transform([0,0,0],[-np.pi/2,0,0])
    position=np.asarray(seat['center'])-rotation[:3,:3] @ base['center_native']
    return {'position':position.tolist(),'rpy':[0.,0.,float(yaw)]}
