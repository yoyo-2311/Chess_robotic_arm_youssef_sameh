"""Owned planning-scene objects, including support, configurable board and pieces."""
import copy
from contextlib import contextmanager
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld, CollisionObject, AttachedCollisionObject, ObjectColor, AllowedCollisionEntry, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from chess_brain_handover.core import chess


def pose(position, rotation=None):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, position)
    q = [0., 0., 0., 1.] if rotation is None else Rotation.from_matrix(rotation).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    return p


def object_msg(name, dimensions, position, rotation, kind=SolidPrimitive.BOX):
    obj = CollisionObject(id=name)
    obj.header.frame_id = 'world'
    obj.operation = CollisionObject.ADD
    obj.primitives = [SolidPrimitive(type=kind, dimensions=list(map(float, dimensions)))]
    obj.primitive_poses = [pose(position, rotation)]
    return obj


class Scene:
    def __init__(self, backend, geometry):
        self.backend, self.geometry = backend, geometry
        self.ids = set()
        self.at = {}
        self.pieces = {}
        self.attached = None
        self.get_client = backend.node.create_client(GetPlanningScene, '/get_planning_scene', callback_group=backend.node.io_group)

    def initialize(self, fen=None):
        """Atomic replacement of only this module's owned geometry. Caller holds operation lock."""
        g = self.geometry
        b = g.raw['board']
        scene = PlanningScene(is_diff=True)
        for id in self.ids:
            scene.world.collision_objects.append(CollisionObject(id=id, operation=CollisionObject.REMOVE))
        offboard = {k:copy.deepcopy(v) for k,v in self.pieces.items() if k.startswith('chess_bin_')}
        self.ids, self.at, self.pieces = set(), {}, {}
        def add(obj, rgba):
            scene.world.collision_objects.append(obj)
            color = ObjectColor(id=obj.id)
            color.color.r, color.color.g, color.color.b, color.color.a = rgba
            scene.object_colors.append(color)
            self.ids.add(obj.id)
        rot = g.T[:3, :3]
        if g.raw.get('workcell',{}).get('enabled',False):
            from chess_workcell_description.assembly import Assembly,read_stl
            from shape_msgs.msg import Mesh,MeshTriangle
            from geometry_msgs.msg import Point
            for name,path,t in Assembly().components(g.raw):
                triangles=read_stl(path);mesh=Mesh()
                for triangle in triangles:
                    index=len(mesh.vertices)
                    mesh.vertices.extend(Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in triangle)
                    mesh.triangles.append(MeshTriangle(vertex_indices=[index,index+1,index+2]))
                obj=CollisionObject(id=name);obj.header.frame_id='world';obj.operation=CollisionObject.ADD
                obj.meshes=[mesh];obj.mesh_poses=[pose(t[:3,3],t[:3,:3])]
                add(obj,[.45,.5,.55,1.] if name=='workcell_controle_box' else [.45,.28,.12,1.])
        else:
            add(object_msg('chess_board_body', [b['length'], b['width'], b['surface_height']], g.world([0, 0, b['surface_height']/2]), rot), [.45,.28,.12,1.])
        rx, ry = (b['length']-8*b['square_size_x'])/2, (b['width']-8*b['square_size_y'])/2
        for name, size, centre in [
            ('left', [rx,b['width'],b['thickness']-b['surface_height']], [-b['length']/2+rx/2,0,(b['surface_height']+b['thickness'])/2]),
            ('right',[rx,b['width'],b['thickness']-b['surface_height']], [b['length']/2-rx/2,0,(b['surface_height']+b['thickness'])/2]),
            ('front',[8*b['square_size_x'],ry,b['thickness']-b['surface_height']], [0,-b['width']/2+ry/2,(b['surface_height']+b['thickness'])/2]),
            ('back',[8*b['square_size_x'],ry,b['thickness']-b['surface_height']], [0,b['width']/2-ry/2,(b['surface_height']+b['thickness'])/2])]:
            if min(size) > 0:
                add(object_msg('chess_rim_'+name,size,g.world(centre),rot), [.4,.25,.1,1.])
        for f in 'abcdefgh':
            for r in '12345678':
                sq = f+r
                add(object_msg('chess_tile_'+sq,[b['square_size_x']*.98,b['square_size_y']*.98,.0002],g.square(sq, -.0001),rot), [.85,.8,.65,1.] if (ord(f)+int(r))%2 else [.12,.12,.12,1.])
        support = g.raw['support']
        if support['enabled']:
            add(object_msg('chess_support',support['size'],support['position'],Rotation.from_euler('xyz',support['rpy']).as_matrix()), [.3,.35,.4,1.])
        for arm, bin in g.raw['capture_bins'].items():
            # Small synthetic tray underneath the configured slot centres, not measured hardware.
            for slot in range(bin['slots']):
                local = g.bin_local(arm,slot)
                add(object_msg(f'chess_tray_{arm}_{slot}',[.01,.01,.004],g.world(local-[0,0,.002]),rot),[.35,.4,.45,1.])
        for name, piece in offboard.items():
            _,_,arm,slot = name.split('_')
            local = g.bin_local(arm,int(slot))+[0,0,g.piece_height(piece['symbol'])/2+g.raw['motion']['place_clearance']]
            piece['local'] = local.tolist()
            self.pieces[name] = piece
            add(self.piece_object(name,piece['symbol'],local),[.6,.6,.6,1.])
        if fen:
            board = chess.Board(fen)
            for idx, piece in board.piece_map().items():
                sq, symbol = chess.square_name(idx), piece.symbol()
                name = 'chess_piece_'+sq
                local = g.local_square(sq,g.piece_height(symbol)/2)
                obj = self.piece_object(name,symbol,local)
                self.pieces[name] = {'symbol':symbol,'local':local.tolist()}
                self.at[sq] = name
                add(obj,[.95,.95,.95,1.] if piece.color else [.15,.15,.2,1.])
        self.backend.apply(scene)

    def piece_object(self, name, symbol, local):
        g = self.geometry
        return object_msg(name,[g.piece_height(symbol),g.raw['pieces']['radius']],g.world(local),g.T[:3,:3],SolidPrimitive.CYLINDER)

    def touch(self, name, arm, allowed):
        req = GetPlanningScene.Request(components=PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))
        matrix = self.backend.wait(self.get_client.call_async(req)).scene.allowed_collision_matrix
        for key in [name,arm+'_link4',arm+'_link5',arm+'_tool0']:
            if key not in matrix.entry_names:
                matrix.entry_names.append(key)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        i = matrix.entry_names.index(name)
        for link in [arm+'_link4',arm+'_link5',arm+'_tool0']:
            j = matrix.entry_names.index(link)
            matrix.entry_values[i].enabled[j] = matrix.entry_values[j].enabled[i] = allowed
        self.backend.apply(PlanningScene(is_diff=True, allowed_collision_matrix=matrix))

    @contextmanager
    def preview_attachment(self, name, arm, grasp_state):
        """Scoped planner-only payload. No robot/gripper command or FEN change.

        The caller holds the motion lock. Build the attachment in tool-local
        coordinates at the hypothetical grasp, not at the parked live TCP.
        Restore the source object even when feasibility checks fail. An uncertain
        scene update propagates as a blocking error, never a safe rejection.
        """
        if self.attached is not None:
            raise RuntimeError('Cannot preview over a real attachment')
        req=GetPlanningScene.Request(components=PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))
        matrix=copy.deepcopy(self.backend.wait(self.get_client.call_async(req)).scene.allowed_collision_matrix)
        piece=self.pieces[name]
        original=self.piece_object(name,piece['symbol'],piece['local'])
        joints=dict(zip(grasp_state.joint_state.name,grasp_state.joint_state.position))
        tool=self.backend.model.matrix(arm+'_tool0',joints)
        world=np.eye(4);world[:3,:3]=self.geometry.T[:3,:3];world[:3,3]=self.geometry.world(piece['local'])
        relative=np.linalg.inv(tool) @ world
        obj=copy.deepcopy(original);obj.header.frame_id=arm+'_tool0'
        obj.primitive_poses=[pose(relative[:3,3],relative[:3,:3])]
        try:
            self.backend.apply(PlanningScene(is_diff=True,world=PlanningSceneWorld(collision_objects=[CollisionObject(id=name,operation=CollisionObject.REMOVE)])))
            scene=PlanningScene(is_diff=True);scene.robot_state.is_diff=True
            scene.robot_state.attached_collision_objects=[AttachedCollisionObject(link_name=arm+'_tool0',object=obj,touch_links=[arm+'_tool0',arm+'_link4',arm+'_link5'])]
            self.backend.apply(scene)
            yield
        finally:
            scene=PlanningScene(is_diff=True);scene.robot_state.is_diff=True
            scene.robot_state.attached_collision_objects=[AttachedCollisionObject(link_name=arm+'_tool0',object=CollisionObject(id=name,operation=CollisionObject.REMOVE))]
            try:
                self.backend.apply(scene)
                self.backend.apply(PlanningScene(is_diff=True,world=PlanningSceneWorld(collision_objects=[original]),allowed_collision_matrix=matrix))
            except Exception as exc:
                from .backend import MotionError
                raise MotionError('service_or_action_timeout: payload preview restoration uncertain: '+str(exc)) from exc

    def attach(self, name, arm):
        piece = self.pieces[name]
        obj = self.piece_object(name,piece['symbol'],piece['local'])
        attached = AttachedCollisionObject(link_name=arm+'_tool0',object=obj,touch_links=[arm+'_tool0',arm+'_link4',arm+'_link5'])
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        self.backend.apply(scene)
        self.attached = name

    def detach(self, name, arm, local):
        self.pieces[name]['local'] = list(map(float,local))
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(link_name=arm+'_tool0',object=CollisionObject(id=name,operation=CollisionObject.REMOVE))]
        scene.world.collision_objects = [self.piece_object(name,self.pieces[name]['symbol'],local)]
        self.backend.apply(scene)
        self.attached = None

    def remove(self, name):
        self.backend.apply(PlanningScene(is_diff=True,world=PlanningSceneWorld(collision_objects=[CollisionObject(id=name,operation=CollisionObject.REMOVE)])))
        self.pieces.pop(name,None)
        self.ids.discard(name)

    def rename(self, old, new):
        piece = self.pieces[old]
        obj = self.piece_object(new,piece['symbol'],piece['local'])
        self.backend.apply(PlanningScene(is_diff=True,world=PlanningSceneWorld(collision_objects=[CollisionObject(id=old,operation=CollisionObject.REMOVE),obj])))
        self.pieces[new] = self.pieces.pop(old)
        self.ids.discard(old)
        self.ids.add(new)
