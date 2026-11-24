"""Declared entry points and capabilities; no shell interpolation."""
from dataclasses import dataclass

START_FEN='rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1'
SCENARIOS={
    'Standard game':START_FEN,
    'After c4 e5: g1f3':'rnbqkbnr/pppp1ppp/8/4p3/2P5/8/PP1PPPPP/RNBQKBNR w KQkq - 0 2',
    'Capture e4d5':'rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2',
    'Castling e1g1':'r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1',
    'En passant e5d6':'4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 1',
    'Promotion a7a8q':'4k3/P7/8/8/8/8/8/4K3 w - - 0 1',
}
@dataclass(frozen=True)
class Mode:
    key:str
    title:str
    package:str
    launch:str
    explanation:str

MODES=[
    Mode('stockfish','Human vs Stockfish — simulated match','dual_scara_desktop','stockfish_match.launch.py','robot1 = White, robot2 = Black. Mock controllers/gripper/vision; confirmed game state only.'),
    Mode('mock','Full mock chess — repeatable scenarios','dual_chess_moveit_config','simulation_pipeline.launch.py','Deterministic mock engine. Submit either side manually; promotion needs explicit operator confirmation.'),
    Mode('moveit','MoveIt / square tests — mock controllers','dual_chess_moveit_config','demo.launch.py','Plan previews do not move joints. Execute sends a real ROS trajectory action to mock controllers.'),
    Mode('manual','Manual joints — visualization only','dual_scara_desktop','manual.launch.py','Desktop owns the only joint-state publisher. Sliders visualize joints; no collision checking or motors.'),
    Mode('gazebo','Gazebo — physics and direct joint controls','dual_chess_gazebo','gazebo.launch.py','Existing Gazebo Harmonic simulation; direct position topics, not MoveIt trajectory execution. CAD/floor contact remains provisional.'),
    Mode('logic','Chess logic only — external interfaces','dual_chess_bringup','chess_logic_only.launch.py','Existing brain, no robot/vision/readiness source. Requires external sync and safety; no automatic coordinator.'),
]

def launch_argv(mode,geometry,rviz,stockfish,scenario,headless_gazebo=False):
    args=['ros2','launch',mode.package,mode.launch]
    if mode.key!='logic':args+=['geometry_file:='+str(geometry)]
    if mode.key in ['stockfish','mock','moveit','manual']:args+=['rviz:='+str(bool(rviz)).lower()]
    if mode.key in ['stockfish','logic']:args+=['stockfish_path:='+str(stockfish)]
    if mode.key in ['stockfish','mock','moveit']:args+=['motion_start_delay:=2.0']
    if mode.key=='moveit':args+=['publish_mock_readiness:=true']
    if mode.key=='mock':args+=['initial_fen:='+SCENARIOS[scenario]]
    if mode.key=='logic':args+=['auto_coordinator_enabled:=false','executor_enabled:=false']
    if mode.key=='gazebo':args+=['gz_args:=-r -s' if headless_gazebo else 'gz_args:=-r']
    return args
