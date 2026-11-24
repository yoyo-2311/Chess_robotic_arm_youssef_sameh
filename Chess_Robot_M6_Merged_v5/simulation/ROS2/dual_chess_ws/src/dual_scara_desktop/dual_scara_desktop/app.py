"""Documented local entry point: ros2 run dual_scara_desktop desktop."""
import argparse
import os
from pathlib import Path
import signal
import sys


def find_workspace():
    requested=os.environ.get('DUAL_SCARA_WORKSPACE')
    if requested:return Path(requested).resolve()
    for location in [Path.cwd(),Path(__file__).resolve()]:
        for parent in [location,*location.parents]:
            if (parent/'src/dual_chess_description').is_dir():return parent
    raise RuntimeError('Cannot locate workspace; set DUAL_SCARA_WORKSPACE or pass --workspace.')


def main(argv=None):
    parser=argparse.ArgumentParser(description='Local Dual SCARA Qt console (ROS 2 environment must be sourced).')
    parser.add_argument('--workspace');parser.add_argument('--ros-domain-id',type=int,default=211)
    parser.add_argument('--headless-gazebo',action='store_true')
    parser.add_argument('--geometry-file',type=Path,help='Geometry YAML profile; can also be selected in the window')
    args=parser.parse_args(argv)
    workspace=Path(args.workspace).resolve() if args.workspace else find_workspace()
    if not 0<=args.ros_domain_id<=232:parser.error('ROS domain must be between 0 and 232')
    local_deps=workspace/'.qt-deps'
    if local_deps.is_dir():sys.path.insert(0,str(local_deps))
    import ctypes
    for library in local_deps.glob('os/usr/lib/*/libxcb-cursor.so.0'):
        ctypes.CDLL(str(library),mode=ctypes.RTLD_GLOBAL)
    for name in ['QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH']:os.environ.pop(name,None)
    try:from PySide6.QtWidgets import QApplication
    except ImportError:
        parser.exit(2,'PySide6 is missing. See docs/10_desktop_application.md or run scripts/install_desktop_dependencies.sh.\n')
    from ament_index_python.packages import get_package_share_directory
    from .window import MainWindow
    from .controller import DesktopController
    app=QApplication(sys.argv[:1]);app.setApplicationName('Dual SCARA Chess Robot')
    profile=args.geometry_file.expanduser().resolve() if args.geometry_file else Path(get_package_share_directory('dual_chess_description'))/'config/chess_geometry.yaml'
    window=MainWindow(profile,args.ros_domain_id);controller=DesktopController(window,workspace,args.ros_domain_id,args.headless_gazebo)
    signal.signal(signal.SIGINT,lambda *_:controller.close())
    window.show()
    result=app.exec()
    return result


if __name__=='__main__':raise SystemExit(main())
