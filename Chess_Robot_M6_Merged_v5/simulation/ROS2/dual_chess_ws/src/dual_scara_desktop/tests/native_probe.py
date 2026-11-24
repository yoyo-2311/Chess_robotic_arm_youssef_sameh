"""Opt-in native X11 Qt smoke test. Opens a desktop window, captures it, then closes cleanly."""
import argparse,ctypes,json,os,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--workspace',required=True);p.add_argument('--domain',type=int,default=215);p.add_argument('--mode',choices=['manual','moveit','gazebo']);p.add_argument('--geometry',type=Path);args=p.parse_args()
for lib in Path(args.workspace,'.qt-deps').glob('os/usr/lib/*/libxcb-cursor.so.0'):ctypes.CDLL(str(lib),mode=ctypes.RTLD_GLOBAL)
for key in ['QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH']:os.environ.pop(key,None)
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from ament_index_python.packages import get_package_share_directory
from dual_scara_desktop.window import MainWindow
from dual_scara_desktop.controller import DesktopController
from dual_scara_desktop.modes import MODES
app=QApplication([]);profile=Path(get_package_share_directory('dual_chess_description'))/'config/chess_geometry.yaml'
if args.geometry:profile=args.geometry.resolve()
w=MainWindow(profile,args.domain);controller=DesktopController(w,args.workspace,args.domain,headless_gazebo=True);w.show()
if args.mode:
 w.mode.setCurrentIndex(next(i for i,m in enumerate(MODES) if m.key==args.mode));controller.start_selected()
def finish():
 out=Path(args.workspace,'.desktop-state');w.grab().save(str(out/'qt_native.png'))
 (out/'qt_native.json').write_text(json.dumps({'qt_platform':app.platformName(),'window_visible':w.isVisible(),'desktop_only':not bool(args.mode),'mode':args.mode,'ready':controller.ready,'component_failed':controller.component_failed,'software_stop_checked':controller.software_stopped},indent=2))
 controller.close()
started=time.monotonic();done=False;passed=not bool(args.mode)
def check():
 global done,passed
 if done:return
 if not args.mode and time.monotonic()-started>2 or args.mode and controller.ready:
  passed=True;done=True
  if args.mode=='moveit':
   controller.motion_stop();assert controller.supervisor.running and not controller.ready
  finish()
 elif time.monotonic()-started>100:
  done=True;print(w.status.text(),w.log.toPlainText()[-4000:],flush=True);controller.close()
timer=QTimer();timer.setInterval(100);timer.timeout.connect(check);timer.start()
app.exec()
assert not controller.supervisor.alive() and not controller.bridge.alive()
assert passed,'Backend readiness failed'
print('Qt backend smoke and clean close passed:',args.mode)
