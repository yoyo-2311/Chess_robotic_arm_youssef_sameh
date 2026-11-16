#!/usr/bin/env python3
"""Static consistency audit of the workspace (works for ament_cmake AND ament_python packages).

Checks (no ROS runtime needed; `xacro` must be installed for the URDF part):
  * package.xml: well-formed, <name> == folder name;
  * ament_cmake: install(DIRECTORY/PROGRAMS) targets exist;
  * ament_python: resource marker, setup.py data_files and console-script modules exist;
  * launch files: every package they use is a declared exec_depend; every urdf/rviz/world/config/launch
    file they reference exists and is installed by its package;
  * all .py files compile;
  * every top-level *.urdf.xacro expands (several argument variants), is a valid tree
    (tools via description/scripts/check_urdf_structure.py) and every package:// mesh exists and is installed.
Usage (workspace root, ROS sourced):  python3 tools/static_audit.py      Exit code 0 = no problems.
"""
import glob, os, re, subprocess, sys, xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'src')
problems = []
EXTERNAL_LAUNCH_FILES = {'gz_sim.launch.py'}   # shipped by ros_gz_sim, not by this workspace
def bad(m): problems.append(m); print('  [FAIL]', m)
def ok(m): print('  [ok]  ', m)

pkgs = sorted(d for d in os.listdir(SRC) if os.path.isfile(os.path.join(SRC, d, 'package.xml')))
print('Packages:', pkgs)
info = {}
for p in pkgs:
    base = os.path.join(SRC, p)
    root = ET.parse(os.path.join(base, 'package.xml')).getroot()
    btype = (root.findtext('export/build_type') or '').strip()
    deps = {e.text.strip() for e in root if e.tag.endswith('depend') and e.text}
    installed = []   # directories (cmake) or launch files (python data_files) installed into share/<pkg>
    if root.findtext('name') != p: bad(f'{p}: package.xml name {root.findtext("name")!r} != folder')
    if btype == 'ament_cmake':
        cm = open(os.path.join(base, 'CMakeLists.txt')).read()
        for m in re.finditer(r'install\(\s*DIRECTORY\s+([^)]*?)DESTINATION', cm, re.S): installed += m.group(1).split()
        for d in installed:
            if not os.path.isdir(os.path.join(base, d)): bad(f'{p}: install(DIRECTORY {d}) but folder missing')
        for m in re.finditer(r'PROGRAMS\s+(\S+)', cm):
            if not os.path.isfile(os.path.join(base, m.group(1))): bad(f'{p}: install(PROGRAMS {m.group(1)}) missing')
    elif btype == 'ament_python':
        sp = os.path.join(base, 'setup.py'); s = open(sp).read()
        if not os.path.isfile(os.path.join(base, 'resource', p)): bad(f'{p}: resource/{p} marker missing')
        for f in re.findall(r"\[\s*'([^']+\.(?:py|xml|yaml|rviz|xacro|sdf|launch|json|csv))'\s*\]", s) + re.findall(r"\['(launch/[^']+)'\]", s):
            if not f.startswith('resource/') and not os.path.isfile(os.path.join(base, f)): bad(f'{p}: setup.py data file missing: {f}')
            installed.append(f)
        for mod in re.findall(r"=\s*([\w\.]+):\w+", s):
            if not os.path.isfile(os.path.join(base, mod.replace('.', '/') + '.py')): bad(f'{p}: console_script module {mod} missing')
    else: bad(f'{p}: unknown build_type {btype!r}')
    info[p] = dict(base=base, deps=deps, installed=installed, type=btype)
    ok(f'{p} ({btype}): manifest and install targets consistent so far')

pyfiles = [f for p in pkgs for f in glob.glob(os.path.join(SRC, p, '**', '*.py'), recursive=True) if '_vendor' not in f]
for f in pyfiles:
    try: compile(open(f, encoding='utf-8').read(), f, 'exec')
    except SyntaxError as e: bad(f'syntax error {os.path.relpath(f, ROOT)}: {e}')
ok(f'{len(pyfiles)} python files compile')

def installed_by(sub, fn):
    for q, i in info.items():
        if os.path.isfile(os.path.join(i['base'], sub, fn)) and (sub in i['installed'] or f'{sub}/{fn}' in i['installed']): return q
    return None

for p in pkgs:
    ld = os.path.join(info[p]['base'], 'launch')
    for f in sorted(glob.glob(os.path.join(ld, '*.py'))) if os.path.isdir(ld) else []:
        src = open(f).read(); rel = f'{p}/launch/{os.path.basename(f)}'
        need = set(re.findall(r"package='([^']+)'", src)) | set(re.findall(r"(?:get_package_share_directory|FindPackageShare)\('([^']+)'\)", src))
        if 'ament_index_python' in src: need.add('ament_index_python')
        if 'launch_ros' in src: need.add('launch_ros')
        miss = sorted(n for n in need if n not in info[p]['deps'] and n != p)
        (bad if miss else ok)(f'{rel}: undeclared exec_depend {miss}' if miss else f'{rel}: dependencies declared')
        for sub, fn in re.findall(r"'(urdf|rviz|worlds|config|launch)',\s*'([^']+\.(?:xacro|rviz|sdf|yaml|py))'", src):
            if fn in EXTERNAL_LAUNCH_FILES:
                ok(f'{rel}: {sub}/{fn} -> external ros_gz_sim file, skipped'); continue
            q = installed_by(sub, fn)
            (ok if q else bad)(f'{rel}: {sub}/{fn} -> ' + (f'installed by {q}' if q else 'NOT FOUND / NOT INSTALLED'))

desc = next((p for p in pkgs if os.path.isdir(os.path.join(SRC, p, 'urdf'))), None)
if desc:
    D = info[desc]['base']; chk = os.path.join(D, 'scripts', 'check_urdf_structure.py')
    tops = sorted(glob.glob(os.path.join(D, 'urdf', '*.urdf.xacro')))
    for t in tops:
        for args in ('', 'sim_gazebo:=true', 'robot2_enabled:=false', 'robot1_enabled:=false robot2_enabled:=false'):
            r = subprocess.run(['xacro', t] + args.split(), capture_output=True, text=True)
            nm = os.path.basename(t)
            if r.returncode: bad(f'xacro {nm} {args}: {r.stderr.strip()[:200]}'); continue
            c = subprocess.run([sys.executable, chk], input=r.stdout, capture_output=True, text=True)
            (ok if c.returncode == 0 else bad)(f'xacro {nm} {args or "(default)"}: expands; ' + c.stdout.strip().splitlines()[-2].strip() if c.returncode == 0 else f'xacro {nm} {args}: structure INVALID')
            for uri in set(re.findall(r'filename="package://' + desc + r'/([^"]+)"', r.stdout)):
                if not os.path.isfile(os.path.join(D, uri)): bad(f'mesh missing: {uri}')
                elif uri.split('/')[0] not in info[desc]['installed']: bad(f'mesh dir not installed: {uri}')
print('\nRESULT:', 'OK - no problems found' if not problems else f'{len(problems)} problem(s)')
sys.exit(1 if problems else 0)
