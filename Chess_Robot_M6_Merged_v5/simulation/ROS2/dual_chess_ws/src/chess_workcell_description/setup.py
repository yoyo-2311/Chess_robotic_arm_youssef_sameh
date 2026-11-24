from setuptools import setup,find_packages
from pathlib import Path
assets=[]
for folder in ['meshes','urdf','launch','reference','config']:
 for directory in [Path(folder),*Path(folder).rglob('*')]:
  if directory.is_dir() and '__pycache__' not in directory.parts:
   files=[str(p) for p in directory.iterdir() if p.is_file()]
   if files:assets.append(('share/chess_workcell_description/'+str(directory),files))
setup(name='chess_workcell_description',version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/chess_workcell_description']),('share/chess_workcell_description',['package.xml','README.md']),*assets],
 install_requires=['setuptools'],tests_require=['pytest'],maintainer='Youssef Sameh',maintainer_email='yoyosameh11@gmail.com',
 description='Immutable CAD environment assets and frame adapters',license='BSD-3-Clause')
