from setuptools import setup, find_packages
from glob import glob
setup(name='dual_scara_desktop',version='0.1.0',packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/dual_scara_desktop']),
                  ('share/dual_scara_desktop',['package.xml']),('share/dual_scara_desktop/launch',glob('launch/*.launch.py'))],
      tests_require=['pytest'],install_requires=['setuptools'],zip_safe=True,maintainer='Youssef Sameh',maintainer_email='yoyosameh11@gmail.com',
      description='Local Qt desktop for dual SCARA ROS modes and confirmed Stockfish matches',license='Apache-2.0',
      entry_points={'console_scripts':['desktop = dual_scara_desktop.app:main']})
