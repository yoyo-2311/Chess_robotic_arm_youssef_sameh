from setuptools import setup, find_packages
setup(name='chess_motion_interface', version='0.2.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/chess_motion_interface']),
                  ('share/chess_motion_interface', ['package.xml'])],
      tests_require=['pytest'], install_requires=['setuptools'], zip_safe=True, maintainer='Youssef Sameh',
      maintainer_email='yoyosameh11@gmail.com', description='Configurable chess scene and MoveIt operation adapter', license='Apache-2.0',
      entry_points={'console_scripts': [
          'motion_interface = chess_motion_interface.node:main',
          'mock_promotion_operator = chess_motion_interface.mocks:operator_main',
          'mock_vision_input = chess_motion_interface.mocks:input_main',
          'mock_gripper = chess_motion_interface.mocks:gripper_main',
          'mock_vision = chess_motion_interface.mocks:vision_main',
          'mock_engine = chess_motion_interface.mocks:engine_main',
          'square_target = chess_motion_interface.mocks:square_main']})
