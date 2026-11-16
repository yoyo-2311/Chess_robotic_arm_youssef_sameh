from setuptools import setup

package_name = 'chess_robot_executor'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/executor.launch.py']),
        ('share/' + package_name + '/config', ['config/executor_params.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'], zip_safe=True,
    maintainer='M6 Simulation / Control',
    maintainer_email='noreply@example.com',
    description='Chess robot executor: move plans to SCARA trajectories (dry-run by default)',
    license='MIT',
    entry_points={'console_scripts': [
        'robot_executor = chess_robot_executor.node:main',
    ]},
)
