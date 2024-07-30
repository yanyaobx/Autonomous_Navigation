from setuptools import find_packages, setup

package_name = 'turtlebot_rrt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rcpsl',
    maintainer_email='rcpsl@todo.todo',
    description='RRT Node for TurtleBot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_node = turtlebot_rrt.rrt_node:main',
            'motion_planner = turtlebot_rrt.motion_planner:main',
            'map_reader = turtlebot_rrt.map_reader_template:main',

        ],
    },
)

