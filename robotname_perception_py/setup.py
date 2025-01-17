from setuptools import find_packages, setup

package_name = 'robotname_perception_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itsrobocon3',
    maintainer_email='itsrobocon3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'silo_camera_node = robotname_perception_py.silo_camera:main',
            'yolov5_node = robotname_perception_py.yolov5_node:main',
            'depth_camera_node = robotname_perception_py.depth_camera_node:main',
            'silo_visualizer = robotname_perception_py.silo_visualizer:main',
            'global_pos = robotname_perception_py.global_position:main',
            'line_detection = robotname_perception_py.line_detection:main',
            'silo_depth_camera_node = robotname_perception_py.silo_depth_camera:main'
        ],
    },
)
