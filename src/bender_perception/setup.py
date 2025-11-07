from setuptools import find_packages, setup

package_name = 'bender_perception'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = bender_perception.camera_node:main',
            'yolo_node = bender_perception.yolo_node:main',
            'grasping_node = bender_perception.grasping_node:main',
            'workspace_left = bender_perception.workspace_left:main',
            'workspace_right = bender_perception.workspace_right:main',
            'head_tracker = bender_perception.head_tracker:main',
            'mecanum_pid = bender_perception.mecanum_pid:main'
        ],
    },

)
