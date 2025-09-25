import os
from glob import glob
from setuptools import setup

package_name = "bender_description"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Incluir archivos launch
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        # Incluir archivos urdf
        (os.path.join('share', package_name, 'urdf'), 
            glob('urdf/*.xacro')),
        # Incluir archivos rviz
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')),
        # Incluir meshes
        (os.path.join('share', package_name, 'meshes'), 
            glob('meshes/*.stl')),
        # Incluir config
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    package_dir={'': '.'},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tu_nombre",
    maintainer_email="tu_email@example.com",
    description="Scripts personalizados para controlar MoveIt con ROS 2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "moveit_target_pose = bender_description.moveit_target_pose:main",
            'cylinder_marker = bender_description.cylinder_marker:main',
            'global_motion_planner = bender_description.global_motion_planner:main',
        ],
    },
)
