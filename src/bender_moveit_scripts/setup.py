from setuptools import setup

package_name = "bender_moveit_scripts"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mattias Prieto",
    maintainer_email="tu_email@example.com",
    description="Scripts personalizados para controlar MoveIt con ROS 2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "moveit_target_pose = bender_moveit_scripts.moveit_target_pose:main",
	    "move_left_arm = bender_moveit_scripts.move_left_arm:main",
	    "move_groups_control = bender_moveit_scripts.move_groups_control:main",
            "publish_target_pose = bender_moveit_scripts.publish_target_pose:main",
        ],
    },
)
