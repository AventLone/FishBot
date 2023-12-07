from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==== 定义功能包路径 ==== #
    fishbot_description_dir = FindPackageShare(package="fishbot_description").find(
        "fishbot_description"
    )

    # ==== 定义节点 ==== #
    node_fishbot_bringup = Node(
        package="fishbot",
        executable="fishbot_bringup",
        # name="fishbot_bringup",
        # output="screen",
    )

    # ==== 定义launhc文件 ==== #
    launch_showURDF = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_description_dir, "/launch", "/rviz2.launch.py"]
        )
    )

    ld = LaunchDescription()
    ld.add_action(node_fishbot_bringup)
    ld.add_action(launch_showURDF)

    return ld
