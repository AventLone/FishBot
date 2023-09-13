import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package="fishbot_description").find(
        "fishbot_description"
    )

    rviz_config = LaunchConfiguration(
        "rviz_config", default=os.path.join(pkg_share, "config/fishbot.rviz")
    )

    # ==== 定义launhc文件 ==== #
    launch_showURDF = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, "/launch", "/rviz2.launch.py"]),
        launch_arguments={"rviz_config": rviz_config}.items()
    )

    ld = LaunchDescription()
    ld.add_action(launch_showURDF)

    return ld
