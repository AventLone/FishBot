import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==== 定义功能包路径 ==== #
    fishbot_dir = FindPackageShare(package="fishbot").find("fishbot")
    fishbot_description_dir = FindPackageShare(package="fishbot_description").find(
        "fishbot_description"
    )
    # pkg_share = FindPackageShare(package="fishbot_description").find(
    #     "fishbot_description"
    # )

    rviz_config = LaunchConfiguration(
        "rviz_config",
        default=os.path.join(fishbot_description_dir, "config/cartographer.rviz"),
    )

    # ==== 定义参数 ==== #
    resolution = LaunchConfiguration("resolution", default="0.05")  # 地图的分辨率
    publish_period_sec = LaunchConfiguration(
        "publish_period_sec", default="1.0"
    )  # 地图的发布周期
    configuration_directory = LaunchConfiguration(
        "configuration_directory", default=os.path.join(fishbot_dir, "config")
    )  # 配置文件夹路径
    configuration_basename = LaunchConfiguration(
        "configuration_basename", default="fishbot_laser_2d.lua"
    )  # 配置文件

    rviz_config = LaunchConfiguration(
        "rviz_config",
        default=os.path.join(fishbot_description_dir, "config/cartographer.rviz"),
    )

    # ==== cartographer_node节点 ==== #
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        arguments=[
            "-configuration_directory",
            configuration_directory,
            "-configuration_basename",
            configuration_basename,
        ],
    )
    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        arguments=[
            "-resolution",
            resolution,
            "-publish_period_sec",
            publish_period_sec,
        ],
    )
    tf2_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser",
        arguments=["0.0", "0.0", "0.0", "0", "0", "0", "laser_frame", "base_footprint"]
    )
    # tf2_node_2 = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_tf_pub_laser",
    #     arguments=["0.0", "0.0", "0.0", "0", "0", "0", "odom", "base_footprint"],
    # )
    # tf2_node_3 = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_tf_pub_laser",
    #     arguments=["0.0", "0.0", "0.0", "0", "0", "0", "map", "odom"],
    # )
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
        ),
        launch_arguments={"rviz_config": rviz_config}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(tf2_node)
    # ld.add_action(node_fishbot_bringup)
    # ld.add_action(tf2_node_2)
    # ld.add_action(tf2_node_3)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(launch_showURDF)

    return ld
