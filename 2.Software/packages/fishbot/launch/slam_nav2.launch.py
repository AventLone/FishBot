import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ==== 定义功能包路径 ==== #
    fishbot_dir = FindPackageShare(package="fishbot").find("fishbot")
    fishbot_description_dir = FindPackageShare(package="fishbot_description").find(
        "fishbot_description"
    )

    rviz_config = LaunchConfiguration(
        "rviz_config",
        default=os.path.join(fishbot_description_dir, "config/nav2.rviz"),
    )

    nav2_bringup_dir = FindPackageShare(package="nav2_bringup").find("nav2_bringup")
    nav2_param_path = os.path.join(fishbot_dir, "param", "fishbot.yaml")

    # ==== 定义 Cartographer 参数 ==== #
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

    # ==== 定义Nav2参数 ==== #
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    nav2_param = LaunchConfiguration("params_file", default=nav2_param_path)

    # ==== 定义rviz2参数 ==== #
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
        arguments=["0.0", "0.0", "0.0", "0", "0", "0", "laser_frame", "base_footprint"],
    )
    microros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=["udp4", "--port", "8888"],
    )

    # ==== 定义launhc文件 ==== #
    launch_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, "/launch", "/bringup_launch.py"]
        ),
        launch_arguments={
            "map": "",
            "use_sim_time": use_sim_time,
            "params_file": nav2_param,
        }.items(),
    )
    launch_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_description_dir, "/launch", "/rviz2.launch.py"]
        ),
        launch_arguments={"rviz_config": rviz_config}.items(),
    )

    # ==== 启动节点 ==== #
    ld = LaunchDescription()
    ld.add_action(microros_agent_node)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(tf2_node)
    ld.add_action(launch_nav2)
    ld.add_action(launch_rviz2)

    return ld
