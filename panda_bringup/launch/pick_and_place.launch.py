import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    target_color = LaunchConfiguration("target_color")

    target_color_arg = DeclareLaunchArgument(
        "target_color",
        default_value="B",
        description="Color to pick: R, G, or B",
    )

    # ------------------- Gazebo -------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    # ------------------- Controllers -------------------
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- MoveIt -------------------
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_moveit"),
                "launch",
                "movit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- Vision Node -------------------
    vision_node = Node(
        package="panda_vision",
        executable="color_detector",
        name="color_detector",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ------------------- MoveIt Color Picker Node -------------------
    color_picker_node = Node(
        package="pymoveit2",
        executable="pick_and_place.py",
        name="pick_and_place",
        output="screen",
        parameters=[
            {"target_color": target_color},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        target_color_arg,
        gazebo,
        controller,
        moveit,
        vision_node,
        TimerAction(period=10.0, actions=[color_picker_node]),
    ])
