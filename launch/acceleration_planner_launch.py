from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    csv_path_arg = DeclareLaunchArgument(
        "csv_path",
        default_value="car_map_data.csv",
        description="Caminho para o CSV com os cones (colunas: x,y,color)",
    )

    return LaunchDescription(
        [
            csv_path_arg,
            Node(
                package="acceleration_planner",
                executable="cone_publisher",
                name="cone_publisher",
                parameters=[{"csv_path": LaunchConfiguration("csv_path")}],
                output="screen",
            ),
            Node(
                package="acceleration_planner",
                executable="planner_node",
                name="acceleration_planner",
                output="screen",
            ),
        ]
    )
