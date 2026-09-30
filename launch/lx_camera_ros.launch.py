import os

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression


def launch_setup(context: LaunchContext):
    uav_name = LaunchConfiguration('uav_name')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    camera_name = LaunchConfiguration('camera_name')
    camera_params_file = LaunchConfiguration('camera_params_file')

    camera_node = Node(
        package='lx_camera_ros',
        executable='lx_camera_node',
        name=camera_name,
        namespace=uav_name,
        output='screen',
        emulate_tty=True,
        parameters=[
            ParameterFile(camera_params_file, allow_substs=True),
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='lx_camera',
        namespace=uav_name,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('lx_camera_ros'), 'rviz', 'lx_camera.rviz',
            ]),
            '-f', LaunchConfiguration('rviz_frame').perform(context).strip('/'),
        ],
        # The RViz config stores absolute topic names from the original launch.
        remappings=[
            ('/lx_camera_node/' + topic, topic)
            for topic in ('LxCamera_Rgb', 'LxCamera_Amp', 'LxCamera_Depth', 'LxCamera_Cloud')
        ],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    return [camera_node, rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'uav_name',
            default_value=os.getenv('UAV_NAME', 'uav1'),
            description='Top-level namespace.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=PythonExpression(['"', os.getenv('REAL_UAV', 'true'), '" == "false"']),
            description='Whether to use simulation time.',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='rgbd',
            description='Camera node name.',
        ),
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lx_camera_ros'), 'params', 'default.yaml',
            ]),
            description='Full path to the file with the camera parameters.',
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description='Whether to launch rviz2.',
        ),
        DeclareLaunchArgument(
            'rviz_frame',
            default_value=[LaunchConfiguration('uav_name'), '/',
                           LaunchConfiguration('camera_name'), '/link'],
            description='RViz fixed frame. Override when using a custom base_frame_id.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
