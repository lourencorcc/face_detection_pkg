from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    params_file_dec = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('face_detection_pkg'), 'config', 'follow_human_params.yaml'),
        description='Full path to params file for the follow_human node.')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_dec = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    person_point_topic = LaunchConfiguration('person_point_topic')
    person_point_topic_dec = DeclareLaunchArgument(
        'person_point_topic',
        default_value='/person_point',
        description='The topic for the person point messages.')

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_topic_dec = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='The topic for the velocity commands.')

    follow_human_node = Node(
        package='face_detection_pkg',
        executable='follow_human',
        name='follow_human',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/person_point', person_point_topic), ('/cmd_vel', cmd_vel_topic)]
    )

    return LaunchDescription([
        params_file_dec,
        use_sim_time_dec,
        person_point_topic_dec,
        cmd_vel_topic_dec,
        follow_human_node
    ])
