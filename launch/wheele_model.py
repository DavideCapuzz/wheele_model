import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true')
    # Check if we're told to use sim time
    package_name = 'wheele_model'
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','robot','model.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', './vehicle_mini.rviz'],
        #arguments=['-d', os.path.join(pkg_ros_gz_sim_wheele,  'rviz', 'vehicle.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    static_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link/LidarCircular/LidarCircularSensor', 'laser']
    )

    static_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link/Camera/Camera', 'cam_frame']
    )

    # Launch!
    return LaunchDescription([
        node_robot_state_publisher,
        # rviz_launch_arg,
        use_sim_time_launch_arg,
        # rviz
        # static_laser_tf_node,
        # static_camera_tf_node
    ])