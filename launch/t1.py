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

import xacro


def generate_launch_description():

    gz_interface = Node(
            package='gz_interface',
            namespace='gz_node',
            executable='gz_node',
            name='gz_node'
        )


    # Bridge to forward tf and joint states to ros2
    gz_topic = '/model/base_link'
    # joint_state_gz_topic = '/world/map' + gz_topic + '/joint_state'
    joint_state_gz_topic = '/world/odom/model/base_link/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry (Gazebo -> ROS2)
            gz_topic + '/cmd_vel0@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # gz_topic +  '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            gz_topic + '/cmd_vel1@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/gz_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/gz_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/lidarAS@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
            #('vehicle/vehicle/LidarCircular/vehicle/LidarCircularSensor', 'laser'),
            ('/model/base_link/odometry', '/odom')
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        gz_interface,
        bridge
    ])