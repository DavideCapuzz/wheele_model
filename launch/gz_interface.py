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
        default_value='false',
        description='Use sim time if true')

    pkg_path = os.path.join(get_package_share_directory("wheele_model"))
    world_file = os.path.join(pkg_path,'worlds','world.sdf')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v4 ' + world_file], 'on_exit_shutdown': 'true'}.items(),
    )

    model_name = 'wheele'
    world_name = 'setup_room'
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', model_name,
                    '-x', '0.0',
                    '-z', '0.0',
                    '-Y', '0.0',
                    '-topic', '/robot_description'],
                 output='screen')
    
    # custom bridge node designed by me
    gz_interface = Node(
            package='gz_interface',
            namespace='gz_node',
            executable='gz_node',
            name='gz_node'
        )


    # Bridge to forward tf and joint states to ros2
    gz_topic = '/model/' + model_name
    # joint_state_gz_topic = '/world/map' + gz_topic + '/joint_state'
    joint_state_gz_topic = '/world/' + world_name + '/model/' + model_name + '/joint_state'
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
            #'/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/gz_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/gz_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/lidarASgz@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scangz@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/contact@ros_gz_interfaces/msg/Contact[gz.msgs.Contact',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/cmd_vel0@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # gz_topic +  '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            #'/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel1@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
            #('vehicle/vehicle/LidarCircular/vehicle/LidarCircularSensor', 'laser'),
            #('/model/base_link/odometry', '/odom')
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    static_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )


    # Launch!
    return LaunchDescription([
        # static_map_tf_node,
        gz_interface,
        bridge,
        gazebo,
        spawn,
        use_sim_time_launch_arg
    ])