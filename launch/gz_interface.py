import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import xacro
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge
import xml.etree.ElementTree as ET
import yaml

def local(tag: str) -> str:
    # Strip namespace: "{ns}tag" -> "tag"
    return tag.split('}', 1)[1] if '}' in tag else tag

def print_xml(node: ET.Element, indent: int = 0):
    pad = "  " * indent
    attrs = " ".join(f'{k}="{v}"' for k, v in node.attrib.items())
    open_tag = f"<{local(node.tag)}{(' ' + attrs) if attrs else ''}>"
    text = (node.text or "").strip()

    if text:
        print(f"{pad}{open_tag} {text}")
    else:
        print(f"{pad}{open_tag}")

    for child in list(node):
        print_xml(child, indent + 1)

    print(f"{pad}</{local(node.tag)}>")

def modify_world_file(pkg_path, world_file, config_file):
    world_file_updated = "/tmp/temp.sdf"
    world_tree = ET.parse(world_file)
    world_root = world_tree.getroot()
    for key in config_file["environment_setup"].keys():
        model_sdf_path = os.path.join(pkg_path,"description" ,"worlds" ,"component" ,config_file["environment_setup"][key]["model_file"])
        model_tree = ET.parse(model_sdf_path)
        model_root = model_tree.getroot()
        pose=config_file["environment_setup"][key]["pose"]
        # print_xml(model_root)
        model_root.find('pose').text = pose
        for uri in model_root.iter("uri"):
            if uri.text:  # make sure it has text
                uri.text = os.path.join(pkg_path, uri.text)
        world_root.find('world').append(model_root)

    world_root.find('.//latitude_deg').text = str(config_file["world_setup"]["latitude_origin"])
    world_root.find('.//longitude_deg').text = str(config_file["world_setup"]["longitude_origin"])
    ET.indent(world_tree, space="  ", level=0)
    # Save merged SDF
    world_tree.write(world_file_updated, encoding='utf-8', xml_declaration=True)
    return world_file_updated

def generate_launch_description():
    set_render_engine = SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE', 'ogre')
    set_software_gl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    set_mesa_override = SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3')
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true')

    pkg_path = os.path.join(get_package_share_directory("wheele_model"))
    world_file = os.path.join(pkg_path,'worlds','world.sdf')

    with open(os.path.join(pkg_path, 'config', 'sim_setup.yaml'), 'r') as file:
        config_file = yaml.safe_load(file)
    world_file_updated = modify_world_file(pkg_path, world_file, config_file)

    # Include the Gazebo launch file, provided by the gazebo_ros package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_file_updated + ' -r -v 4 --render-engine ogre', 'on_exit_shutdown': 'true'}.items(),
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
            # joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            # link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
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
            '/model/wheele/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/wheele/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # gz_topic +  '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            #'/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel1@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        remappings=[
            ('/model/wheele/odometry', '/ground_truth/odom'),
            ('/model/wheele/pose', '/ground_truth/pose'),
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
        # arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'wheele']
    )


    # Launch!
    return LaunchDescription([
        set_render_engine,      # First
        set_software_gl,        # Second
        set_mesa_override,
        static_map_tf_node,
        gz_interface,
        bridge,
        gazebo,
        spawn,
        use_sim_time_launch_arg
    ])