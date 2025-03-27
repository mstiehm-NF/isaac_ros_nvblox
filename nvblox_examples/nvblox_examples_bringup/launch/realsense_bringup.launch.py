import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def load_camera_pose(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def generate_launch_description():
    bringup_dir = get_package_share_directory('nvblox_examples_bringup')

    # Load camera pose from YAML file
    camera_pose_file = '/usr/config/camera_pose.yaml'
    camera_pose = load_camera_pose(camera_pose_file)

    x, y, z = camera_pose['translation']['x'], camera_pose['translation']['y'], camera_pose['translation']['z']
    roll, pitch, yaw = camera_pose['rotation']['roll'], camera_pose['rotation']['pitch'], camera_pose['rotation']['yaw']

    # Launch Arguments
    launch_args = [
        DeclareLaunchArgument(
        'run_rviz', default_value='False',
        description='Whether to start RVIZ'),
        DeclareLaunchArgument(
        'from_bag', default_value='False',
        description='Whether to run from a bag or live realsense data'),
        DeclareLaunchArgument(
        'bag_path', default_value='rosbag2*',
        description='Path of the bag (only used if from_bag == True)'),
        DeclareLaunchArgument(
        'flatten_odometry_to_2d', default_value='False',
        description='Whether to flatten the odometry to 2D (camera only moving on XY-plane).'),
        DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace for all nodes and topics'),
        DeclareLaunchArgument(
        'reset_emitter_on_off', default_value='True',
        description='Set the emitter on/off parameter'),
        DeclareLaunchArgument(
        'camera_name', default_value=[LaunchConfiguration('namespace'), '/camera'])
    ]

    global_frame = LaunchConfiguration('global_frame', default='odom')
    namespace = LaunchConfiguration('namespace')

    # Create a shared container to hold composable nodes 
    # for speed ups through intra process communication.
    shared_container_name = "shared_nvblox_container"
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen')
    
    # Static transform publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        ## Arguments: x, y, z, r, p, y, frame_id, child_frame_id
        arguments=[str(x), str(y), str(z), str(roll), str(pitch), str(yaw), 'base_link', 'camera_link'],
        output='screen')    

    # Realsense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'sensors', 'realsense.launch.py')]),
        launch_arguments={
            'namespace': namespace,
            'attach_to_shared_component_container': 'True',
            'component_container_name': shared_container_name}.items(),
        condition=UnlessCondition(LaunchConfiguration('from_bag')))
    
    # Realsense param set
    camera_name = LaunchConfiguration('camera_name')
    reset_rs_param = ExecuteProcess(
        cmd = [
            'bash', '-c',
            'for i in 1 2; do '
            '  while ! ros2 param set "$1" depth_module.emitter_on_off true | grep -q "Set parameter successful"; do '
            '    echo "Attempting to set depth_module.emitter_on_off true on camera: $1 (iteration $i)"; '
            '    echo "Parameter not set yet. Retrying in 5 seconds..."; '
            '    sleep 5; '
            '  done; '
            '  echo "Parameter set successfully for $1 (iteration $i)."; '
            'done',
            '_',  # dummy $0 so that $1 becomes the camera_name
            camera_name
        ],
        shell=False,
        output='screen',
        condition=IfCondition(LaunchConfiguration('reset_emitter_on_off'))
    )

    # Vslam
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'perception', 'vslam.launch.py')]),
        launch_arguments={'namespace': namespace,
                          'output_odom_frame_name': global_frame, 
                          'setup_for_realsense': 'True',
                          'run_odometry_flattening': LaunchConfiguration('flatten_odometry_to_2d'),
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items())

    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'nvblox', 'nvblox.launch.py')]),
        launch_arguments={'namespace': namespace,
                          'global_frame': global_frame,
                          'setup_for_realsense': 'True',
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items())

    # Ros2 bag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
        shell=True, output='screen',
        condition=IfCondition(LaunchConfiguration('from_bag')))

    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'rviz', 'rviz.launch.py')]),
        launch_arguments={'namespace': namespace,
                          'config_name': 'realsense_example.rviz',
                          'global_frame': global_frame}.items(),
        condition=IfCondition(LaunchConfiguration('run_rviz')))

    return LaunchDescription(launch_args + [
        shared_container,
        realsense_launch,
        vslam_launch,
        nvblox_launch,
        bag_play,
        rviz_launch,
        static_tf,
        reset_rs_param])