# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import yaml

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression, OrSubstitution

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu
from launch_ros.actions import PushRosNamespace


from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera, NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

def load_camera_pose(file_path):
    try:
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data
    except Exception as e:
        print(f"Error loading camera pose from {file_path}: {e}")
        # Return default pose if file loading fails
        return {
            'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'rotation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        }

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'namespace', '', description='Namespace for all nodes and topics.', cli=True)
    args.add_arg(
        'rosbag', 'None', description='Path to rosbag (running on sensor if not set).', cli=True)
    args.add_arg('rosbag_args', '',
                 description='Additional args for ros2 bag play.', cli=True)
    args.add_arg('log_level', 'info', choices=[
                 'debug', 'info', 'warn'], cli=True)
    args.add_arg('num_cameras', 1,
                 description='How many cameras to use.', cli=True)
    args.add_arg('camera_serial_numbers', '',
                 description='List of the serial no of the extra cameras. (comma separated)',
                 cli=True)
    args.add_arg(
        'multicam_urdf_path',
        lu.get_path('nvblox_examples_bringup',
                    'config/urdf/4_realsense_carter_example_calibration.urdf.xacro'),
        description='Path to a URDF file describing the camera rig extrinsics. Only used in multicam.',
        cli=True)
    args.add_arg(
        'mode',
        default=NvbloxMode.static,
        choices=NvbloxMode.names(),
        description='The nvblox mode.',
        cli=True)
    args.add_arg(
        'people_segmentation',
        default=NvbloxPeopleSegmentation.peoplesemsegnet_vanilla,
        choices=[
            str(NvbloxPeopleSegmentation.peoplesemsegnet_vanilla),
            str(NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg)
        ],
        description='The  model type of PeopleSemSegNet (only used when mode:=people_segmentation).',
        cli=True)
    args.add_arg(
        'attach_to_container',
        'False',
        description='Add components to an existing component container.',
        cli=True)
    args.add_arg(
        'container_name',
        NVBLOX_CONTAINER_NAME,
        description='Name of the component container.')
    args.add_arg(
        'run_realsense',
        'False',
        description='Launch Realsense drivers')
    args.add_arg(
        'use_foxglove_whitelist',
        True,
        description='Disable visualization of bandwidth-heavy topics',
        cli=True)
    args.add_arg(
        'camera_pose_file',
        '/usr/config/camera_pose.yaml',
        description='Path to the camera_pose.yaml file.',
        cli=True)

    actions = args.get_launch_actions()

    # Load camera pose from YAML file
    # This needs to be done within an OpaqueFunction or similar to use LaunchConfiguration value
    # For simplicity, we'll create the node with substitutions.
    # If the file path itself needs to be dynamic via LaunchArg, it's more complex.
    # Assuming camera_pose_file_arg is resolved correctly by lu.static_transform if it could take it.
    # Since lu.static_transform expects concrete values for translation/rotation,
    # we define the node using an OpaqueFunction to load the YAML.

    def static_tf_publisher_setup(context):
        camera_pose_file_path = args.camera_pose_file.perform(context)
        camera_pose = load_camera_pose(camera_pose_file_path)
        x = str(camera_pose['translation']['x'])
        y = str(camera_pose['translation']['y'])
        z = str(camera_pose['translation']['z'])
        roll = str(camera_pose['rotation']['roll'])
        pitch = str(camera_pose['rotation']['pitch'])
        yaw = str(camera_pose['rotation']['yaw'])
        
        current_namespace = args.namespace.perform(context)

        static_tf_node = lu.static_transform(
            parent='base_link',
            child='camera0_link',
            translation=[x, y, z],
            orientation_rpy=[roll, pitch, yaw],
            namespace=current_namespace # Node's namespace, using the performed string
        )
        return [static_tf_node]

    actions.append(OpaqueFunction(function=static_tf_publisher_setup))

    # Globally set use_sim_time if we're running from bag or sim
    actions.append(
        SetParameter('use_sim_time', True, condition=IfCondition(lu.is_valid(args.rosbag))))

    # Single or Multi-realsense
    is_multi_cam = UnlessCondition(lu.is_equal(args.num_cameras, '1'))
    camera_mode = lu.if_else_substitution(
        lu.is_equal(args.num_cameras, '1'),
        str(NvbloxCamera.realsense),
        str(NvbloxCamera.multi_realsense)
    )
    # Only up to 4 Realsenses is supported.
    actions.append(
        lu.assert_condition(
            'Up to 4 cameras have been tested! num_cameras must be less than 5.',
            IfCondition(PythonExpression(['int("', args.num_cameras, '") > 4']))),
    )

    run_rs_driver = UnlessCondition(
        OrSubstitution(lu.is_valid(args.rosbag), lu.is_false(args.run_realsense)))
    # Realsense
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/sensors/realsense.launch.py',
            launch_arguments={
                'namespace': args.namespace,
                'container_name': args.container_name, # Consider namespacing this if attach_to_container is true and target is namespaced
                'camera_serial_numbers': args.camera_serial_numbers,
                'num_cameras': args.num_cameras,
            },
            condition=run_rs_driver))

    # Visual SLAM
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/vslam.launch.py',
            launch_arguments={
                'namespace': args.namespace,
                'container_name': args.container_name,
                'camera': camera_mode,
            },
            # Delay for 1 second to make sure that the static topics from the rosbag are published.
            delay=1.0,
        ))

    # Prepare topic lists for segmentation and detection based on namespace
    # These lists are now generated inside an OpaqueFunction to use resolved launch arguments
    def setup_dynamic_topic_lists(context):
        num_cameras_val_str = args.num_cameras.perform(context)
        try:
            num_cameras_val = int(num_cameras_val_str)
        except ValueError:
            print(f"Warning: Could not parse num_cameras '{num_cameras_val_str}' as int, defaulting to 1.")
            num_cameras_val = 1
        
        current_namespace_val = args.namespace.perform(context)

        # Base names for cameras, used for node sub-namespacing in detection/segmentation
        camera_base_names_list = [f'camera{i}' for i in range(num_cameras_val)]
        
        # Full topic paths, prefixed with the global namespace
        camera_input_topics_list = [f"{current_namespace_val}/{name}/color/image_raw" if current_namespace_val else f"/{name}/color/image_raw" for name in camera_base_names_list]
        input_camera_info_topics_list = [f"{current_namespace_val}/{name}/color/camera_info" if current_namespace_val else f"/{name}/color/camera_info" for name in camera_base_names_list]
        output_resized_image_topics_list = [f"{current_namespace_val}/{name}/segmentation/image_resized" if current_namespace_val else f"/{name}/segmentation/image_resized" for name in camera_base_names_list]
        output_resized_camera_info_topics_list = [f"{current_namespace_val}/{name}/segmentation/camera_info_resized" if current_namespace_val else f"/{name}/segmentation/camera_info_resized" for name in camera_base_names_list]

        segmentation_actions = []
        detection_actions = []

        # Check if 'people_segmentation' is part of the mode string
        mode_str = args.mode.perform(context) # Get the evaluated mode string
        if NvbloxMode.people_segmentation.name in mode_str: # Compare with enum's name
            segmentation_actions.append(
                lu.include(
                    'nvblox_examples_bringup',
                    'launch/perception/segmentation.launch.py',
                    launch_arguments={
                        'namespace': args.namespace, # Global namespace for the launch file
                        'container_name': args.container_name,
                        'people_segmentation': args.people_segmentation,
                        'namespace_list': camera_base_names_list, # Base names for sub-namespacing
                        'input_topic_list': camera_input_topics_list,
                        'input_camera_info_topic_list': input_camera_info_topics_list,
                        'output_resized_image_topic_list': output_resized_image_topics_list,
                        'output_resized_camera_info_topic_list': output_resized_camera_info_topics_list,
                        'num_cameras': args.num_cameras,
                        'one_container_per_camera': True 
                    }
                )
            )
        
        if NvbloxMode.people_detection.name in mode_str: # Compare with enum's name
            detection_actions.append(
                lu.include(
                    'nvblox_examples_bringup',
                    'launch/perception/detection.launch.py',
                    launch_arguments={
                        'namespace': args.namespace, # Global namespace
                        'namespace_list': camera_base_names_list, # Base names for sub-namespacing
                        'input_topic_list': camera_input_topics_list, # Full input topic paths
                        'num_cameras': args.num_cameras,
                        'container_name': args.container_name,
                        'one_container_per_camera': True
                    }
                )
            )
        return segmentation_actions + detection_actions

    actions.append(OpaqueFunction(function=setup_dynamic_topic_lists))
    # Nvblox
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/nvblox.launch.py',
            launch_arguments={
                'namespace': args.namespace,
                'container_name': args.container_name,
                'mode': args.mode,
                'camera': camera_mode,
                'num_cameras': args.num_cameras,
            }))

    # TF transforms for multi-realsense
    # Wrap robot_state_publisher in a GroupAction with PushRosNamespace
    robot_state_publisher_group = GroupAction(
        actions=[
            PushRosNamespace(args.namespace),
            lu.add_robot_description(
                robot_calibration_path=args.multicam_urdf_path,
                # The lu.add_robot_description itself doesn't take a namespace for the node.
                # Pushing the namespace should affect nodes created within this group.
            )
        ],
        condition=is_multi_cam
    )
    actions.append(robot_state_publisher_group)

    # Container
    actions.append(
        lu.component_container(
            args.container_name, # The name of the container itself
            condition=UnlessCondition(args.attach_to_container),
            log_level=args.log_level))

    return LaunchDescription(actions)