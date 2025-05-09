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

from typing import List, Optional, Union

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


EMITTER_FLASHING_CONFIG_FILE_PATH = lu.get_path('nvblox_examples_bringup', 'config/sensors/realsense_emitter_flashing.yaml')
EMITTER_ON_CONFIG_FILE_PATH = lu.get_path('nvblox_examples_bringup', 'config/sensors/realsense_emitter_on.yaml')

# By default our behaviour is:
# - Run the splitter on camera0,
# - Don't run the splitter on the remaining cameras.
# NOTE(alexmillane, 16.08.2024): At the moment this is the *only* behaviour we support.
def get_default_run_splitter_list(num_cameras: int) -> List[bool]:
    run_splitter_list = [False] * num_cameras
    if num_cameras > 0:
        run_splitter_list[0] = True
    return run_splitter_list


def get_camera_node(
    camera_name: str,
    config_file_path: str,
    serial_number: Optional[str] = None, # Serial number should be string
    namespace: Union[str, Substitution, List[Union[str, Substitution]]] = ''
) -> ComposableNode:
    parameters = []
    parameters.append(config_file_path)
    parameters.append({'camera_name': camera_name})
    if serial_number and serial_number.lower() != 'none' and serial_number != '':
        parameters.append({'serial_no': serial_number})
    realsense_node = ComposableNode(
        namespace=namespace,
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name=f'{camera_name}_realsense_camera', # Give a unique name to the node
        parameters=parameters)
    return realsense_node


def get_splitter_node(
    camera_name: str,
    namespace: Union[str, Substitution, List[Union[str, Substitution]]] = ''
) -> ComposableNode:
    # Construct remappings with the global namespace if provided
    # The camera_name itself is already part of the sub-namespace for topics
    
    # If namespace is a LaunchConfiguration or list containing it, it will be resolved.
    # If namespace is an empty string, topics will be relative to the node's final namespace.
    # If namespace is a non-empty string, it's a fixed prefix.

    # The f-string for remappings should use camera_name directly, as it's a local sub-identifier.
    # The overall node namespace is handled by the 'namespace' parameter of ComposableNode.
    realsense_splitter_node = ComposableNode(
        namespace=namespace,
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA'
        }],
        remappings=[
            ('input/infra_1', 'infra1/image_rect_raw'),
            ('input/infra_1_metadata', 'infra1/metadata'),
            ('input/infra_2', 'infra2/image_rect_raw'),
            ('input/infra_2_metadata', 'infra2/metadata'),
            ('input/depth', 'depth/image_rect_raw'),
            ('input/depth_metadata', 'depth/metadata'),
            ('input/pointcloud', 'depth/color/points'),
            ('input/pointcloud_metadata', 'depth/metadata'),
        ])
    return realsense_splitter_node


# Changed signature to accept context and the argument container
def add_cameras(context: LaunchContext, args_container: lu.ArgumentContainer) -> List[Action]:
    """Adds a camera and (optional) realsense splitter for each camera up to num_cameras."""

    # Perform substitutions to get concrete values for logic
    camera_serial_numbers_str = args_container.camera_serial_numbers.perform(context)
    num_cameras_str = args_container.num_cameras.perform(context)
    container_name_str = args_container.container_name.perform(context)
    # args_container.namespace is a LaunchConfiguration, can be passed directly to nodes

    # Serial numbers.
    if not camera_serial_numbers_str or camera_serial_numbers_str.lower() == 'none':
        # If no serial numbers are provided, we might operate based on num_cameras without specific serials
        # Assuming RealSenseNodeFactory can handle finding cameras by index if serial_no is not given
        camera_serial_numbers_list = [None] * int(num_cameras_str) # Create a list of Nones
    else:
        camera_serial_numbers_list = camera_serial_numbers_str.split(',')
    
    # Number of cameras to run
    try:
        num_cameras_val = int(num_cameras_str)
    except ValueError:
        raise ValueError(f"Invalid value for num_cameras: '{num_cameras_str}'. Must be an integer.")

    if not camera_serial_numbers_list and num_cameras_val > 0 :
         # This case implies num_cameras > 0 but no serials.
         # We'll rely on the driver to pick cameras if serials are None.
         camera_serial_numbers_list = [None] * num_cameras_val


    if num_cameras_val > len(camera_serial_numbers_list) and any(s is not None for s in camera_serial_numbers_list):
        # If specific serials are given, num_cameras cannot exceed the count of these serials.
         raise ValueError(
            f"num_cameras ({num_cameras_val}) cannot exceed the number of "
            f"provided camera_serial_numbers ({len(camera_serial_numbers_list)}) "
            "when serial numbers are specified."
        )
    
    # Run splitter list. I.e. a list of bools indicating per-camera if we should run a splitter.
    # This should be based on num_cameras_val, the actual number of cameras we will launch.
    run_splitter_list = get_default_run_splitter_list(num_cameras_val)

    actions = []
    for idx in range(num_cameras_val):
        # Use serial number if available for this index, otherwise None
        camera_serial_number = camera_serial_numbers_list[idx] if idx < len(camera_serial_numbers_list) else None
        run_splitter = run_splitter_list[idx]
        
        nodes_for_this_camera = [] # Renamed to avoid conflict with outer 'nodes' if any
        camera_name = f'camera{idx}'
        
        # Config file
        if run_splitter:
            config_file_path = EMITTER_FLASHING_CONFIG_FILE_PATH
        else:
            config_file_path = EMITTER_ON_CONFIG_FILE_PATH
            
        # Log message uses resolved Python variables
        log_message = lu.log_info(
            f'Setting up Realsense camera: {camera_name} '
            f'(Serial: {camera_serial_number if camera_serial_number else "Any"}), '
            f'Running splitter: {run_splitter}, '
            f'Namespace: {args_container.namespace.perform(context) if isinstance(args_container.namespace, Substitution) else args_container.namespace}'
        )
        
        # Define the full namespace for the camera node
        # This will be [GlobalNamespaceLaunchConfig, '/', 'cameraX']
        camera_node_namespace = [args_container.namespace, '/', camera_name]

        nodes_for_this_camera.append(
            get_camera_node(
                camera_name=camera_name, # This is for topic sub-namespacing, not the node's ROS namespace
                config_file_path=config_file_path,
                serial_number=camera_serial_number,
                namespace=camera_node_namespace # Pass the constructed full namespace
        ))
        
        # Splitter
        if run_splitter:
            # The splitter for camera0 should also be under the camera0 sub-namespace
            # but also respect the global namespace.
            splitter_node_namespace = [args_container.namespace, '/', camera_name]
            nodes_for_this_camera.append(
                get_splitter_node(
                    camera_name=camera_name, # For remappings
                    namespace=splitter_node_namespace # Pass the constructed full namespace
            ))
            
        # Load composable nodes for this camera with a delay
        # lu.load_composable_nodes expects a string for container_name
        load_nodes_action = lu.load_composable_nodes(container_name_str, nodes_for_this_camera)
        
        actions.append(log_message) # Log before trying to load
        if idx > 0: # Add delay only for subsequent cameras
            actions.append(
                TimerAction(
                    period=float(idx * 10.0), # Ensure period is float
                    actions=[load_nodes_action]
                )
            )
        else: # Load first camera immediately
            actions.append(load_nodes_action)

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME, cli=True)
    args.add_arg('run_standalone', 'True', cli=True)
    args.add_arg('camera_serial_numbers', '', description="Comma-separated list of camera serial numbers. Leave empty to auto-detect.", cli=True)
    args.add_arg('num_cameras', 1, description="Number of cameras to launch.", cli=True)
    args.add_arg('namespace', '', description='Global namespace for all nodes in this launch file.', cli=True)

    # Use a lambda to pass both context and the args ArgumentContainer
    args.add_opaque_function(lambda context: add_cameras(context, args))
    
    # Get all actions, including OpaqueFunction and DeclareLaunchArguments
    launch_actions = args.get_launch_actions() 
    
    # Add the component container if running standalone
    launch_actions.append(
        lu.component_container(
            args.container_name, # This is the name of the container node
            condition=IfCondition(lu.is_true(args.run_standalone))
        )
    )
    return LaunchDescription(launch_actions)