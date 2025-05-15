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
# Ensure LaunchConfiguration is available
from launch.substitutions import LaunchConfiguration

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
    namespace: Union[str, Substitution, List[Union[str, Substitution]]] = '',
    imu_optical_frame_id_override: Optional[str] = None # New argument for IMU frame ID
) -> ComposableNode:
    parameters = []
    parameters.append(config_file_path)
    parameters.append({'camera_name': camera_name})
    if serial_number and serial_number.lower() != 'none' and serial_number != '':
        parameters.append({'serial_no': serial_number})
    
    # Add the IMU frame ID override if provided and not empty
    if imu_optical_frame_id_override and imu_optical_frame_id_override.strip():
        parameters.append({'imu_optical_frame_id': imu_optical_frame_id_override})
        
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
    # The remappings are relative to the node's final resolved namespace.
    # The topics from realsense2_camera node are already sub-namespaced by camera_name
    # (e.g., /<global_ns>/<camera_name>/infra1/image_rect_raw).
    # The splitter node's remappings should map its internal topic names
    # to these existing, fully-qualified (or relative to node ns) topic names.
    realsense_splitter_node = ComposableNode(
        namespace=namespace, # This will be like [LaunchConfig('namespace'), '/', 'cameraX']
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA'
        }],
        remappings=[
            # These topics are expected to be published by the realsense_camera_node
            # already under its own sub-namespace (e.g., camera0/infra1/image_rect_raw).
            # The splitter node is in the same sub-namespace.
            ('input/infra_1', 'infra1/image_rect_raw'),
            ('input/infra_1_metadata', 'infra1/metadata'),
            ('input/infra_2', 'infra2/image_rect_raw'),
            ('input/infra_2_metadata', 'infra2/metadata'),
            ('input/depth', 'depth/image_rect_raw'),
            ('input/depth_metadata', 'depth/metadata'),
            ('input/pointcloud', 'depth/color/points'),
            ('input/pointcloud_metadata', 'depth/metadata'), # Assuming depth metadata can serve for pointcloud
        ])
    return realsense_splitter_node


# Changed signature to accept context and the argument container
def add_cameras(context: LaunchContext, args_container: lu.ArgumentContainer) -> List[Action]:
    """Adds a camera and (optional) realsense splitter for each camera up to num_cameras."""

    # Perform substitutions to get concrete values for logic
    camera_serial_numbers_str = args_container.camera_serial_numbers.perform(context)
    num_cameras_str = args_container.num_cameras.perform(context)
    container_name_str = args_container.container_name.perform(context)
    imu_optical_frame_id_override_str = args_container.imu_optical_frame_id_override.perform(context)

    # Use LaunchConfiguration('namespace') directly to get the substitution object
    namespace_lc = LaunchConfiguration('namespace')
    resolved_namespace_for_log = namespace_lc.perform(context)

    # Serial numbers.
    if not camera_serial_numbers_str or camera_serial_numbers_str.lower() == 'none':
        camera_serial_numbers_list = [None] * int(num_cameras_str) 
    else:
        camera_serial_numbers_list = camera_serial_numbers_str.split(',')
    
    try:
        num_cameras_val = int(num_cameras_str)
    except ValueError:
        raise ValueError(f"Invalid value for num_cameras: '{num_cameras_str}'. Must be an integer.")

    if not camera_serial_numbers_list and num_cameras_val > 0 :
         camera_serial_numbers_list = [None] * num_cameras_val

    if num_cameras_val > len(camera_serial_numbers_list) and any(s is not None for s in camera_serial_numbers_list):
         raise ValueError(
            f"num_cameras ({num_cameras_val}) cannot exceed the number of "
            f"provided camera_serial_numbers ({len(camera_serial_numbers_list)}) "
            "when serial numbers are specified."
        )
    
    run_splitter_list = get_default_run_splitter_list(num_cameras_val)

    actions = []
    for idx in range(num_cameras_val):
        camera_serial_number = camera_serial_numbers_list[idx] if idx < len(camera_serial_numbers_list) else None
        run_splitter = run_splitter_list[idx]
        
        nodes_for_this_camera = [] 
        camera_name = f'camera{idx}'
        
        if run_splitter:
            config_file_path = EMITTER_FLASHING_CONFIG_FILE_PATH
        else:
            config_file_path = EMITTER_ON_CONFIG_FILE_PATH
            
        log_message = lu.log_info(
            f'Setting up Realsense camera: {camera_name} '
            f'(Serial: {camera_serial_number if camera_serial_number else "Any"}), '
            f'Running splitter: {run_splitter}, '
            f'Namespace: {resolved_namespace_for_log}'
        )
        
        camera_node_ns_list = [namespace_lc, '/', camera_name]

        current_imu_override = None
        if idx == 0 and imu_optical_frame_id_override_str.strip():
            current_imu_override = imu_optical_frame_id_override_str
            log_message_imu = lu.log_info(f"Applying imu_optical_frame_id_override='{current_imu_override}' to {camera_name}")
            actions.append(log_message_imu)

        nodes_for_this_camera.append(
            get_camera_node(
                camera_name=camera_name, 
                config_file_path=config_file_path,
                serial_number=camera_serial_number,
                namespace=camera_node_ns_list, 
                imu_optical_frame_id_override=current_imu_override
        ))
        
        if run_splitter:
            splitter_node_ns_list = [namespace_lc, '/', camera_name]
            nodes_for_this_camera.append(
                get_splitter_node(
                    camera_name=camera_name, 
                    namespace=splitter_node_ns_list
            ))
            
        load_nodes_action = lu.load_composable_nodes(container_name_str, nodes_for_this_camera)
        
        actions.append(log_message)
        if idx > 0: 
            actions.append(
                TimerAction(
                    period=float(idx * 10.0), 
                    actions=[load_nodes_action]
                )
            )
        else: 
            actions.append(load_nodes_action)

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME, cli=True)
    args.add_arg('run_standalone', 'True', cli=True)
    args.add_arg('camera_serial_numbers', '', description="Comma-separated list of camera serial numbers. Leave empty to auto-detect.", cli=True)
    args.add_arg('num_cameras', 1, description="Number of cameras to launch.", cli=True)
    args.add_arg('namespace', '', description='Global namespace for all nodes in this launch file.', cli=True)
    args.add_arg('imu_optical_frame_id_override', '', description='Override for the IMU optical frame ID for camera0. If empty, driver default is used.', cli=True)

    args.add_opaque_function(lambda context: add_cameras(context, args))
    
    launch_actions = args.get_launch_actions() 
    
    launch_actions.append(
        lu.component_container(
            LaunchConfiguration('container_name'), 
            condition=IfCondition(lu.is_true(LaunchConfiguration('run_standalone')))
        )
    )
    return LaunchDescription(launch_actions)