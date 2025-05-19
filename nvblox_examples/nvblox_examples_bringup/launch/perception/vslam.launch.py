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

from typing import List

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

def add_vslam(context: LaunchContext, args: lu.ArgumentContainer) -> List[Action]:
    actions = []

    # Get the namespace value (evaluated string)
    current_namespace_str = args.namespace.perform(context)
    
    # Determine camera type (used for base_frame logic)
    camera_str = args.camera.perform(context)
    camera = NvbloxCamera[camera_str]

    camera0_prefix = "camera0"

    realsense_remappings = [
        ('visual_slam/camera_info_0', f'{camera0_prefix}/infra1/camera_info'),
        ('visual_slam/camera_info_1', f'{camera0_prefix}/infra2/camera_info'),
        ('visual_slam/image_0', f'{camera0_prefix}/realsense_splitter_node/output/infra_1'),
        ('visual_slam/image_1', f'{camera0_prefix}/realsense_splitter_node/output/infra_2'),
        ('visual_slam/imu', f'{camera0_prefix}/imu'),
    ]

    map_frame_tf = 'map'
    odom_frame_tf = 'odom'
    rig_frame_tf = 'base_link'
    base_frame_tf = 'base_link'

    # Log with evaluated strings since it's an f-string
    actions.append(lu.log_info(f'Starting cuVSLAM with namespace: "{current_namespace_str}", base_frame: "{base_frame_tf}"'))

    base_parameters = {
        'num_cameras': 2,
        'min_num_images': 2,
        'enable_localization_n_mapping': True,
        'gyro_noise_density': 0.000244,
        'gyro_random_walk': 0.000019393,
        'accel_noise_density': 0.001862,
        'accel_random_walk': 0.003,
        'calibration_frequency': 200.0,
        'rig_frame': rig_frame_tf, 
        'enable_slam_visualization': False,
        'enable_landmarks_view': False,
        'enable_observations_view': False,
        'path_max_size': 1000000,
        'verbosity': 5,
        'enable_debug_mode': False,
        'debug_dump_path': '/tmp/cuvslam',
        'map_frame': map_frame_tf,      
        'odom_frame': odom_frame_tf,     
        'base_frame': base_frame_tf,
        'enable_ground_constraint_in_odometry': True,
        'enable_ground_constraint_in_slam': True,
        'enable_imu_fusion': True,  
    }
    
    imu_frame_tf =f'{camera0_prefix}_gyro_optical_frame'
    camera_optical_frames_tf = [
        f'{camera0_prefix}_infra1_optical_frame',
        f'{camera0_prefix}_infra2_optical_frame',
    ]

    realsense_parameters = {
        'enable_rectified_pose': True,
        'enable_image_denoising': True,
        'rectified_images': True,
        'imu_frame': imu_frame_tf, 
        'camera_optical_frames': camera_optical_frames_tf, 
    }

    if camera is NvbloxCamera.realsense or camera is NvbloxCamera.multi_realsense:
        remappings = realsense_remappings
        camera_parameters = realsense_parameters
    else:
        raise Exception(f'Camera type "{camera_str}" not implemented for vslam.')

    parameters = []
    parameters.append(base_parameters)
    parameters.append(camera_parameters)
    parameters.append(
        {'enable_ground_constraint_in_odometry': lu.is_true(args.enable_ground_constraint_in_odometry.perform(context))})
    parameters.append({'enable_imu_fusion': lu.is_true(args.enable_imu_fusion.perform(context))})

    vslam_node = ComposableNode(
        name='visual_slam_node',
        namespace=current_namespace_str, # Use evaluated string for ComposableNode namespace
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=remappings,
        parameters=parameters)
    
    container_name_str = args.container_name.perform(context) # Evaluated string
    actions.append(lu.load_composable_nodes(container_name_str, [vslam_node]))

    actions.append(lu.component_container(
        container_name_str,
        condition=IfCondition(args.run_standalone) # Use IfCondition directly with the LaunchConfiguration
    ))

    return actions

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('namespace', '', description='Namespace for the vslam node and topics') 
    args.add_arg('camera', description='Camera type (e.g., realsense, multi_realsense)')
    args.add_arg(
        'enable_ground_constraint_in_odometry',
        'True',
        description='Whether to constraint robot movement to a 2d plane (e.g. for AMRs).',
        cli=True)
    args.add_arg(
        'enable_imu_fusion',
        'True',
        description='Whether to use imu data in visual slam.',
        cli=True)
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    
    # Use OpaqueFunction to allow context-based evaluation of launch arguments
    args.add_opaque_function(lambda context: add_vslam(context, args))
    
    return LaunchDescription(args.get_launch_actions())