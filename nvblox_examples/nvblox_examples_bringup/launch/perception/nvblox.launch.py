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

from typing import List, Tuple

from launch import Action, LaunchDescription, LaunchContext
from launch_ros.descriptions import ComposableNode
from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

def get_isaac_sim_remappings(mode: NvbloxMode, num_cameras: int,
                             lidar: bool, namespace: str) -> List[Tuple[str, str]]:
    remappings = []
    camera_names = ['front_stereo_camera', 'left_stereo_camera',
                    'right_stereo_camera'][:num_cameras]
    for i, name in enumerate(camera_names):
        # Isaac Sim topics are typically global, so we don't add the namespace to the 'to' side here
        # unless the Isaac Sim setup itself is namespaced, which is not assumed by default.
        # If Isaac Sim topics ARE namespaced, the caller of this launch file needs to provide
        # the full namespaced topic names in a higher-level remapping.
        # The 'from' side (nvblox internal topics) will be namespaced by the node's namespace.
        remappings.append((f'camera_{i}/depth/image', f'/{name}/depth/ground_truth'))
        remappings.append((f'camera_{i}/depth/camera_info', f'/{name}/left/camera_info'))
        remappings.append((f'camera_{i}/color/image', f'/{name}/left/image_raw'))
        remappings.append((f'camera_{i}/color/camera_info', f'/{name}/left/camera_info'))
    if mode is NvbloxMode.people_segmentation:
        remappings.append(
            ('camera_0/mask/image', '/semantic_conversion/front_stereo_camera/semantic_mono8'))
        remappings.append(('camera_0/mask/camera_info', '/front_stereo_camera/left/camera_info'))
    if lidar:
        remappings.append(('pointcloud', '/front_3d_lidar/point_cloud'))
    return remappings

def get_realsense_remappings(mode: NvbloxMode, num_cameras: int = 1, namespace: str = "") -> List[Tuple[str, str]]:
    remappings = []
    for i in range(0, num_cameras):
        camera_prefix = f"camera{i}"
        if i == 0:
            # Only cam0 (i == 0) runs splitter.
            remappings.append(
                (f'camera_{i}/depth/image', f'camera{i}/realsense_splitter_node/output/depth'))
            remappings.append((f'camera_{i}/depth/camera_info', f'camera{i}/depth/camera_info'))
        else:
            remappings.append((f'camera_{i}/depth/image', f'camera{i}/depth/image_rect_raw'))
            remappings.append((f'camera_{i}/depth/camera_info', f'camera{i}/depth/camera_info'))

        if mode is NvbloxMode.people_segmentation:
            # nvblox takes resized images from semseg inputs
            remappings.append(
                (f'camera_{i}/color/image', f'camera{i}/segmentation/image_resized'))
            remappings.append(
                (f'camera_{i}/color/camera_info', f'camera{i}/segmentation/camera_info_resized'))
            remappings.append((f'camera_{i}/mask/image', f'camera{i}/segmentation/people_mask'))
            remappings.append(
                (f'camera_{i}/mask/camera_info', f'camera{i}/segmentation/camera_info_resized'))

        else:
            remappings.append((f'camera_{i}/color/image', f'camera{i}/color/image_raw'))
            remappings.append((f'camera_{i}/color/camera_info', f'camera{i}/color/camera_info'))

            if mode is NvbloxMode.people_detection:
                remappings.append((f'camera_{i}/mask/image', f'camera{i}/detection/people_mask'))
                remappings.append(
                    (f'camera_{i}/mask/camera_info', f'camera{i}/color/camera_info'))
    return remappings


def get_zed_remappings(mode: NvbloxMode, namespace: str) -> List[Tuple[str, str]]:
    assert mode is NvbloxMode.static, 'Nvblox only supports static mode for ZED cameras.'
    # ZED topics are typically under a fixed namespace like '/zed', so we use that directly.
    # If the ZED node itself is further namespaced, the caller needs to handle that.
    zed_base_topic_prefix = "/zed/zed_node/"
    # If a global namespace is provided, we prepend it. This assumes the ZED node is also under this global namespace.
    if namespace:
        zed_base_topic_prefix = f"/{namespace}{zed_base_topic_prefix}"

    remappings = []
    remappings.append(('camera_0/depth/image', f'{zed_base_topic_prefix}depth/depth_registered'))
    remappings.append(('camera_0/depth/camera_info', f'{zed_base_topic_prefix}depth/camera_info'))
    remappings.append(('camera_0/color/image', f'{zed_base_topic_prefix}rgb/image_rect_color'))
    remappings.append(('camera_0/color/camera_info', f'{zed_base_topic_prefix}rgb/camera_info'))
    remappings.append(('pose', f'{zed_base_topic_prefix}pose')) # VSLAM pose topic for ZED
    return remappings


def add_nvblox(context: LaunchContext, args: lu.ArgumentContainer) -> List[Action]:
    actions = []
    current_namespace_str = args.namespace.perform(context) # Evaluated string
    container_name_str = args.container_name.perform(context) # Evaluated string

    mode = NvbloxMode[args.mode.perform(context)]
    camera = NvbloxCamera[args.camera.perform(context)]
    num_cameras = int(args.num_cameras.perform(context))
    use_lidar = lu.is_true(args.lidar.perform(context))

    if camera == NvbloxCamera.realsense:
        assert num_cameras == 1, 'NvbloxCamera.realsense shall only be set for num_cameras==1'

    base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_base.yaml')
    segmentation_config = lu.get_path('nvblox_examples_bringup',
                                      'config/nvblox/specializations/nvblox_segmentation.yaml')
    detection_config = lu.get_path('nvblox_examples_bringup',
                                   'config/nvblox/specializations/nvblox_detection.yaml')
    dynamics_config = lu.get_path('nvblox_examples_bringup',
                                  'config/nvblox/specializations/nvblox_dynamics.yaml')
    isaac_sim_config = lu.get_path('nvblox_examples_bringup',
                                   'config/nvblox/specializations/nvblox_sim.yaml')
    realsense_config = lu.get_path('nvblox_examples_bringup',
                                   'config/nvblox/specializations/nvblox_realsense.yaml')
    multi_realsense_config = lu.get_path(
        'nvblox_examples_bringup', 'config/nvblox/specializations/nvblox_multi_realsense.yaml')
    zed_config = lu.get_path('nvblox_examples_bringup',
                             'config/nvblox/specializations/nvblox_zed.yaml')

    if mode is NvbloxMode.static:
        mode_config = {}
    elif mode is NvbloxMode.people_segmentation:
        mode_config = segmentation_config
        assert not use_lidar, 'Can not run lidar with people segmentation mode.'
    elif mode is NvbloxMode.people_detection:
        mode_config = detection_config
        assert not use_lidar, 'Can not run lidar with people detection mode.'
    elif mode is NvbloxMode.dynamic:
        mode_config = dynamics_config
        assert not use_lidar, 'Can not run lidar with dynamic mode.'
    else:
        raise Exception(f'Mode {mode} not implemented for nvblox.')

    if camera is NvbloxCamera.isaac_sim:
        remappings = get_isaac_sim_remappings(mode, num_cameras, use_lidar, current_namespace_str)
        camera_config = isaac_sim_config
        assert num_cameras <= 1 or mode is not NvbloxMode.people_segmentation, \
            'Can not run multiple cameras with people segmentation in Isaac Sim.'
    elif camera is NvbloxCamera.realsense:
        remappings = get_realsense_remappings(mode, num_cameras, current_namespace_str)
        camera_config = realsense_config
        assert not use_lidar, 'Can not run lidar for realsense example.'
    elif camera is NvbloxCamera.multi_realsense:
        remappings = get_realsense_remappings(mode, num_cameras, current_namespace_str)
        camera_config = multi_realsense_config
        assert not use_lidar, 'Can not run lidar for multi realsense example.'
    elif camera in [NvbloxCamera.zed2, NvbloxCamera.zedx]:
        remappings = get_zed_remappings(mode, current_namespace_str)
        camera_config = zed_config
        assert num_cameras == 1, 'Zed example can only run with 1 camera.'
        assert not use_lidar, 'Can not run lidar for zed example.'
    else:
        raise Exception(f'Camera {camera} not implemented for nvblox.')

    parameters = []
    parameters.append(base_config)
    parameters.append(mode_config)
    parameters.append(camera_config)
    parameters.append({'num_cameras': num_cameras})
    parameters.append({'use_lidar': use_lidar})
    parameters.append({'global_frame': 'odom'})


    nvblox_node = ComposableNode(
        name='nvblox_node',
        namespace=current_namespace_str, # Use evaluated string
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    actions.append(lu.component_container(
        container_name_str,
        condition=IfCondition(args.run_standalone) # Use IfCondition directly with the LaunchConfiguration
    ))
    
    actions.append(lu.load_composable_nodes(container_name_str, [nvblox_node]))
    actions.append(
        lu.log_info( # Pass LaunchConfigurations directly to log_info
            ["Starting nvblox with namespace: '", args.namespace,
             "', camera: '", args.camera, "', mode: '", args.mode, "'."]))
    return actions

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('namespace', '', description='Namespace for the nvblox node and topics')
    args.add_arg('mode', default=NvbloxMode.static.name, choices=NvbloxMode.names())
    args.add_arg('camera', default=NvbloxCamera.realsense.name, choices=NvbloxCamera.names())
    args.add_arg('num_cameras', 1, description='Number of cameras being used.')
    args.add_arg('lidar', 'False', description='Whether to use lidar data.')
    args.add_arg('global_frame', 'odom', description='The global frame of reference for nvblox.')
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')

    args.add_opaque_function(lambda context: add_nvblox(context, args))
    return LaunchDescription(args.get_launch_actions())