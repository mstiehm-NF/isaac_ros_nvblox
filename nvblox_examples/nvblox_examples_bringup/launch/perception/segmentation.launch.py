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

import ast
from typing import List

from launch import Action, LaunchDescription, LaunchContext
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME, \
    SEMSEGNET_INPUT_IMAGE_WIDTH, SEMSEGNET_INPUT_IMAGE_HEIGHT


def create_segmentation_pipeline(context: LaunchContext,
                                 args: lu.ArgumentContainer,
                                 pipeline_namespace: str,
                                 input_topic: str,
                                 input_camera_info_topic: str,
                                 output_resized_image_topic: str,
                                 output_resized_camera_info_topic: str,
                                 main_container_fqn: str) -> List[Action]:

    people_segmentation_str = args.people_segmentation.perform(context)
    people_segmentation = NvbloxPeopleSegmentation[people_segmentation_str]

    vanilla_engine_file_path_val = args.vanilla_engine_file_path.perform(context)
    shuffleseg_engine_file_path_val = args.shuffleseg_engine_file_path.perform(context)

    network_image_width_val = int(args.network_image_width.perform(context))
    network_image_height_val = int(args.network_image_height.perform(context))

    # Default tensor name args are strings representing lists, parse them.
    input_tensor_names_str = args.input_tensor_names.perform(context)
    input_tensor_names_val = ast.literal_eval(input_tensor_names_str)
    output_binding_names_str = args.output_binding_names.perform(context)
    output_binding_names_val = ast.literal_eval(output_binding_names_str)
    output_tensor_names_str = args.output_tensor_names.perform(context)
    output_tensor_names_val = ast.literal_eval(output_tensor_names_str)

    force_engine_update_val = lu.is_true(args.force_engine_update.perform(context))
    verbose_val = lu.is_true(args.verbose.perform(context))
    network_output_type_val = args.network_output_type.perform(context)
    color_segmentation_mask_encoding_val = args.color_segmentation_mask_encoding.perform(context)
    one_container_per_camera_val = lu.is_true(args.one_container_per_camera.perform(context))
    log_level_val = args.log_level.perform(context)
    base_container_name_val = args.container_name.perform(context)


    if people_segmentation is NvbloxPeopleSegmentation.peoplesemsegnet_vanilla:
        engine_file_path = vanilla_engine_file_path_val
        input_binding_names = ['input_1:0']  # Existing hardcoded value
    elif people_segmentation is NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg:
        engine_file_path = shuffleseg_engine_file_path_val
        input_binding_names = ['input_2']  # Existing hardcoded value
    else:
        raise Exception(f'People segmentation mode {people_segmentation} not implemented.')

    resize_node = ComposableNode(
        name='segmentation_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace=pipeline_namespace,
        parameters=[{
            'output_width': network_image_width_val,
            'output_height': network_image_height_val,
            'keep_aspect_ratio': False,
            'input_qos': 'SENSOR_DATA',
        }],
        remappings=[
            ('image', input_topic),
            ('camera_info', input_camera_info_topic),
            ('resize/image', output_resized_image_topic),
            ('resize/camera_info', output_resized_camera_info_topic),
        ]
    )

    if people_segmentation is NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg:
        people_preprocessing_node = ComposableNode(
            name='image_to_tensor_node',
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
            namespace=pipeline_namespace,
            parameters=[{
                'scale': True,
                'tensor_name': input_tensor_names_val[0],
            }],
            remappings=[
                ('image', output_resized_image_topic),
                ('tensor', 'segmentation/tensor_input'),
            ]
        )
        nodes_list = [resize_node, people_preprocessing_node]
    else: # Vanilla
        people_preprocessing_node = ComposableNode(
            name='image_to_tensor_node',
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
            namespace=pipeline_namespace,
            parameters=[{
                'scale': True,
                'tensor_name': input_tensor_names_val[0],
            }],
            remappings=[
                ('image', output_resized_image_topic),
                ('tensor', 'segmentation/image_to_tensor_output'),
            ]
        )
        people_bchw_node = ComposableNode(
            name='interleaved_to_planar_node',
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::InterleavedToPlanarNode',
            namespace=pipeline_namespace,
            parameters=[
                {
                    'input_tensor_shape': [network_image_height_val, network_image_width_val, 3],
                    'num_blocks': 40,
                }
            ],
            remappings=[
                ('interleaved_tensor', 'segmentation/image_to_tensor_output'),
                ('planar_tensor', 'segmentation/tensor_input')
            ],
        )
        nodes_list = [resize_node, people_preprocessing_node, people_bchw_node]

    people_tensor_rt_node = ComposableNode(
        name='people_trt_node',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        namespace=pipeline_namespace,
        parameters=[{
            'engine_file_path': engine_file_path,
            'output_binding_names': output_binding_names_val,
            'output_tensor_names': output_tensor_names_val,
            'input_tensor_names': input_tensor_names_val,
            'input_binding_names': input_binding_names,
            'force_engine_update': force_engine_update_val,
            'verbose': verbose_val,
        }],
        remappings=[
            ('tensor_pub', 'segmentation/tensor_input'),
            ('tensor_sub', 'segmentation/tensor_output')
        ]
    )
    nodes_list.append(people_tensor_rt_node)

    people_decoder_node = ComposableNode(
        name='unet_decoder_node',
        package='isaac_ros_unet',
        plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
        namespace=pipeline_namespace,
        parameters=[{
            'network_output_type': network_output_type_val,
            'color_segmentation_mask_encoding': color_segmentation_mask_encoding_val,
            'mask_width': network_image_width_val,
            'mask_height': network_image_height_val,
            'color_palette': [
                0x556B2F, 0x800000, 0x008080, 0x000080, 0x9ACD32, 0xFF0000, 0xFF8C00, 0xFFD700,
                0x00FF00, 0xBA55D3, 0x00FA9A, 0x00FFFF, 0x0000FF, 0xF08080, 0xFF00FF, 0x1E90FF,
                0xDDA0DD, 0xFF1493, 0x87CEFA, 0xFFDEAD
            ],
        }],
        remappings=[
            ('tensor_sub', 'segmentation/tensor_output'),
            ('unet/raw_segmentation_mask', 'segmentation/people_mask')
        ]
    )
    nodes_list.append(people_decoder_node)

    if one_container_per_camera_val:
        # Container name is node name. Namespace is applied to this node name.
        per_camera_container_name = base_container_name_val + '_people_segmentation_' + pipeline_namespace.replace('/', '_')
        segmentation_container_action = ComposableNodeContainer(
            name=per_camera_container_name, # Node name for this specific container
            package='rclcpp_components',
            namespace=pipeline_namespace, # ROS namespace for this container node
            executable='component_container_mt',
            arguments=['--ros-args', '--log-level', log_level_val],
            composable_node_descriptions=nodes_list,
            output='screen'
        )
        return [segmentation_container_action]
    else:
        # Nodes are loaded into the main_container_fqn.
        # nodes_list already have their 'namespace' parameter set to pipeline_namespace.
        return [lu.load_composable_nodes(main_container_fqn, nodes_list)]


def add_segmentation(context: LaunchContext, args: lu.ArgumentContainer) -> List[Action]:
    actions = []
    global_namespace_val = args.namespace.perform(context)
    num_cameras_val = int(args.num_cameras.perform(context))

    # These are expected to be Python lists if passed from realsense_example.launch.py,
    # or string representations from CLI/defaults that need parsing.
    # For robustness, always try to parse if it's a string.
    def parse_list_arg_from_string(arg_str_val_from_context):
        if isinstance(arg_str_val_from_context, str):
            try:
                return ast.literal_eval(arg_str_val_from_context)
            except (ValueError, SyntaxError) as e:
                raise RuntimeError(f"Failed to parse list argument string '{arg_str_val_from_context}': {e}")
        # This case should ideally not be hit if the input is always a string from .perform()
        # that needs ast.literal_eval. If it can be a direct list (e.g. from parent launch file),
        # then this check is useful.
        elif isinstance(arg_str_val_from_context, list):
            return arg_str_val_from_context
        else:
            raise TypeError(f"Argument '{arg_str_val_from_context}' is not a list or string representation of a list.")

    cam_base_ns_list = parse_list_arg_from_string(args.namespace_list.perform(context))
    input_topics = parse_list_arg_from_string(args.input_topic_list.perform(context))
    input_cam_infos = parse_list_arg_from_string(args.input_camera_info_topic_list.perform(context))
    output_resized_imgs = parse_list_arg_from_string(args.output_resized_image_topic_list.perform(context))
    output_resized_cam_infos = parse_list_arg_from_string(args.output_resized_camera_info_topic_list.perform(context))


    assert len(cam_base_ns_list) == len(input_topics), \
        "Number of namespace_list must match number of input_topic_list!"
    assert len(input_cam_infos) == len(input_topics), \
        "Number of input_camera_info_topic_list must match number of input_topic_list!"
    assert len(output_resized_imgs) == len(input_topics), \
        "Number of output_resized_image_topic_list must match number of input_topic_list!"
    assert len(output_resized_cam_infos) == len(input_topics), \
        "Number of output_resized_camera_info_topic_list must match number of input_topic_list!"
    assert len(input_topics) > 0, \
        "At least one input topic must be provided to people segmentation!"
    assert num_cameras_val > 0, \
        "At least one camera must be enabled for people segmentation!"
    assert num_cameras_val <= len(input_topics), \
        "Number of input topics must not be less than number of cameras!"

    run_standalone_val = lu.is_true(args.run_standalone.perform(context))
    one_container_per_camera_val = lu.is_true(args.one_container_per_camera.perform(context))
    base_container_name = args.container_name.perform(context)

    # Determine the fully qualified name of the main container if it's used
    # The FQN for a container node is typically /<namespace>/<name>
    main_container_fqn = f"/{global_namespace_val}/{base_container_name}" if global_namespace_val else f"/{base_container_name}"


    if run_standalone_val and not one_container_per_camera_val:
        actions.append(lu.component_container(name=base_container_name, namespace=global_namespace_val))

    for i in range(num_cameras_val):
        cam_base_ns = cam_base_ns_list[i]
        effective_cam_pipeline_ns = f"{global_namespace_val}/{cam_base_ns}" if global_namespace_val else cam_base_ns
        
        pipeline_actions = create_segmentation_pipeline(
            context,
            args,
            pipeline_namespace=effective_cam_pipeline_ns,
            input_topic=input_topics[i],
            input_camera_info_topic=input_cam_infos[i],
            output_resized_image_topic=output_resized_imgs[i],
            output_resized_camera_info_topic=output_resized_cam_infos[i],
            main_container_fqn=main_container_fqn
        )
        actions.extend(pipeline_actions)
    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('namespace', '', description='Global namespace for all nodes and topics in this launch file')
    args.add_arg('people_segmentation',
                 NvbloxPeopleSegmentation.peoplesemsegnet_vanilla.name, # Use .name for default string
                 choices=[NvbloxPeopleSegmentation.peoplesemsegnet_vanilla.name,
                          NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg.name],
                 description='People Segmentation model')
    args.add_arg('num_cameras', 1,
                 description='Number of cameras requiring people segmentation pipeline')
    args.add_arg('namespace_list', '["camera0"]',
                 description='List of base namespaces for each segmentation inference pipeline (e.g., ["camera0", "camera1"])')

    args.add_arg(
        'input_topic_list',
        '["/camera0/color/image_raw"]', 
        description='List of camera image input topics for each segmentation inference pipeline')
    args.add_arg(
        'input_camera_info_topic_list',
        '["/camera0/color/camera_info"]', 
        description='List of input camera info topics for each segmentation inference pipeline')
    args.add_arg(
        'output_resized_image_topic_list',
        '["/camera0/segmentation/image_resized"]', 
        description='List of output resized image topics for each segmentation inference pipeline')
    args.add_arg(
        'output_resized_camera_info_topic_list',
        '["/camera0/segmentation/camera_info_resized"]',
        description='List of output resized camera info topics for each segmentation pipeline')

    args.add_arg('network_image_width', SEMSEGNET_INPUT_IMAGE_WIDTH,
                 description='Number of columns for network input tensor image')
    args.add_arg('network_image_height', SEMSEGNET_INPUT_IMAGE_HEIGHT,
                 description='Number of rows for network input tensor image')
    args.add_arg('verbose', 'False',
                 description='TensorRT verbosely log if True')
    args.add_arg('force_engine_update', 'False',
                 description='TensorRT update the TensorRT engine file if True')
    default_base_dir = '/opt/nvidia/isaac_ros/models/peoplesemsegnet' 
    args.add_arg('shuffleseg_engine_file_path',
                 lu.get_isaac_ros_ws_path() + 
                 f'{default_base_dir}/optimized_deployable_shuffleseg_unet_amr_v1.0/1/model.plan',
                 description='Full path to shuffleseg model TRT engine')
    args.add_arg('vanilla_engine_file_path',
                 lu.get_isaac_ros_ws_path() + 
                 f'{default_base_dir}/deployable_quantized_vanilla_unet_onnx_v2.0/1/model.plan',
                 description='Full path to vanilla model TRT engine')
    args.add_arg('input_tensor_names', '["input_tensor"]',
                 description='List of TRT input tensor names (string representation of list)')
    args.add_arg('input_tensor_formats', '["nitros_tensor_list_nchw_rgb_f32"]',
                 description='List of TRT input tensor nitros type formats (string representation of list)')
    args.add_arg('output_tensor_names', '["output_tensor"]',
                 description='List of TRT output tensor names (string representation of list)')
    args.add_arg('output_binding_names', '["argmax_1"]',
                 description='List of TRT output tensor binding names (string representation of list)')
    args.add_arg('output_tensor_formats', '["nitros_tensor_list_nhwc_rgb_f32"]',
                 description='List of TRT output tensor nitros type formats (string representation of list)')
    args.add_arg('network_output_type', 'argmax')
    args.add_arg('color_segmentation_mask_encoding', 'rgb8')
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME,
                 description='Base name of container where segmentation nodes are (if not one_container_per_camera)')
    args.add_arg('run_standalone', 'False',
                 description='Run in a standalone container if True and not one_container_per_camera')
    args.add_arg('one_container_per_camera', 'True',
                 description='Run per-camera based segmentation nodes in separate containers if True')
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)

    args.add_opaque_function(lambda context: add_segmentation(context, args))
    return LaunchDescription(args.get_launch_actions())