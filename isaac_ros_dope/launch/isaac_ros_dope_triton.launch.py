# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for DOPE encoder->Triton->DOPE decoder."""
    model_name = 'dope_ketchup_pol'
    launch_dir_path = os.path.dirname(os.path.realpath(__file__))
    model_dir_path = launch_dir_path + '/../../test/models'

    dope_encoder_node = ComposableNode(
        name='dope_encoder',
        package='isaac_ros_dnn_encoders',
        plugin='isaac_ros::dnn_inference::DnnImageEncoderNode',
        parameters=[{
            'network_image_width': 640,
            'network_image_height': 480,
            'network_image_encoding': 'rgb8',
            'network_normalization_type': 'positive_negative'
        }],
        remappings=[('encoded_tensor', 'tensor_pub')])

    dope_inference_node = ComposableNode(
        name='dope_inference',
        package='isaac_ros_triton',
        plugin='isaac_ros::dnn_inference::TritonNode',
        parameters=[{
            'model_name': model_name,
            'model_repository_paths': [model_dir_path],
            'max_batch_size': 0,
            'input_tensor_names': ['input'],
            'input_binding_names': ['input'],
            'output_tensor_names': ['output'],
            'output_binding_names': ['output']
        }])

    dope_decoder_node = ComposableNode(
        name='dope_decoder',
        package='isaac_ros_dope',
        plugin='isaac_ros::dope::DopeDecoderNode',
        parameters=[{
            'object_name': 'Ketchup',
            'frame_id': 'map'
        }],
        remappings=[('belief_map_array', 'tensor_sub'),
                    ('dope/pose_array', 'poses')])

    container = ComposableNodeContainer(
        name='dope_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[dope_encoder_node, dope_inference_node, dope_decoder_node],
        output='screen',
    )

    return LaunchDescription([container])
