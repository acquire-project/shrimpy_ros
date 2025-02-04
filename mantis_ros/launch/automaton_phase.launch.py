# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

example_parameters = {
    'blackfly_s': {
        'debug': True,
        'quiet': False,
        'compute_brightness': False,
        'adjust_timestamp': True,
        'dump_node_map': False,
        # set parameters defined in blackfly_s.yaml
        'gain_auto': 'Off',
        # 'pixel_format': 'BayerRG8',
        'exposure_auto': 'Off',
        # to use a user set, do this:
        # 'user_set_selector': 'UserSet0',
        # 'user_set_load': 'Yes',
        # These are useful for GigE cameras
        # 'device_link_throughput_limit': 380000000,
        # 'gev_scps_packet_size': 9000,
        # ---- to reduce the sensor width and shift the crop
        # 'image_width': 1408,
        # 'image_height': 1080,
        # 'offset_x': 16,
        # 'offset_y': 0,
        # 'binning_x': 1,
        # 'binning_y': 1,
        #'connect_while_subscribed': True,
        # Timing configuration
        #'acquisition_mode': 'MultiFrame',
        #'frame_count': 11,
        'exposure_time': 500,
        'frame_rate_auto': 'On',
        'frame_rate': 10.0,
        'frame_rate_enable': True,
        'trigger_mode': 'Off',
        'line2_selector': 'Line2',  # Select GPIO line
        'line2_linemode': 'Output',  # Configure as output
        'line2_source': 'ExposureActive',  # Signal during exposure
        'line2_v33enable': False,
        'buffer_queue_size': 10,
        'chunk_mode_active': True,
        'chunk_selector_frame_id': 'FrameID',
        'chunk_enable_frame_id': True,
        'chunk_selector_exposure_time': 'ExposureTime',
        'chunk_enable_exposure_time': True,
        'chunk_selector_gain': 'Gain',
        'chunk_enable_gain': True,
        'chunk_selector_timestamp': 'Timestamp',
        'chunk_enable_timestamp': True,
    },
}


def launch_setup(context, *args, **kwargs):
    """Launch camera driver node."""
    camera_parameter_file = LaunchConfig('camera_parameter_file').perform(context)
    camera_type = LaunchConfig('camera_type').perform(context)
    if not camera_parameter_file:
        camera_parameter_file = PathJoinSubstitution(
            [FindPackageShare('mantis_ros'), 'config', camera_type + '.yaml']
        )
    if camera_type not in example_parameters:
        raise Exception('no example parameters available for type ' + camera_type)

    parameter_file = LaunchConfig('parameter_file').perform(context)
    if not parameter_file:
        parameter_file = PathJoinSubstitution(
            [FindPackageShare('mantis_ros'), 'config', camera_type + '.yaml']
        ) 
    
    camera_node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        output='screen',
        name=[LaunchConfig('camera_name')],
        #prefix=['gdbserver localhost:3000'],
        parameters=[
            example_parameters[camera_type],
            {
                'ffmpeg_image_transport.encoding': 'hevc_nvenc',
                'parameter_file': camera_parameter_file,
                'serial_number': [LaunchConfig('serial')],
            },
        ],
        remappings=[
            ('~/control', '/exposure_control/control'),
        ],
    )

    triggerscope_node = Node(
        package='triggerscope_ros',
        executable='triggerscope_server_node',
        output='screen',
        name='triggerscope'
    )
    
    phase_acquisition_server = Node(
        package='mantis_ros',
        executable='phase_acquisition_action_server',
        output='screen',
        name='phase_acquisition'
    )
    
    phase_reconstruction = Node(
        package='waveorder_ros',
        executable='phase_reconstruction_node',
        output='screen',
        name='phase_reconstruction',
        remappings=[
            ('image_raw', '/flir_camera/image_raw'),
        ],
    )
    
    phase_zarr_writer = Node(
        package='acquire_zarr',
        executable='float_volume_zarr_writer',
        output='screen',
        name='phase_zarr_writer',
        parameters= [parameter_file],
        remappings=[
            ('image_data', '/phase_reconstruction'),
        ],
    )   
    
    raw_zarr_writer = Node(
        package='acquire_zarr',
        executable='image_zarr_writer_node',
        output='screen',
        name='raw_image_zarr_writer',
        parameters= [parameter_file],
        remappings=[
            ('image_data', '/flir_camera/image_raw'),
        ],
    )   
    
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='screen',
        name='foxglove_bridge'
    )

    nodes = [
        camera_node,
        triggerscope_node,
        phase_acquisition_server,
        phase_reconstruction,
        phase_zarr_writer,
        raw_zarr_writer,
        #foxglove_bridge,
    ]
    return nodes

def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription(
        [
            LaunchArg(
                'camera_name',
                default_value=['flir_camera'],
                description='camera name (ros node name)',
            ),
            LaunchArg(
                'camera_type',
                default_value='blackfly_s',
                description='type of camera (blackfly_s, chameleon...)',
            ),
            LaunchArg(
                'serial',
                default_value="'21278186'",
                description='FLIR serial number of camera (in quotes!!)',
            ),
            LaunchArg(
                'camera_parameter_file',
                default_value='',
                description='path to ros parameter definition file (override camera type)',
            ),
            LaunchArg(
                'parameter_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('mantis_ros'), 'config', 'automaton_config.yaml']),
                description='path to configuration YAML file for the configuration of ROS2 nodes',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
