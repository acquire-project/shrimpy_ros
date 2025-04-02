import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
    
def generate_launch_description():

    # Declare launch arguments
    start_zenoh_router = LaunchConfiguration('start_zenoh_router')
    start_zenoh_router_launch_arg = DeclareLaunchArgument('start_zenoh_router', default_value='true', description='Start zenoh router (Default: true)')
    
    # Zarr 
    start_zarr_writer = LaunchConfiguration('start_zarr_writer')
    start_zarr_writer_launch_arg = DeclareLaunchArgument('start_zarr_writer', default_value='false', description='Start zarr writer (Default: false)')
    
    image_width = LaunchConfiguration('image_width')
    image_width_launch_arg = DeclareLaunchArgument('image_width', default_value='2448', description='Image width')
    
    image_height = LaunchConfiguration('image_height')
    image_height_launch_arg = DeclareLaunchArgument('image_height', default_value='2048', description='Image height')
    
    image_depth = LaunchConfiguration('image_depth')
    image_depth_launch_arg = DeclareLaunchArgument('image_depth', default_value='11', description='Image depth (number of images in z-stack)')
    
    zarr_out_path = LaunchConfiguration('zarr_out_path')
    zarr_out_path_launch_arg = DeclareLaunchArgument('zarr_out_path', default_value='/tmp/out.zarr', description='zarr output path')
    
    # Nodes
    zenoh_router = Node(
        package='rmw_zenoh_cpp', 
        executable='rmw_zenohd', 
        output='screen',
        condition=launch.conditions.IfCondition(start_zenoh_router))

    zarr_writer_node = Node(
        package='acquire_zarr', 
        executable='image_zarr_writer_node', 
        parameters=[{
            'zarr_path': zarr_out_path,
            'dimension_sizes': [0, image_depth, image_height, image_width],
            'chunk_sizes': [image_depth, image_height, image_width],
        }],
        output='screen',
        condition=launch.conditions.IfCondition(start_zarr_writer))

    waveorder_ros_node = Node(
        package='waveorder_ros',
        executable='phase_reconstruction_node',
        parameters=[{
            'xyz_shape': [image_depth, image_height, image_width],  # zyx shape
            'yx_pixel_size': 0.345,
            'z_pixel_size': 3,
            'index_of_refraction_media': 1,
            'z_padding': 5,
            'wavelength_illumination': 0.532,
            'numerical_aperture_illumination': 0.4,
            'numerical_aperture_detection': 0.45,
        }],
        remappings=[
                ('image_raw', '/flir_camera/image_raw'),
        ],
        output='screen')

    # List of nodes, arguements, etc to add to the launch description.
    return launch.LaunchDescription([
        start_zenoh_router_launch_arg,
        start_zarr_writer_launch_arg,
        image_width_launch_arg,
        image_height_launch_arg,
        image_depth_launch_arg,
        zarr_out_path_launch_arg,
        zenoh_router,
        zarr_writer_node,
        waveorder_ros_node])

