#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # Get launch configurations (for camera bridge)
    vehicle = LaunchConfiguration('vehicle').perform(context)
    ID = LaunchConfiguration('ID').perform(context)
    world = LaunchConfiguration('world').perform(context)
    namespace_val = LaunchConfiguration('namespace').perform(context)
    enable_camera_val = LaunchConfiguration('enable_camera').perform(context)

    # List to hold camera bridge actions
    camera_actions = []
    
    # PX4 SITL launch
    px4_sitl_node = OpaqueFunction(function=launch_px4)

    # Only create camera bridge if enabled
    if enable_camera_val.lower() == 'true':
        # Determine namespace for remapping
        ns = f'px4_{ID}'
        if namespace_val and namespace_val != '':
            ns = namespace_val

        # Build topic names as strings (since we already performed the LaunchConfigurations)
        gz_topic = f'/world/{world}/model/{vehicle}_{ID}/link/camera_link/sensor/imager/image'
        ros_topic = f'/{ns}/camera/image'

        # Using ros_gz_image for more efficient camera bridging
        # This provides automatic compression support via image_transport
        camera_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            name=f'camera_bridge_{ID}',
            arguments=[gz_topic],
            remappings=[
                (gz_topic, ros_topic),
                (f'{gz_topic}/compressed', f'{ros_topic}/compressed')
            ],
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )

        # Delay camera bridge to ensure PX4 and Gazebo are fully started
        camera_bridge_delayed = TimerAction(
            period=6.0,
            actions=[camera_bridge]
        )
        camera_actions.append(camera_bridge_delayed)

        # NOTE: Compression node removed - ros_gz_image provides automatic compression
        # via image_transport. Compressed images are available at /{ns}/camera/image_raw/compressed
        # For H.264 compression, install: sudo apt install ros-humble-ffmpeg-image-transport
        
    return [
        px4_sitl_node,
    ] + camera_actions
    
def launch_px4(context):
    # Get PX4 directory path
    px4_dir = find_px4()
    # Resolve launch configurations to actual values
    x_val = context.launch_configurations['x']
    y_val = context.launch_configurations['y']
    z_val = context.launch_configurations['z']
    roll_val = context.launch_configurations['roll']
    pitch_val = context.launch_configurations['pitch']
    yaw_val = context.launch_configurations['yaw']
    vehicle_val = context.launch_configurations['vehicle']
    ID_val = context.launch_configurations['ID']
    autostart_val = context.launch_configurations['autostart']
    namespace_val = context.launch_configurations['namespace']
    # Build pose string
    pose_str = f"{x_val},{y_val},{z_val},{roll_val},{pitch_val},{yaw_val}"
    # Build environment variables dictionary
    env_vars = {
        'PX4_GZ_STANDALONE': '1',
        'PX4_SYS_AUTOSTART': autostart_val,
        'PX4_GZ_MODEL_POSE': pose_str,
        'PX4_SIM_MODEL': f"gz_{vehicle_val}",
        'PX4_SIM_SPEED_FACTOR': '1',  # Changed to '1' (was '1.0')
    }

    # Add GZ_PARTITION if set in environment (critical for Docker/containerized environments)
    if 'GZ_PARTITION' in os.environ:
        env_vars['GZ_PARTITION'] = os.environ['GZ_PARTITION']
        print(f"[DEBUG] GZ_PARTITION set to: {os.environ['GZ_PARTITION']}")
    else:
        print("[WARNING] GZ_PARTITION not found in environment!")
        
    if 'speedfactor' in os.environ:
        env_vars['PX4_SIM_SPEED_FACTOR'] = os.environ['speedfactor']
        print(f"[DEBUG] PX4_SIM_SPEED_FACTOR set to: {os.environ['speedfactor']}")

    # Add namespace if provided
    if namespace_val and namespace_val != '':
        env_vars['PX4_UXRCE_DDS_NS'] = namespace_val

    # Debug: Print all env vars being passed to PX4
    print(f"[DEBUG] Spawning PX4 with ID={ID_val}, pose={pose_str}")
    print(f"[DEBUG] Environment variables: {env_vars}")

    # Create PX4 process
    px4_process = ExecuteProcess(
        cmd=[
            px4_dir + '/build/px4_sitl_default/bin/px4',
            '-i', ID_val
        ],
        additional_env=env_vars,
        output='screen',
        shell=False,
        name=f'px4_{ID_val}'
    )
    return [px4_process]

def find_px4():
    # Get PX4 directory path - try multiple methods
    px4_dir = None

    # Method 1: Check environment variable PX4_DIR
    if 'PX4_DIR' in os.environ:
        px4_dir = os.environ['PX4_DIR']

    # Method 2: Try to find px4 executable in PATH and get its directory
    if px4_dir is None:
        import shutil
        px4_bin = shutil.which('px4')
        if px4_bin:
            # px4 binary is typically at PX4-Autopilot/build/px4_sitl_default/bin/px4
            # So we go up 4 levels to get the root directory
            px4_dir = os.path.abspath(os.path.join(os.path.dirname(px4_bin), '../../../..'))

    # Method 3: Check common PX4 installation locations
    if px4_dir is None:
        common_paths = [
            os.path.expanduser('~/PX4-Autopilot'),
            os.path.expanduser('~/px4'),
            '/opt/PX4-Autopilot',
            '/usr/local/PX4-Autopilot',
            '/PX4-Autopilot',
        ]
        for path in common_paths:
            if os.path.exists(os.path.join(path, 'build/px4_sitl_default/bin/px4')):
                px4_dir = path
                break

    # Method 4: Fallback - use default
    if px4_dir is None:
        px4_dir = os.path.expanduser('~/PX4-Autopilot')
        print(f"Warning: PX4 directory not found automatically. Using default: {px4_dir}")
        print("Set PX4_DIR environment variable or ensure 'px4' is in PATH")

    return px4_dir

def generate_launch_description():
    declared_arguments = []
    # Declare launch arguments
    # Vehicle pose (x,y,z,roll,pitch,yaw)
    declared_arguments.append( 
        DeclareLaunchArgument('x', default_value='0', description='X position')
        )
    declared_arguments.append(
        DeclareLaunchArgument('y', default_value='0', description='Y position')
        )
    declared_arguments.append(
        DeclareLaunchArgument('z', default_value='0', description='Z position')
        )
    declared_arguments.append(
        DeclareLaunchArgument('roll', default_value='0', description='Roll')
        )
    declared_arguments.append(
        DeclareLaunchArgument('pitch', default_value='0', description='Pitch')
        )
    declared_arguments.append(
        DeclareLaunchArgument('yaw', default_value='0', description='Yaw')
        )
    declared_arguments.append(
        DeclareLaunchArgument('vehicle',
        default_value='x500_mono_cam',
        description='Vehicle model (e.g., x500, x500_mono_cam)')
        )
    declared_arguments.append(
        DeclareLaunchArgument('ID', 
        default_value='1', 
        description='Vehicle instance ID mav_system_id = ID + 1')
        )
    declared_arguments.append(
        DeclareLaunchArgument('autostart', 
            default_value='4001', 
            description='PX4 autostart ID')
        )
    declared_arguments.append(
        DeclareLaunchArgument('namespace', 
            default_value='', 
            description='DDS namespace (e.g., px4_1, px4_2). If empty, no namespace is set.')
        )
    # Camera bridge option
    declared_arguments.append(
        DeclareLaunchArgument('enable_camera', default_value='false', description='Enable camera bridge (only works with camera-equipped models)')
        )
    declared_arguments.append(
        DeclareLaunchArgument('world', default_value='default', description='Gazebo world name')
        )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])