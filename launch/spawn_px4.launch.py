#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Declare launch arguments
    # Vehicle pose (x,y,z,roll,pitch,yaw)
    x_arg = DeclareLaunchArgument('x', default_value='0', description='X position')
    y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0', description='Z position')
    roll_arg = DeclareLaunchArgument('roll', default_value='0', description='Roll')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0', description='Pitch')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0', description='Yaw')

    # Vehicle model and config
    vehicle_arg = DeclareLaunchArgument(
        'vehicle',
        default_value='gz_x500',
        description='Vehicle model (e.g., gz_x500, gz_x500_mono_cam)'
    )

    ID_arg = DeclareLaunchArgument('ID', default_value='1', description='Vehicle instance ID')

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='4001',
        description='PX4 autostart ID'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='DDS namespace (e.g., px4_1, px4_2). If empty, no namespace is set.'
    )

    # Camera bridge option
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable camera bridge (only works with camera-equipped models)'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='default',
        description='Gazebo world name'
    )

    # Get launch configurations
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    vehicle = LaunchConfiguration('vehicle')
    ID = LaunchConfiguration('ID')
    autostart = LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace')
    enable_camera = LaunchConfiguration('enable_camera')
    world = LaunchConfiguration('world')

    # Get PX4 directory path
    px4_dir = find_px4()

    # Build PX4 command with environment variables
    # Example: PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2
    px4_cmd = [
        'bash', '-c',
        ' '.join([
            'PX4_GZ_STANDALONE=1',
            ['PX4_SYS_AUTOSTART=', autostart],
            # Construct pose string: "x,y,z,roll,pitch,yaw"
            'PX4_GZ_MODEL_POSE=\\"',
            x, ',', y, ',', z, ',', roll, ',', pitch, ',', yaw, '\\"',
            ['PX4_SIM_MODEL=', vehicle],
            # Add namespace if provided
            PythonExpression([
                '"PX4_UXRCE_DDS_NS=', namespace, '" if "', namespace, '" != "" else ""'
            ]),
            [px4_dir, '/build/px4_sitl_default/bin/px4'],
            '-i', ID
        ])
    ]

    px4_sitl_node = ExecuteProcess(
        cmd=px4_cmd,
        output='screen',
        shell=False,
        name=['px4_', ID]
    )

    # Camera bridge node (only launched if enable_camera is true)
    # Example: ros2 run ros_gz_bridge parameter_bridge /world/default/model/x500_mono_cam_2/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=['camera_bridge_', ID],
        arguments=[
            ['/world/', world, '/model/', vehicle, '_', ID, '/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image']
        ],
        remappings=[
            (["/world/", world, "/model/", vehicle, "_", ID, "/link/camera_link/sensor/camera/image"], [vehicle, "/camera/image_raw"]),
        ],
        output='screen',
        condition=IfCondition(enable_camera),
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        # Declare all arguments
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        vehicle_arg,
        ID_arg,
        autostart_arg,
        namespace_arg,
        enable_camera_arg,
        world_arg,

        # Launch PX4 SITL
        px4_sitl_node,

        # Launch camera bridge (conditional)
        camera_bridge,
    ])

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