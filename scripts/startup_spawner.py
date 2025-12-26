#!/usr/bin/env python3

import os
import yaml
import subprocess
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class StartupSpawner(Node):
    def __init__(self):
        super().__init__('startup_spawner')

        # Declare parameters
        self.declare_parameter('config_file', '')

        # Get config file path
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        # Use default if empty
        if config_file == "":
            config_file = os.path.join(
                get_package_share_directory('muav_gcs_gz'),
                'config',
                'scene.yaml'
            )

        self.get_logger().info(f'Loading configuration from: {config_file}')

        # Load configuration
        with open(config_file, 'r') as file:
            self.config_data = yaml.safe_load(file)

        # Get models directory
        self.models_dir = os.path.join(
            get_package_share_directory('muav_gcs_gz'),
            'models'
        )

        # Wait a bit for Gazebo to be fully ready
        time.sleep(2)

        # Spawn entities from config
        self.spawn_entities()

    def spawn_entities(self):
        """Spawn all entities defined in the configuration file"""

        for entity_name, entity_data in self.config_data.items():
            # Skip 'origin' entry
            if entity_name == 'origin':
                continue

            entity_type = entity_data.get('type')

            # Skip drones - they are spawned by the launch file
            if entity_type == 'drone':
                continue

            pose = entity_data.get('pose', {})

            # Map entity types to model names
            model_name = None
            if entity_type == 'base':
                model_name = 'uav_base'
            elif entity_type == 'windTurbine':
                model_name = 'wind_turbine'
            else:
                self.get_logger().warn(f'Unknown entity type: {entity_type}')
                continue

            # Get pose data
            xyz = pose.get('xyz', [0, 0, 0])
            rpy = pose.get('rpy', [0, 0, 0])

            # Spawn the entity
            self.spawn_entity(entity_name, model_name, xyz, rpy)

    def spawn_entity(self, entity_name, model_name, xyz, rpy):
        """Spawn a single entity in Gazebo using ros_gz_sim create"""

        model_path = os.path.join(self.models_dir, model_name, 'model.sdf')

        # Check if model file exists
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            return

        # Read the SDF file
        with open(model_path, 'r') as file:
            sdf_content = file.read()

        # Build the spawn command using ros2 run ros_gz_sim create
        self.get_logger().info(f'Spawning {entity_name} ({model_name}) at {xyz}')

        # Use ros2 run ros_gz_sim create
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', entity_name,
            '-file', model_path,
            '-x', str(xyz[0]),
            '-y', str(xyz[1]),
            '-z', str(xyz[2]),
            '-R', str(rpy[0]),
            '-P', str(rpy[1]),
            '-Y', str(rpy[2]),
            '-world', 'default'
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                self.get_logger().info(f'Successfully spawned: {entity_name}')
            else:
                self.get_logger().error(f'Failed to spawn {entity_name}: {result.stderr}')
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Timeout spawning {entity_name}')
        except Exception as e:
            self.get_logger().error(f'Error spawning {entity_name}: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    spawner = StartupSpawner()

    spawner.get_logger().info('Startup spawning complete')

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
