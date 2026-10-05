"""Helpers compartidos por los launch de muav_gcs_gz: descubrimiento de PX4 y entorno de Gazebo.

Replica lo que PX4 genera en <build>/gz_env.sh (plantilla gz_bridge/gz_env.sh.in): los modelos
de PX4 (x500_base, x500_gimbal, gimbal, ...) que este paquete referencia con model://, los
plugins de gz compilados por PX4 (MotorFailurePlugin, OpticalFlowSystem, ...) y el server.config
propio de PX4. Los mundos de este paquete no declaran plugins, asi que dependen de ese
server.config para tener sensores, viento, etc. El servidor de Gazebo tiene que verlos.
"""

import os
import shutil

from launch.actions import AppendEnvironmentVariable, SetEnvironmentVariable

PX4_BUILD_TARGET = 'px4_sitl_default'


def find_px4():
    # Get PX4 directory path - try multiple methods
    px4_dir = None

    # Method 1: Check environment variable PX4_DIR
    if 'PX4_DIR' in os.environ:
        px4_dir = os.environ['PX4_DIR']

    # Method 2: Try to find px4 executable in PATH and get its directory
    if px4_dir is None:
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


def px4_gz_env_actions():
    """Acciones de launch que exponen a Gazebo los modelos, plugins y server.config de PX4.

    Hay que ponerlas ANTES de la accion que arranca gz. Todo se deriva de find_px4():

    - GZ_SIM_RESOURCE_PATH     += <PX4>/Tools/simulation/gz/models
    - GZ_SIM_SYSTEM_PLUGIN_PATH += <PX4>/build/px4_sitl_default/src/modules/simulation/gz_plugins
    - GZ_SIM_SERVER_CONFIG_PATH = <PX4>/src/modules/simulation/gz_bridge/server.config

    Las dos primeras se AGREGAN (append): ros_gz ya pone ahi /opt/ros/<distro>/share. La tercera
    solo se define si el usuario no la habia fijado. Lo que no exista se avisa y se omite.
    """
    px4_dir = find_px4()
    models_dir = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    plugins_dir = os.path.join(
        px4_dir, 'build', PX4_BUILD_TARGET, 'src', 'modules', 'simulation', 'gz_plugins')
    server_config = os.path.join(px4_dir, 'src', 'modules', 'simulation', 'gz_bridge', 'server.config')

    actions = []

    def append_once(var, path):
        if path not in os.environ.get(var, '').split(os.pathsep):
            actions.append(AppendEnvironmentVariable(var, path))

    if os.path.isdir(models_dir):
        append_once('GZ_SIM_RESOURCE_PATH', models_dir)
    else:
        print(f"[WARNING] PX4 Gazebo models not found at {models_dir}. Set PX4_DIR (PX4 root).")

    if os.path.isdir(plugins_dir):
        append_once('GZ_SIM_SYSTEM_PLUGIN_PATH', plugins_dir)
    else:
        print(f"[WARNING] PX4 Gazebo plugins not found at {plugins_dir}. "
              f"Build PX4 first (make {PX4_BUILD_TARGET}).")

    if 'GZ_SIM_SERVER_CONFIG_PATH' not in os.environ:
        if os.path.isfile(server_config):
            actions.append(SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', server_config))
        else:
            print(f"[WARNING] PX4 server.config not found at {server_config}.")

    print(f"[DEBUG] PX4 Gazebo environment from: {px4_dir}")
    return actions
