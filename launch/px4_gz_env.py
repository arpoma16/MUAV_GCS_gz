"""Helpers compartidos por los launch de muav_gcs_gz: descubrimiento de PX4 y entorno de Gazebo.

Los modelos de PX4 (x500_base, x500_gimbal, gimbal, ...) y su server.config viven en
<PX4>/Tools/simulation/gz. Los modelos de este paquete los referencian con model://, y
los mundos dependen del server.config para tener sensores, asi que el servidor de Gazebo
tiene que verlos en su entorno.
"""

import os
import shutil

from launch.actions import AppendEnvironmentVariable, SetEnvironmentVariable


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
    """Acciones de launch que exponen los modelos y el server.config de PX4 a Gazebo.

    Hay que ponerlas ANTES de la accion que arranca gz. La carpeta se toma de PX4_GZ_DIR
    (si se quiere apuntar a otra copia de los modelos, p. ej. un clon de PX4-gazebo-models)
    o, por defecto, de <PX4_DIR>/Tools/simulation/gz.

    - GZ_SIM_RESOURCE_PATH se AGREGA (append): ros_gz ya pone ahi /opt/ros/<distro>/share.
    - GZ_SIM_SERVER_CONFIG_PATH solo se define si el usuario no lo habia fijado.
    """
    gz_dir = os.environ.get('PX4_GZ_DIR') or os.path.join(find_px4(), 'Tools', 'simulation', 'gz')
    models_dir = os.path.join(gz_dir, 'models')
    server_config = os.path.join(gz_dir, 'server.config')

    if not os.path.isdir(models_dir):
        print(f"[WARNING] PX4 Gazebo models not found at {models_dir}. "
              "Set PX4_DIR (PX4 root) or PX4_GZ_DIR (folder with models/ and server.config).")
        return []

    actions = []
    if models_dir not in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(os.pathsep):
        actions.append(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_dir))
    if 'GZ_SIM_SERVER_CONFIG_PATH' not in os.environ:
        if os.path.isfile(server_config):
            actions.append(SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', server_config))
        else:
            print(f"[WARNING] PX4 server.config not found at {server_config}.")

    print(f"[DEBUG] PX4 Gazebo assets from: {gz_dir}")
    return actions
